/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
 *
 * CMK is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation, version 2.
 *
 * CMK is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program. If not, see
 * <https://www.gnu.org/licenses/old-licenses/gpl-2.0.html>.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

//
// low_power.c — see low_power.h for the design.
//

#include "low_power.h"
#include "eeconfig.h"            // runtime LP timeouts (bug #2)
#include "config/cmk_config.h"
#include "config/debug_config.h"
#include "ipc/ipc_core.h"
#include "pico/time.h"
#ifdef USB_ENABLE
#include "usb/usb_hid.h"
#endif
#if defined(RGB_MATRIX_ENABLE)
#include "rgb_matrix.h"
#endif

#ifdef LOW_POWER_ENABLE
// Core 0 keyscan owns policy/tier; all shared state crosses the IPC lock.
static power_local_t s_local;
static lp_state_t s_state;
static bool s_reboot;
static uint64_t s_retry_after;
// Runtime idle thresholds in ms (bug #2), from eeconfig (seconds x1000) at init
// and on low_power_refresh_config() — called by the config protocol from the
// USB/ih_link task: aligned 32-bit stores, benign cross-task race. 0 = tier off.
static volatile uint32_t s_light_idle_ms = LP_LIGHT_IDLE_MS;
static volatile uint32_t s_deep_idle_ms  = LP_DEEP_IDLE_MS;
static void set_state(lp_state_t state) {
    if (state == s_state) return;
#if defined(RGB_MATRIX_ENABLE)
    if ((state == LP_ACTIVE) != (s_state == LP_ACTIVE))
        rgb_matrix_set_suspended(state != LP_ACTIVE);
#endif
    DBG_STATUS("[LP] tier %u -> %u\n", (unsigned)s_state, (unsigned)state);
    s_state=state;
}
void low_power_init(void) {
    s_local=(power_local_t){0}; s_state=LP_ACTIVE; s_reboot=false; s_retry_after=0;
    low_power_refresh_config();   // main.c runs eeconfig_init() before action_init()
}
void low_power_refresh_config(void) {
    s_light_idle_ms = (uint32_t)eeconfig_read_lp_light_s() * 1000u;
    s_deep_idle_ms  = (uint32_t)eeconfig_read_lp_deep_s()  * 1000u;
    // Core 1 has no EEPROM, so it reads the window from here — but publish it on
    // the NEXT tick, never from this function: low_power_init() runs inside
    // action_init(), which main.c executes BEFORE ipc_init(). Calling
    // ipc_power_local() here takes an uninitialised spinlock and hangs the boot
    // before the tasks start, before USB and before Core 1 is launched — the
    // board goes mute on USB *and* on BLE (b025 did exactly this).
    // low_power_tick() and low_power_note_activity() publish s_local anyway, so
    // the value reaches Core 1 within one tick: nothing can decide DEEP before.
    s_local.deep_ms = s_deep_idle_ms;
    DBG_STATUS("[LP] timeout runtime: light=%lu ms deep=%lu ms (0 = disabilitato)\n",
               (unsigned long)s_light_idle_ms, (unsigned long)s_deep_idle_ms);
}
void low_power_note_activity(uint32_t ticks) {
    (void)ticks; s_local.key_us=time_us_64();
    s_local.sample_us=s_local.key_us; s_local.ready=true; ipc_power_local(&s_local);
}
void low_power_note_remote_activity(uint32_t ticks) {
    (void)ticks; s_local.remote_key_us=time_us_64();
    s_local.sample_us=s_local.remote_key_us; s_local.ready=true; ipc_power_local(&s_local);
}
void low_power_update_held(bool held) { s_local.held=held; }
void low_power_tick(uint32_t ticks) {
    (void)ticks;
    uint64_t now=time_us_64();
    s_local.sample_us=now; s_local.ready=true;
#ifdef USB_ENABLE
    s_local.usb=usb_hid_is_active();
#endif
    ipc_power_local(&s_local);
    power_state_t p; ipc_power_get(&p);
    // There is no radio resume after commit. A new key/USB or stalled teardown
    // requires the existing clean-reboot path, never a fictitious ACTIVE state.
    if (p.phase >= POWER_COMMITTED) {
        s_reboot=s_local.usb || s_local.held || s_local.key_us > p.commit_us ||
                 s_local.remote_key_us > p.commit_us ||
                 (p.phase==POWER_COMMITTED && now-p.commit_us >= 3000000);
        set_state(p.phase==POWER_OFF ? LP_DEEP : LP_LIGHT);
        return;
    }
    // Runtime thresholds (bug #2): 0 = tier disabled. A zero DEEP window must
    // NEVER reach power_veto()/power_request(): with window = 0 every recent()
    // test is false, so the veto would come back clean and the radio would be
    // powered off immediately — the exact opposite of "disabled".
    uint32_t deep_ms  = s_deep_idle_ms;
    uint32_t light_ms = s_light_idle_ms;
    if (deep_ms == 0) {
        if (p.phase==POWER_REQUESTED) {   // disabled mid-window: withdraw
            ipc_power_cancel();
            s_retry_after=now+1000000;
            DBG_STATUS("[LP] DEEP disabilitato: request %u annullata\n",p.token);
        }
    } else {
        uint8_t veto=power_veto(&p,now,deep_ms);
        if (p.phase==POWER_REQUESTED && (veto || now-p.request_us >= 3000000)) {
            ipc_power_cancel();
            s_retry_after=now+1000000;
            DBG_STATUS("[LP] cancel request %u veto=0x%02x\n",p.token,veto);
        } else if (p.phase==POWER_RUNNING && !veto && now>=s_retry_after) {
            uint16_t id=ipc_power_request(now,deep_ms);
            if (id) DBG_STATUS("[LP] request %u\n",id);
        }
    }
    uint64_t key=power_last_key(&p);
    bool light=light_ms && !s_local.held && now>=key &&
               now-key >= (uint64_t)light_ms*1000;
    set_state(light ? LP_LIGHT : LP_ACTIVE);
}
lp_state_t low_power_state(void) { return s_state; }
uint32_t low_power_scan_interval_ms(void) { return s_state==LP_ACTIVE ? KS_SCAN_INTERVAL_MS : LP_LIGHT_SCAN_MS; }
bool low_power_deep_requested(void) { return s_state==LP_DEEP; }
bool low_power_reboot_needed(void) { return s_reboot; }
#else
void low_power_init(void) {}
void low_power_refresh_config(void) {}
void low_power_note_activity(uint32_t t) { (void)t; }
void low_power_note_remote_activity(uint32_t t) { (void)t; }
void low_power_update_held(bool held) { (void)held; }
void low_power_tick(uint32_t t) { (void)t; }
lp_state_t low_power_state(void) { return LP_ACTIVE; }
uint32_t low_power_scan_interval_ms(void) { return KS_SCAN_INTERVAL_MS; }
bool low_power_deep_requested(void) { return false; }
bool low_power_reboot_needed(void) { return false; }
#endif
