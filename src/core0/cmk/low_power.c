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
#include "connection_manager.h"   // connection_manager_request_radio (DEEP radio gating)
#include "usb/usb_hid.h"          // usb_hid_is_active — inhibit DEEP on USB
#include "config/cmk_config.h"
#include "config/debug_config.h"

#if defined(RGB_MATRIX_ENABLE)
#include "rgb_matrix.h"
#endif

#ifdef LOW_POWER_ENABLE

// Idle tracking. s_last_activity is the tick of the most recent key event;
// s_state is the current tier. Both live on Core 0 only (keyscan task).
static uint32_t   s_last_activity = 0;
static lp_state_t s_state         = LP_ACTIVE;

// LED suspend is active in both LIGHT and DEEP; toggle the rgb_matrix flag only
// on the ACTIVE↔(LIGHT|DEEP) boundary.
static inline bool suspended_in(lp_state_t s) { return s != LP_ACTIVE; }

// DEEP enter/exit (BLE radio power) — wired in Stage 4 (connection_manager +
// IPC). Kept as hooks so the state machine and LED/throttle behaviour land
// independently of the radio path.
static void enter_deep(void);
static void exit_deep(void);

static void set_state(lp_state_t s) {
    if (s == s_state) return;
    lp_state_t old = s_state;
    s_state = s;

#if defined(RGB_MATRIX_ENABLE)
    if (suspended_in(s) != suspended_in(old)) {
        rgb_matrix_set_suspended(suspended_in(s));
    }
#endif

    if (s == LP_DEEP)          enter_deep();
    else if (old == LP_DEEP)   exit_deep();

    DBG_POWER("[LP] state %d → %d\n", (int)old, (int)s);
}

void low_power_init(void) {
    s_last_activity = 0;
    s_state         = LP_ACTIVE;
}

void low_power_note_activity(uint32_t now) {
    s_last_activity = now;
    if (s_state != LP_ACTIVE) {
        set_state(LP_ACTIVE);
    }
}

void low_power_tick(uint32_t now) {
    uint32_t idle = now - s_last_activity;   // tick wrap is harmless (both uint32)

    lp_state_t want;
    if      (idle >= LP_DEEP_IDLE_MS)  want = LP_DEEP;
    else if (idle >= LP_LIGHT_IDLE_MS) want = LP_LIGHT;
    else                               want = LP_ACTIVE;

    // Never enter DEEP while USB is connected: we are mains-powered (deep sleep
    // pointless) and the DEEP wake reboots the chip, which would re-enumerate USB
    // on every keypress. Cap at LIGHT — LEDs/throttle still save power.
    if (want == LP_DEEP && usb_hid_is_active()) want = LP_LIGHT;

    if (want != s_state) set_state(want);
}

lp_state_t low_power_state(void) {
    return s_state;
}

uint32_t low_power_scan_interval_ms(void) {
    return (s_state == LP_ACTIVE) ? KS_SCAN_INTERVAL_MS : LP_LIGHT_SCAN_MS;
}

bool low_power_deep_requested(void) {
    return s_state == LP_DEEP;
}

// --- DEEP radio gating ------------------------------------------------------
// Power the BLE radio down on DEEP entry and back up on exit. The keyscan stays
// on the throttled LIGHT cadence (LP_LIGHT_SCAN_MS) in DEEP too, so a keypress
// is still detected (and snaps back to ACTIVE) within one throttled scan — no
// GPIO-IRQ keyscan suspend is used, because without tickless idle the 1 kHz
// SysTick wakes the core every ms regardless, so suspending the scan would save
// nothing. (GPIO-IRQ wake belongs with the deferred tickless/clock-scaling step.)
static void enter_deep(void) {
    connection_manager_request_radio(false);
}
static void exit_deep(void) {
    connection_manager_request_radio(true);
}

#else  // !LOW_POWER_ENABLE — inert stubs

void       low_power_init(void)                  { }
void       low_power_note_activity(uint32_t now) { (void)now; }
void       low_power_tick(uint32_t now)          { (void)now; }
lp_state_t low_power_state(void)                 { return LP_ACTIVE; }
uint32_t   low_power_scan_interval_ms(void)      { return KS_SCAN_INTERVAL_MS; }
bool       low_power_deep_requested(void)        { return false; }

#endif // LOW_POWER_ENABLE
