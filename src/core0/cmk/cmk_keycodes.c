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
// cmk_keycodes.c — CMK custom keycode handlers (Phase 7)
//
// Long-press state machine for BLE slot combos (CMK_TD0-2, CMK_TU).
// Same behavior as the old keyscan.c combo logic, but driven by the
// process_record chain instead of raw matrix positions.
//

#include "cmk_keycodes.h"
#include "keycode.h"
#include "connection_manager.h"
#include "bonding_store.h"
#include "output_select.h"
#include "inter_half_sync.h"
#include "eeconfig.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include "hardware/watchdog.h"

#include <stdio.h>

// ---------------------------------------------------------------------------
// Long-press state for Td{0,1,2} and Tu
// ---------------------------------------------------------------------------

static bool       s_td_active[3];
static uint32_t   s_td_press_tick[3];
static bool       s_td_long_fired[3];

static bool       s_tu_active;
static uint32_t   s_tu_press_tick;
static bool       s_tu_long_fired;

static bool       s_ee_clr_active;
static uint32_t   s_ee_clr_press_tick;
static bool       s_ee_clr_long_fired;

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void cmk_keycodes_init(void) {
    for (int i = 0; i < 3; i++) {
        s_td_active[i]      = false;
        s_td_press_tick[i]  = 0;
        s_td_long_fired[i]  = false;
    }
    s_tu_active       = false;
    s_tu_press_tick   = 0;
    s_tu_long_fired   = false;
    s_ee_clr_active       = false;
    s_ee_clr_press_tick   = 0;
    s_ee_clr_long_fired   = false;
}

// ---------------------------------------------------------------------------
// Td{slot} handler — DS slot directed/general ADV
// ---------------------------------------------------------------------------

static bool handle_td(int slot, bool pressed, uint32_t timestamp) {
    if (pressed) {
        s_td_active[slot]      = true;
        s_td_press_tick[slot]  = timestamp;
        s_td_long_fired[slot]  = false;
        // Selecting a BLE slot routes HID output to BLE (keeps USB cable attached).
        output_select_set(OUTPUT_BLE);
        DBG_KEY("[CMK] Td%d pressed\n", slot);
    } else {
        if (s_td_active[slot] && !s_td_long_fired[slot]) {
            // Short press → directed ADV
            bool ok = connection_manager_ds_adv_directed((uint8_t)slot);
            DBG_KEY("[CMK] Td%d short → directed ADV slot %d: %s\n",
                    slot, slot, ok ? "ok" : "skipped");
        }
        s_td_active[slot] = false;
    }
    return false;  // consumed
}

// ---------------------------------------------------------------------------
// Tu handler — US scan
// ---------------------------------------------------------------------------

static bool handle_tu(bool pressed, uint32_t timestamp) {
    if (pressed) {
        s_tu_active     = true;
        s_tu_press_tick = timestamp;
        s_tu_long_fired = false;
        DBG_KEY("[CMK] Tu pressed\n");
    } else {
        if (!s_tu_long_fired) {
            // Mirror of Td short: manual upstream re-link (e.g. after a real
            // disconnect or the rapid-disconnect backoff stop).
            connection_manager_us_reconnect();
            DBG_KEY("[CMK] Tu short → reconnect/scan upstream\n");
        }
        s_tu_active = false;
    }
    return false;  // consumed
}

// ---------------------------------------------------------------------------
// EE_CLR handler — bonding factory reset
// ---------------------------------------------------------------------------

static bool handle_ee_clr(bool pressed, uint32_t timestamp) {
    if (pressed) {
        s_ee_clr_active     = true;
        s_ee_clr_press_tick = timestamp;
        s_ee_clr_long_fired = false;
        DBG_KEY("[CMK] EE_CLR pressed\n");
    } else {
        if (!s_ee_clr_long_fired) {
            DBG_KEY("[CMK] EE_CLR short → nothing\n");
        }
        s_ee_clr_active = false;
    }
    return false;  // consumed
}

// ---------------------------------------------------------------------------
// Process record — dispatch CMK keycodes
// ---------------------------------------------------------------------------

bool process_record_cmk(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    if (!IS_CMK_KEY(kc)) return true;  // not ours

    switch (kc) {
    case CMK_TD0:  return handle_td(0, event->pressed, event->timestamp);
    case CMK_TD1:  return handle_td(1, event->pressed, event->timestamp);
    case CMK_TD2:  return handle_td(2, event->pressed, event->timestamp);
    case CMK_TU:   return handle_tu(event->pressed, event->timestamp);
    case CMK_EE_CLR: return handle_ee_clr(event->pressed, event->timestamp);

    case CMK_OUT_USB:
        if (event->pressed) {
            output_select_set(OUTPUT_USB);
            DBG_KEY("[CMK] OUT_USB → output=USB\n");
        }
        return false;

    case CMK_OUT_BLE:
        if (event->pressed) {
            output_select_set(OUTPUT_BLE);
            DBG_KEY("[CMK] OUT_BLE → output=BLE\n");
        }
        return false;

    case CMK_RESET:
        if (event->pressed) {
            DBG_KEY("[CMK] RESET → rebooting\n");
            watchdog_reboot(0, 0, 0);
        }
        return false;

    case CMK_CFG_PUSH:
        if (event->pressed) {
            DBG_KEY("[CMK] CFG_PUSH → inter-half push\n");
            inter_half_sync_request_push();
        }
        return false;

    case CMK_CFG_RECV:
        if (event->pressed) {
            DBG_KEY("[CMK] CFG_RECV → arm inter-half receive\n");
            inter_half_sync_arm_receive();
        }
        return false;

    case CMK_SPLIT_TOGG:
        if (event->pressed) {
            uint8_t on = eeconfig_read_split_mode() ? 0 : 1;
            eeconfig_update_split_mode(on);
            DBG_KEY("[CMK] SPLIT_TOGG → split_mode=%d\n", on);
        }
        return false;

    // Future no-ops — return false to consume silently
    case CMK_RGB_TOG:
    case CMK_RGB_INC:
    case CMK_RGB_DEC:
    case CMK_RGB_MOD:
        return false;

    default:
        return false;
    }
}

// ---------------------------------------------------------------------------
// Tick — poll long-press timers
// ---------------------------------------------------------------------------

void cmk_keycodes_tick(uint32_t now_ticks) {
    TickType_t threshold = pdMS_TO_TICKS(KS_LONG_PRESS_MS);

    // Td{0,1,2} long press → clear + general ADV
    for (int x = 0; x < 3; x++) {
        if (s_td_active[x] && !s_td_long_fired[x]) {
            if ((uint32_t)(now_ticks - s_td_press_tick[x]) >= threshold) {
                s_td_long_fired[x] = true;
                connection_manager_ds_adv_general((uint8_t)x);
                DBG_KEY("[CMK] Td%d long %dms → clear + general ADV slot %d\n",
                        x, KS_LONG_PRESS_MS, x);
            }
        }
    }

    // Tu long press → clear US + general scan
    if (s_tu_active && !s_tu_long_fired) {
        if ((uint32_t)(now_ticks - s_tu_press_tick) >= threshold) {
            s_tu_long_fired = true;
            connection_manager_us_scan_general();
            DBG_KEY("[CMK] Tu long %dms → clear US + general scan\n", KS_LONG_PRESS_MS);
        }
    }

    // EE_CLR long press → factory reset
    if (s_ee_clr_active && !s_ee_clr_long_fired) {
        if ((uint32_t)(now_ticks - s_ee_clr_press_tick) >= threshold) {
            s_ee_clr_long_fired = true;
            // Factory reset — clears every bond record AND its SM keys. Deferred to the
            // CM task: the EEPROM clears and the IPC_DELETE_BONDING push must not run from
            // the keyscan task (258-byte stack buffer + FIFO ownership rules).
            connection_manager_clear_all_bonds();
            DBG_KEY("[CMK] EE_CLR long %dms → factory reset (all bonds + SM keys)\n",
                    KS_LONG_PRESS_MS);
        }
    }
}
