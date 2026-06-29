/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
 * Copyright (C) 2016-2026 QMK Firmware contributors
 * Copyright (C) Jun Wako and TMK Keyboard Firmware contributors
 *
 * This file contains code derived from the QMK Firmware project
 * (https://github.com/qmk/qmk_firmware) and, indirectly, from the
 * TMK Keyboard Firmware by Jun Wako, both distributed under the
 * GNU General Public License version 2.
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
// process_key_lock.c — Key Lock: lock a key pressed (Phase 11)
//
// CMK_KEY_LOCK → next key pressed stays held (release ignored).
// Press the same key again to unlock.
//

#include "process_key_lock.h"
#include "keycode.h"
#include "action_util.h"
#include "config/debug_config.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint16_t s_kl_keycode;  // locked keycode (KC_NO = none)
static bool     s_kl_waiting;  // waiting for next key to lock

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void key_lock_init(void) {
    s_kl_keycode = KC_NO;
    s_kl_waiting = false;
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_key_lock(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // Handle CMK_KEY_LOCK keycode
    if (kc == CMK_KEY_LOCK) {
        if (event->pressed) {
            if (s_kl_keycode != KC_NO) {
                // Already locked — unlock
                DBG_KEY("[KLOCK] unlock 0x%04x\n", s_kl_keycode);
                unregister_code((uint8_t)s_kl_keycode);
                s_kl_keycode = KC_NO;
            } else {
                // Arm: next keypress will be locked
                s_kl_waiting = true;
                DBG_KEY("[KLOCK] armed — waiting for key\n");
            }
        }
        return false;  // consumed
    }

    // Waiting for a key to lock
    if (s_kl_waiting && event->pressed && IS_BASIC(kc)) {
        s_kl_keycode = kc;
        s_kl_waiting = false;
        // Don't consume — let the key be registered normally
        // (the release will be suppressed by key_lock_should_suppress)
        DBG_KEY("[KLOCK] locked 0x%04x\n", kc);
        return true;  // pass through (key registers normally)
    }

    return true;  // pass through
}

// ---------------------------------------------------------------------------
// Suppress — called from action_exec on release
// ---------------------------------------------------------------------------

bool key_lock_should_suppress(uint16_t keycode, bool pressed) {
    if (!pressed && keycode == s_kl_keycode && s_kl_keycode != KC_NO) {
        // Suppress release of the locked key
        return true;
    }
    return false;
}
