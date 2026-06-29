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
// process_secure.c — Secure: lock/unlock keyboard (Phase 11)
//
// When locked, send_keyboard_report() and mouse reports are suppressed.
// Unlock via: CMK_SECURE_UNLOCK keycode, or matching unlock sequence.
//

#include "process_secure.h"
#include "keycode.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

// ---------------------------------------------------------------------------
// Unlock sequence (weak — user can override)
// ---------------------------------------------------------------------------

__attribute__((weak))
const uint8_t secure_unlock_sequence[][2] = {
    {0, 0}, {0, 1}, {0, 2}, {0, 3}   // default: first 4 keys in row 0
};

__attribute__((weak))
const uint8_t secure_unlock_sequence_len = 4;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool    s_secure_locked;
static uint8_t s_unlock_progress;

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void secure_init(void) {
    s_secure_locked   = false;
    s_unlock_progress = 0;
}

// ---------------------------------------------------------------------------
// Query
// ---------------------------------------------------------------------------

bool secure_is_locked(void) {
    return s_secure_locked;
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_secure(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // Handle secure keycodes
    if (IS_SECURE(kc)) {
        if (!event->pressed) return false;  // consume release

        switch (kc) {
            case CMK_SECURE_LOCK:
                s_secure_locked = true;
                s_unlock_progress = 0;
                DBG_KEY("[SECURE] LOCKED\n");
                break;
            case CMK_SECURE_UNLOCK:
                s_secure_locked = false;
                s_unlock_progress = 0;
                DBG_KEY("[SECURE] UNLOCKED\n");
                break;
            case CMK_SECURE_TOGGLE:
                s_secure_locked = !s_secure_locked;
                s_unlock_progress = 0;
                DBG_KEY("[SECURE] %s\n", s_secure_locked ? "LOCKED" : "UNLOCKED");
                break;
            default:
                break;
        }
        return false;  // consumed
    }

    // While locked, check unlock sequence on press
    if (s_secure_locked && event->pressed) {
        if (event->row == secure_unlock_sequence[s_unlock_progress][0] &&
            event->col == secure_unlock_sequence[s_unlock_progress][1]) {
            s_unlock_progress++;
            if (s_unlock_progress >= secure_unlock_sequence_len) {
                s_secure_locked = false;
                s_unlock_progress = 0;
                DBG_KEY("[SECURE] unlock sequence complete — UNLOCKED\n");
            }
        } else {
            s_unlock_progress = 0;  // mismatch — reset
        }
    }

    return true;  // pass through (HID suppression is in send_keyboard_report)
}
