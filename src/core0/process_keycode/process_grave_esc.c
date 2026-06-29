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
// process_grave_esc.c — Grave Escape: ESC or ` based on modifiers (Phase 10)
//
// CMK_GESC pressed:
//   - Shift or GUI active → sends KC_GRV (`)
//   - Otherwise           → sends KC_ESC
//

#include "process_grave_esc.h"
#include "keycode.h"
#include "action_util.h"
#include "config/debug_config.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint8_t s_ge_sent;  // KC_ESC or KC_GRV (0 = idle)

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void grave_esc_init(void) {
    s_ge_sent = 0;
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

// Shift + GUI bitmask (left + right)
#define MODS_SHIFT_GUI  (MOD_BIT(KC_LSFT) | MOD_BIT(KC_RSFT) | \
                         MOD_BIT(KC_LGUI) | MOD_BIT(KC_RGUI))

bool process_grave_esc(const keyevent_t *event) {
    if (event->keycode != CMK_GESC) return true;  // pass through

    if (event->pressed) {
        uint8_t mods = get_real_mods();
        if (mods & MODS_SHIFT_GUI) {
            // Shift or GUI held → send grave (`)
            s_ge_sent = KC_GRV;
        } else {
            // No relevant mods → send ESC
            s_ge_sent = KC_ESC;
        }
        register_code(s_ge_sent);
        DBG_KEY("[GESC] press → kc=0x%02x\n", s_ge_sent);
    } else {
        if (s_ge_sent) {
            unregister_code(s_ge_sent);
            s_ge_sent = 0;
        }
    }
    return false;  // consumed
}
