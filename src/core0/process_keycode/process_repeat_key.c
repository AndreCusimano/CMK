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
// process_repeat_key.c — Repeat Key: replay last keycode (Phase 9)
//
// CMK_REP: repeats the last non-modifier keycode pressed.
// CMK_AREP: fires the user-defined alternate for the last keycode.
//

#include "process_repeat_key.h"
#include "keycode.h"
#include "action_util.h"
#include "config/debug_config.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint16_t s_last_keycode;   // last pressed basic/modkey keycode
static uint8_t  s_last_mods;      // real_mods active at that time

// What CMK_REP actually registered at press (R2): s_last_* can be overwritten by
// keys typed WHILE CMK_REP is held — the release must undo what was pressed, not
// whatever is "last" at release time.
static uint8_t  s_rep_held_kc;
static uint8_t  s_rep_held_mods;

// ---------------------------------------------------------------------------
// Weak default: alternate repeat (user can override)
// ---------------------------------------------------------------------------

__attribute__((weak))
uint16_t get_alt_repeat_keycode(uint16_t last_kc) {
    (void)last_kc;
    return KC_NO;  // no alternate by default
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void repeat_key_init(void) {
    s_last_keycode = KC_NO;
    s_last_mods    = 0;
}

// ---------------------------------------------------------------------------
// Process record
// ---------------------------------------------------------------------------

bool process_repeat_key(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // --- Handle CMK_REP ---
    if (kc == CMK_REP) {
        if (s_last_keycode == KC_NO) return false;  // nothing to repeat

        if (event->pressed) {
            s_rep_held_kc   = (uint8_t)(s_last_keycode & 0xFF);
            s_rep_held_mods = s_last_mods;
            add_weak_mods(s_rep_held_mods);
            register_code(s_rep_held_kc);
            DBG_KEY("[REP] repeat kc=0x%04x mods=0x%02x\n",
                    s_last_keycode, s_last_mods);
        } else {
            unregister_code(s_rep_held_kc);
            del_weak_mods(s_rep_held_mods);   // only our bits
            s_rep_held_kc   = 0;
            s_rep_held_mods = 0;
        }
        return false;  // consumed
    }

    // --- Handle CMK_AREP ---
    if (kc == CMK_AREP) {
        uint16_t alt_kc = get_alt_repeat_keycode(s_last_keycode);
        if (alt_kc == KC_NO) return false;  // no alternate defined

        if (event->pressed) {
            register_code((uint8_t)(alt_kc & 0xFF));
            DBG_KEY("[AREP] alt kc=0x%04x for last=0x%04x\n",
                    alt_kc, s_last_keycode);
        } else {
            unregister_code((uint8_t)(alt_kc & 0xFF));
        }
        return false;  // consumed
    }

    // --- Track last keycode (only on press, only basic/modkey, skip modifiers) ---
    if (event->pressed && !IS_HID_MOD(kc) && !IS_CMK_KEY(kc) &&
        !IS_LAYER_KEY(kc) && !IS_TAP_DANCE(kc) && !IS_MOUSEKEY(kc)) {
        if (IS_BASIC(kc) || IS_MOD_KEY(kc)) {
            s_last_keycode = kc;
            s_last_mods    = get_real_mods();
        }
    }

    return true;  // not consumed
}
