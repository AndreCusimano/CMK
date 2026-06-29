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
// process_caps_word.c — Caps Word: auto-shift while typing (Phase 9)
//
// Toggle via CMK_CAPS_WORD keycode. While active, alphabetic keys are
// shifted automatically. Continues on numbers, hyphen, underscore,
// backspace, and modifiers. Deactivates on any other key or idle timeout.
//

#include "process_caps_word.h"
#include "keycode.h"
#include "action_util.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool     s_caps_word_on;
static uint32_t s_caps_word_timer;   // tick of last activity

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void caps_word_init(void) {
    s_caps_word_on    = false;
    s_caps_word_timer = 0;
}

bool caps_word_is_active(void) {
    return s_caps_word_on;
}

// ---------------------------------------------------------------------------
// Helper: should this keycode keep Caps Word alive?
// ---------------------------------------------------------------------------

static bool is_caps_word_continue(uint16_t kc) {
    // Alpha keys (will be shifted)
    if (kc >= KC_A && kc <= KC_Z) return true;
    // Number keys
    if (kc >= KC_1 && kc <= KC_0) return true;
    // Hyphen, underscore (Shift+hyphen), backspace
    if (kc == KC_MINS || kc == KC_BSPC) return true;
    // Modifier keys
    if (IS_HID_MOD(kc)) return true;
    return false;
}

// ---------------------------------------------------------------------------
// Process record
// ---------------------------------------------------------------------------

bool process_caps_word(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // Toggle keycode
    if (kc == CMK_CAPS_WORD) {
        if (event->pressed) {
            s_caps_word_on = !s_caps_word_on;
            s_caps_word_timer = event->timestamp;
            DBG_KEY("[CAPSW] %s\n", s_caps_word_on ? "ON" : "OFF");
        }
        return false;  // consumed
    }

    // Not active → pass through
    if (!s_caps_word_on) return true;

    // Release of an alpha: drop OUR weak shift (added at its press below). Before
    // R2 nothing ever cleared it → it leaked into later reports ('1' typed '!')
    // and stayed latched until some other feature cleared weak mods globally.
    if (!event->pressed) {
        if (IS_BASIC(kc) && kc >= KC_A && kc <= KC_Z) {
            del_weak_mods(MOD_BIT(KC_LSFT));
        }
        return true;
    }

    // Only process basic keycodes
    if (!IS_BASIC(kc)) return true;

    if (kc >= KC_A && kc <= KC_Z) {
        // Alpha → add Shift
        add_weak_mods(MOD_BIT(KC_LSFT));
        s_caps_word_timer = event->timestamp;
        return true;  // continue chain (register_code will fire)
    }

    if (is_caps_word_continue(kc)) {
        // Continue without shift
        s_caps_word_timer = event->timestamp;
        return true;
    }

    // Any other key → deactivate
    s_caps_word_on = false;
    DBG_KEY("[CAPSW] OFF (non-continue key 0x%02x)\n", kc);
    return true;  // pass through the key
}

// ---------------------------------------------------------------------------
// Tick — idle timeout
// ---------------------------------------------------------------------------

void caps_word_tick(uint32_t now_ticks) {
    if (!s_caps_word_on) return;

    TickType_t elapsed = (TickType_t)(now_ticks - s_caps_word_timer);
    if (elapsed >= pdMS_TO_TICKS(CAPS_WORD_IDLE_TIMEOUT)) {
        s_caps_word_on = false;
        DBG_KEY("[CAPSW] OFF (idle timeout)\n");
    }
}
