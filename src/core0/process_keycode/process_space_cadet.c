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
// process_space_cadet.c — Space Cadet: Shift tap → parenthesis (Phase 10)
//
// SC_LSPO: hold = Left Shift,  tap = (
// SC_RSPC: hold = Right Shift, tap = )
// SC_LCPO: hold = Left Ctrl,   tap = (
// SC_RCPC: hold = Right Ctrl,  tap = )
//
// Uses its own tap/hold state machine (cannot reuse action_tapping which
// only handles IS_MOD_TAP / IS_LAYER_TAP keycodes).
//

#include "process_space_cadet.h"
#include "keycode.h"
#include "action_util.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

typedef struct {
    bool     active;
    uint8_t  mod_kc;       // KC_LSFT / KC_RSFT / KC_LCTL / KC_RCTL
    uint8_t  tap_kc;       // KC_9 (for '(') or KC_0 (for ')')
    uint8_t  row, col;
    uint32_t press_time;
    bool     interrupted;  // another key pressed during hold
} sc_state_t;

static sc_state_t s_sc;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static bool sc_decode(uint16_t kc, uint8_t *mod_out, uint8_t *tap_out) {
    switch (kc) {
        case SC_LSPO: *mod_out = KC_LSFT; *tap_out = KC_9; return true;
        case SC_RSPC: *mod_out = KC_RSFT; *tap_out = KC_0; return true;
        case SC_LCPO: *mod_out = KC_LCTL; *tap_out = KC_9; return true;
        case SC_RCPC: *mod_out = KC_RCTL; *tap_out = KC_0; return true;
        default: return false;
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void space_cadet_init(void) {
    s_sc.active = false;
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_space_cadet(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // If tracking, check for interruption by another key press
    if (s_sc.active && event->pressed &&
        (event->row != s_sc.row || event->col != s_sc.col)) {
        s_sc.interrupted = true;
        return true;  // pass through (let the interrupting key be processed)
    }

    // Decode Space Cadet keycode
    uint8_t mod_kc, tap_kc;
    if (!sc_decode(kc, &mod_kc, &tap_kc)) return true;  // not a SC key

    if (event->pressed) {
        s_sc.active      = true;
        s_sc.mod_kc      = mod_kc;
        s_sc.tap_kc      = tap_kc;
        s_sc.row         = event->row;
        s_sc.col         = event->col;
        s_sc.press_time  = event->timestamp;
        s_sc.interrupted = false;

        // Immediately register the modifier (so it's active during hold)
        register_mods(MOD_BIT(mod_kc));
        DBG_KEY("[SC] press mod=0x%02x\n", mod_kc);
        return false;  // consumed
    }

    // Release
    if (!s_sc.active) return false;  // stale release

    // Check if same position
    if (event->row != s_sc.row || event->col != s_sc.col) return true;

    TickType_t elapsed = (TickType_t)(event->timestamp - s_sc.press_time);
    bool is_tap = (elapsed < pdMS_TO_TICKS(SPACE_CADET_TERM)) && !s_sc.interrupted;

    // Unregister the modifier
    unregister_mods(MOD_BIT(s_sc.mod_kc));

    if (is_tap) {
        // Tap → send the parenthesis (Shift + 9 or Shift + 0) via the tap queue:
        // a same-tick register+unregister nets to nothing in the once-per-tick
        // report (the lost-tap bug class — same fix as MT/LT and Auto Shift).
        tap_code_mods(s_sc.tap_kc, MOD_BIT(KC_LSFT));
        DBG_KEY("[SC] tap → kc=0x%02x (shifted, queued)\n", s_sc.tap_kc);
    } else {
        DBG_KEY("[SC] hold released\n");
    }

    s_sc.active = false;
    return false;  // consumed
}

// ---------------------------------------------------------------------------
// Tick — force hold if timeout expires while still pressed
// ---------------------------------------------------------------------------

void space_cadet_tick(uint32_t now_ticks) {
    if (!s_sc.active || s_sc.interrupted) return;

    TickType_t elapsed = (TickType_t)(now_ticks - s_sc.press_time);
    if (elapsed >= pdMS_TO_TICKS(SPACE_CADET_TERM)) {
        s_sc.interrupted = true;  // force hold resolution on release
    }
}
