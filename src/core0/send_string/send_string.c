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
// send_string.c — Send String: non-blocking character sequence output (Phase 9)
//
// Sends one character per tick interval (SEND_STRING_DELAY ms).
// Uses a US ANSI ASCII→keycode LUT (128 entries in flash .rodata).
// Each tick cycle: press → wait → release → wait → next char.
//

#include "send_string.h"
#include "keycode.h"
#include "action_util.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"

// ---------------------------------------------------------------------------
// ASCII → (keycode | shift flag) LUT for US ANSI layout
// ---------------------------------------------------------------------------
// Bit 15 = needs Shift. Bits 7:0 = HID keycode. 0 = unsupported.

#define _S(kc) (0x8000 | (kc))   // mark as shifted

static const uint16_t ascii_to_keycode[128] = {
    // 0x00-0x0F: control chars (unsupported)
    0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
    // 0x10-0x1F: control chars (unsupported)
    0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,
    // 0x20-0x2F:  space ! " # $ % & ' ( ) * + , - . /
    KC_SPC,  _S(KC_1),  _S(KC_QUOT), _S(KC_3),
    _S(KC_4), _S(KC_5), _S(KC_7),    KC_QUOT,
    _S(KC_9), _S(KC_0), _S(KC_8),    _S(KC_EQL),
    KC_COMM,  KC_MINS,  KC_DOT,      KC_SLSH,
    // 0x30-0x39: 0-9
    KC_0, KC_1, KC_2, KC_3, KC_4, KC_5, KC_6, KC_7, KC_8, KC_9,
    // 0x3A-0x3F: : ; < = > ?
    _S(KC_SCLN), KC_SCLN, _S(KC_COMM), KC_EQL, _S(KC_DOT), _S(KC_SLSH),
    // 0x40: @
    _S(KC_2),
    // 0x41-0x5A: A-Z (uppercase — shifted)
    _S(KC_A), _S(KC_B), _S(KC_C), _S(KC_D), _S(KC_E), _S(KC_F), _S(KC_G),
    _S(KC_H), _S(KC_I), _S(KC_J), _S(KC_K), _S(KC_L), _S(KC_M), _S(KC_N),
    _S(KC_O), _S(KC_P), _S(KC_Q), _S(KC_R), _S(KC_S), _S(KC_T), _S(KC_U),
    _S(KC_V), _S(KC_W), _S(KC_X), _S(KC_Y), _S(KC_Z),
    // 0x5B-0x60: [ \ ] ^ _ `
    KC_LBRC, KC_BSLS, KC_RBRC, _S(KC_6), _S(KC_MINS), KC_GRV,
    // 0x61-0x7A: a-z (lowercase — unshifted)
    KC_A, KC_B, KC_C, KC_D, KC_E, KC_F, KC_G,
    KC_H, KC_I, KC_J, KC_K, KC_L, KC_M, KC_N,
    KC_O, KC_P, KC_Q, KC_R, KC_S, KC_T, KC_U,
    KC_V, KC_W, KC_X, KC_Y, KC_Z,
    // 0x7B-0x7F: { | } ~ DEL
    _S(KC_LBRC), _S(KC_BSLS), _S(KC_RBRC), _S(KC_GRV), 0,
};

#undef _S

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static const char *s_ss_ptr;       // current position (NULL = idle)
static uint32_t    s_ss_timer;     // timestamp for delay
static bool        s_ss_key_down;  // key currently pressed, waiting for release cycle
static uint8_t     s_ss_cur_kc;    // current keycode being pressed

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void send_string_init(void) {
    s_ss_ptr      = NULL;
    s_ss_timer    = 0;
    s_ss_key_down = false;
    s_ss_cur_kc   = 0;
}

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

void send_string(const char *str) {
    s_ss_ptr      = str;
    s_ss_key_down = false;
    s_ss_cur_kc   = 0;
    s_ss_timer    = 0;  // start immediately on next tick
    DBG_KEY("[SS] start \"%s\"\n", str ? str : "(null)");
}

bool send_string_is_active(void) {
    return s_ss_ptr != NULL;
}

// Shared LUT lookup for external interpreters (process_macro, Phase 15e).
uint16_t send_string_ascii_entry(char c) {
    if ((uint8_t)c >= 128) return 0;
    return ascii_to_keycode[(uint8_t)c];
}

// ---------------------------------------------------------------------------
// Tick — non-blocking, one character per SEND_STRING_DELAY interval
// ---------------------------------------------------------------------------

void send_string_tick(uint32_t now_ticks) {
    if (s_ss_ptr == NULL) return;

    // Wait for delay
    if (s_ss_timer != 0) {
        TickType_t elapsed = (TickType_t)(now_ticks - s_ss_timer);
        if (elapsed < pdMS_TO_TICKS(SEND_STRING_DELAY)) return;
    }

    if (s_ss_key_down) {
        // Release current key
        unregister_code(s_ss_cur_kc);
        del_weak_mods(MOD_BIT(KC_LSFT));   // only OUR shift (added for shifted chars)
        s_ss_key_down = false;
        s_ss_ptr++;  // advance to next char
        s_ss_timer = now_ticks;

        // Check if done
        if (*s_ss_ptr == '\0') {
            s_ss_ptr = NULL;
            DBG_KEY("[SS] done\n");
        }
        return;
    }

    // Press next character
    char ch = *s_ss_ptr;
    if (ch == '\0') {
        s_ss_ptr = NULL;
        return;
    }

    if ((uint8_t)ch >= 128) {
        // Non-ASCII — skip
        s_ss_ptr++;
        return;
    }

    uint16_t entry = ascii_to_keycode[(uint8_t)ch];
    if (entry == 0) {
        // Unsupported character — skip
        s_ss_ptr++;
        return;
    }

    uint8_t kc = (uint8_t)(entry & 0xFF);
    bool needs_shift = (entry & 0x8000) != 0;

    if (needs_shift) {
        add_weak_mods(MOD_BIT(KC_LSFT));
    }
    register_code(kc);

    s_ss_cur_kc   = kc;
    s_ss_key_down = true;
    s_ss_timer    = now_ticks;
}
