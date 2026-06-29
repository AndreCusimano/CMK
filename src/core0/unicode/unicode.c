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
// unicode.c — Unicode input common infrastructure (Phase 12)
//
// Tick-based state machine for OS-specific Unicode codepoint entry.
// Each tick cycle: press keycode → wait → release keycode → wait → next step.
// Same pattern as send_string.c.
//

#include "unicode.h"
#include "keycode.h"
#include "action_util.h"
#include "eeconfig.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"

// ---------------------------------------------------------------------------
// Step command types
// ---------------------------------------------------------------------------

enum {
    UC_CMD_TAP,    // tap keycode (press → delay → release → delay)
    UC_CMD_HOLD,   // register_mods (real mods — stay held)
    UC_CMD_REL,    // unregister_mods
};

typedef struct {
    uint8_t cmd;   // UC_CMD_*
    uint8_t val;   // keycode (TAP) or HID mod bitmap (HOLD/REL)
} uc_cmd_t;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static unicode_mode_t s_uc_mode;

static uc_cmd_t  s_uc_seq[UC_SEQ_MAX];
static uint8_t   s_uc_seq_len;
static uint8_t   s_uc_seq_idx;
static bool      s_uc_key_down;     // current TAP key is pressed
static uint32_t  s_uc_timer;

// ---------------------------------------------------------------------------
// Helpers — hex digit to keycode
// ---------------------------------------------------------------------------

static uint8_t hex_to_keycode(uint8_t digit) {
    // HID keycodes: KC_1=0x1E..KC_9=0x26, KC_0=0x27, KC_A=0x04..KC_F=0x09
    if (digit == 0) return KC_0;
    if (digit <= 9) return KC_1 + (digit - 1);
    return KC_A + (digit - 10);
}

// ---------------------------------------------------------------------------
// Sequence builder — OS-specific
// ---------------------------------------------------------------------------

static void build_sequence(uint32_t codepoint) {
    s_uc_seq_len = 0;

    // Extract hex digits (up to 4 for BMP, skip leading zeros)
    uint8_t digits[4];
    uint8_t n_digits = 0;
    for (int i = 3; i >= 0; i--) {
        uint8_t d = (codepoint >> (i * 4)) & 0xF;
        if (d != 0 || n_digits > 0 || i == 0) {
            digits[n_digits++] = d;
        }
    }

    switch (s_uc_mode) {
        case UC_MODE_MACOS:
            // Hold Option → hex digits → release Option
            s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_HOLD, MOD_BIT(KC_LALT)};
            for (uint8_t i = 0; i < n_digits; i++) {
                s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_TAP, hex_to_keycode(digits[i])};
            }
            s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_REL, MOD_BIT(KC_LALT)};
            break;

        case UC_MODE_LINUX:
            // Ctrl+Shift+U → release mods → hex digits → Enter
            s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_HOLD, MOD_BIT(KC_LCTL) | MOD_BIT(KC_LSFT)};
            s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_TAP, KC_U};
            s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_REL, MOD_BIT(KC_LCTL) | MOD_BIT(KC_LSFT)};
            for (uint8_t i = 0; i < n_digits; i++) {
                s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_TAP, hex_to_keycode(digits[i])};
            }
            s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_TAP, KC_ENT};
            break;

        case UC_MODE_WINDOWS:
            // Hold Alt → tap KP_PLUS → hex digits → release Alt
            // Requires registry: HKCU\Control Panel\Input Method\EnableHexNumpad = "1"
            s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_HOLD, MOD_BIT(KC_LALT)};
            s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_TAP, KC_KP_PLUS};
            for (uint8_t i = 0; i < n_digits; i++) {
                s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_TAP, hex_to_keycode(digits[i])};
            }
            s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_REL, MOD_BIT(KC_LALT)};
            break;

        case UC_MODE_WINCOMPOSE:
            // tap RAlt → tap U → hex digits → Enter
            s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_TAP, KC_RALT};
            s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_TAP, KC_U};
            for (uint8_t i = 0; i < n_digits; i++) {
                s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_TAP, hex_to_keycode(digits[i])};
            }
            s_uc_seq[s_uc_seq_len++] = (uc_cmd_t){UC_CMD_TAP, KC_ENT};
            break;

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void unicode_init(void) {
    s_uc_mode    = (unicode_mode_t)eeconfig_read_unicode_mode();
    if (s_uc_mode >= UC_MODE_COUNT) s_uc_mode = UC_MODE_LINUX;
    s_uc_seq_len = 0;
    s_uc_seq_idx = 0;
    s_uc_key_down = false;
    s_uc_timer   = 0;
    DBG_KEY("[UC] init mode=%d\n", s_uc_mode);
}

// ---------------------------------------------------------------------------
// Mode management
// ---------------------------------------------------------------------------

unicode_mode_t unicode_get_mode(void) {
    return s_uc_mode;
}

void unicode_set_mode(unicode_mode_t mode) {
    if (mode >= UC_MODE_COUNT) return;
    s_uc_mode = mode;
    eeconfig_update_unicode_mode((uint8_t)mode);
    DBG_KEY("[UC] mode=%d\n", mode);
}

void unicode_cycle_mode(bool forward) {
    int m = (int)s_uc_mode;
    if (forward) {
        m = (m + 1) % UC_MODE_COUNT;
    } else {
        m = (m + UC_MODE_COUNT - 1) % UC_MODE_COUNT;
    }
    unicode_set_mode((unicode_mode_t)m);
}

// ---------------------------------------------------------------------------
// Send codepoint (non-blocking — queues sequence)
// ---------------------------------------------------------------------------

void unicode_send_codepoint(uint32_t codepoint) {
    if (codepoint > 0x7FFF) return;  // BMP only for UC() encoding

    build_sequence(codepoint);
    s_uc_seq_idx  = 0;
    s_uc_key_down = false;
    s_uc_timer    = 0;  // start immediately
    DBG_KEY("[UC] send U+%04lX (%d steps)\n", (unsigned long)codepoint, s_uc_seq_len);
}

bool unicode_is_active(void) {
    return s_uc_seq_idx < s_uc_seq_len;
}

// ---------------------------------------------------------------------------
// Tick — drives the sequence step by step
// ---------------------------------------------------------------------------

void unicode_tick(uint32_t now_ticks) {
    if (s_uc_seq_idx >= s_uc_seq_len) return;

    // Wait for delay between steps
    if (s_uc_timer != 0) {
        TickType_t elapsed = (TickType_t)(now_ticks - s_uc_timer);
        if (elapsed < pdMS_TO_TICKS(UNICODE_SEND_DELAY)) return;
    }

    uc_cmd_t *step = &s_uc_seq[s_uc_seq_idx];

    switch (step->cmd) {
        case UC_CMD_TAP:
            if (s_uc_key_down) {
                // Release phase
                unregister_code(step->val);
                s_uc_key_down = false;
                s_uc_seq_idx++;
                s_uc_timer = now_ticks;
            } else {
                // Press phase
                register_code(step->val);
                s_uc_key_down = true;
                s_uc_timer = now_ticks;
            }
            break;

        case UC_CMD_HOLD:
            register_mods(step->val);
            s_uc_seq_idx++;
            s_uc_timer = now_ticks;
            break;

        case UC_CMD_REL:
            unregister_mods(step->val);
            s_uc_seq_idx++;
            s_uc_timer = now_ticks;
            break;
    }

    // Sequence complete — clean up
    if (s_uc_seq_idx >= s_uc_seq_len) {
        clear_weak_mods();
        DBG_KEY("[UC] done\n");
    }
}
