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
// process_ucis.c — UCIS: Unicode Character Input System (Phase 12)
//
// Press CMK_UCIS → type abbreviation → Enter/Space → match table → output.
// On match: backspace typed chars + send Unicode codepoint.
// On no match: noop (typed chars already on screen, user handles it).
//

#include "process_ucis.h"
#include "unicode.h"
#include "keycode.h"
#include "action_util.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Weak default: empty UCIS table
// ---------------------------------------------------------------------------

__attribute__((weak))
const ucis_entry_t ucis_table[] = {
    {NULL, 0}  // terminator
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool    s_ucis_active;
static char    s_ucis_input[UCIS_MAX_INPUT + 1];  // +1 for NUL
static uint8_t s_ucis_count;

// ---------------------------------------------------------------------------
// Helpers — keycode to ASCII char
// ---------------------------------------------------------------------------

static char kc_to_char(uint16_t kc) {
    if (kc >= KC_A && kc <= KC_Z) return 'a' + (char)(kc - KC_A);
    if (kc == KC_0) return '0';
    if (kc >= KC_1 && kc <= KC_9) return '1' + (char)(kc - KC_1);
    return 0;  // unsupported
}

// ---------------------------------------------------------------------------
// Helpers — send backspace N times (enqueue via register/unregister)
// ---------------------------------------------------------------------------

static void ucis_send_backspaces(uint8_t count) {
    // Synchronous backspaces — works because we're in the keyscan tick context
    // and send_keyboard_report() is called after each register/unregister.
    for (uint8_t i = 0; i < count; i++) {
        register_code(KC_BSPC);
        unregister_code(KC_BSPC);
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void ucis_init(void) {
    s_ucis_active = false;
    s_ucis_count  = 0;
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_ucis(const keyevent_t *event) {
    uint16_t kc = event->keycode;

    // Activate UCIS mode
    if (kc == CMK_UCIS) {
        if (event->pressed && !s_ucis_active) {
            s_ucis_active = true;
            s_ucis_count  = 0;
            s_ucis_input[0] = '\0';
            DBG_KEY("[UCIS] start\n");
        }
        return false;  // consumed
    }

    // Not in UCIS mode → pass through
    if (!s_ucis_active) return true;

    // Only handle presses
    if (!event->pressed) return false;  // consume releases silently

    // Enter or Space → attempt match
    if (kc == KC_ENT || kc == KC_SPC) {
        s_ucis_input[s_ucis_count] = '\0';
        DBG_KEY("[UCIS] end → \"%s\"\n", s_ucis_input);

        // Search table
        bool matched = false;
        for (const ucis_entry_t *e = ucis_table; e->abbreviation != NULL; e++) {
            if (strcmp(s_ucis_input, e->abbreviation) == 0) {
                // Backspace the typed abbreviation
                ucis_send_backspaces(s_ucis_count);
                // Send the Unicode codepoint
                unicode_send_codepoint(e->codepoint);
                DBG_KEY("[UCIS] match \"%s\" → U+%04lX\n",
                        e->abbreviation, (unsigned long)e->codepoint);
                matched = true;
                break;
            }
        }

        if (!matched) {
            DBG_KEY("[UCIS] no match for \"%s\"\n", s_ucis_input);
        }

        s_ucis_active = false;
        return false;  // consumed
    }

    // Backspace → remove last char
    if (kc == KC_BSPC) {
        if (s_ucis_count > 0) {
            s_ucis_count--;
        }
        return true;  // pass through — let the actual backspace go to host
    }

    // Escape → cancel
    if (kc == KC_ESC) {
        s_ucis_active = false;
        DBG_KEY("[UCIS] cancelled\n");
        return false;  // consumed
    }

    // Buffer the character
    char ch = kc_to_char(kc);
    if (ch != 0 && s_ucis_count < UCIS_MAX_INPUT) {
        s_ucis_input[s_ucis_count++] = ch;
    }

    // Let the keypress pass through so the user sees what they're typing
    return true;
}
