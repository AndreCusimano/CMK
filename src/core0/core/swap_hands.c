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
// swap_hands.c — Swap Hands: mirror keymap left ↔ right (Phase 11)
//
// User defines hand_swap_config[MATRIX_ROWS][MATRIX_COLS][2] as weak in keymap.
// Each entry {new_row, new_col} maps original position to swapped position.
//
// Disabled by default (SWAP_HANDS_ENABLE must be defined in cmk_config.h).
//

#include "swap_hands.h"
#include "keycode.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#ifdef SWAP_HANDS_ENABLE

// ---------------------------------------------------------------------------
// User swap table (weak default — identity mapping = no swap)
// ---------------------------------------------------------------------------

__attribute__((weak))
const uint8_t hand_swap_config[MATRIX_ROWS][MATRIX_COLS][2] = {0};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool s_swap_active;
static bool s_swap_oneshot;
static bool s_swap_oneshot_triggered;

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void swap_hands_init(void) {
    s_swap_active   = false;
    s_swap_oneshot  = false;
    s_swap_oneshot_triggered = false;
}

// ---------------------------------------------------------------------------
// Remap — called from action_exec before layer_lookup
// ---------------------------------------------------------------------------

void swap_hands_remap(uint8_t *row, uint8_t *col) {
    if (!s_swap_active) return;

    uint8_t r = *row;
    uint8_t c = *col;
    if (r < MATRIX_ROWS && c < MATRIX_COLS) {
        uint8_t nr = hand_swap_config[r][c][0];
        uint8_t nc = hand_swap_config[r][c][1];
        // Only remap if the mapping is non-zero (valid)
        if (nr != 0 || nc != 0 || (r == 0 && c == 0)) {
            *row = nr;
            *col = nc;
        }
    }

    // One-shot: clear after first non-swap-hands keypress
    if (s_swap_oneshot && !s_swap_oneshot_triggered) {
        s_swap_oneshot_triggered = true;
    }
}

bool swap_hands_is_active(void) {
    return s_swap_active;
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_swap_hands(const keyevent_t *event) {
    uint16_t kc = event->keycode;
    if (!IS_SWAP_HANDS(kc)) {
        // One-shot: deactivate after first non-swap key release
        if (s_swap_oneshot && s_swap_oneshot_triggered && !event->pressed) {
            s_swap_active  = false;
            s_swap_oneshot = false;
            s_swap_oneshot_triggered = false;
            DBG_KEY("[SH] one-shot off\n");
        }
        return true;  // pass through
    }

    switch (kc) {
        case SH_ON:
            if (event->pressed) {
                s_swap_active = true;
                DBG_KEY("[SH] ON\n");
            }
            break;

        case SH_OFF:
            if (event->pressed) {
                s_swap_active = false;
                DBG_KEY("[SH] OFF\n");
            }
            break;

        case SH_TOGG:
            if (event->pressed) {
                s_swap_active = !s_swap_active;
                DBG_KEY("[SH] toggle → %d\n", s_swap_active);
            }
            break;

        case SH_MON:
            if (event->pressed) {
                s_swap_active = true;
                DBG_KEY("[SH] momentary ON\n");
            } else {
                s_swap_active = false;
                DBG_KEY("[SH] momentary OFF\n");
            }
            break;

        case SH_MOFF:
            if (event->pressed) {
                s_swap_active = false;
            } else {
                s_swap_active = true;
            }
            break;

        case SH_OS:
            if (event->pressed) {
                s_swap_active  = true;
                s_swap_oneshot = true;
                s_swap_oneshot_triggered = false;
                DBG_KEY("[SH] one-shot armed\n");
            }
            break;

        default:
            break;
    }

    return false;  // consumed
}

#else  // !SWAP_HANDS_ENABLE — stubs

void swap_hands_init(void) {}
void swap_hands_remap(uint8_t *row, uint8_t *col) { (void)row; (void)col; }
bool swap_hands_is_active(void) { return false; }
bool process_swap_hands(const keyevent_t *event) { (void)event; return true; }

#endif // SWAP_HANDS_ENABLE
