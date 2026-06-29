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
// debounce.c — Symmetric defer per-key debounce (R7: sym_defer_pk)
//
// R7: was sym_defer_g (ONE global timer reset by ANY raw change) — a single
// chattering switch deferred the whole matrix, delaying every other key's
// press/release arbitrarily and even swallowing a complete tap that fit inside
// another key's bounce window. Now each key has its own defer timer (QMK
// sym_defer_pk): a bouncy switch only debounces itself.
//

#include "debounce.h"
#include "config/cmk_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#ifndef DEBOUNCE
#define DEBOUNCE 5
#endif

// Per-key defer deadline (tick of last raw change + DEBOUNCE). 0 = not bouncing.
static TickType_t s_deadline[MATRIX_ROWS][MATRIX_COLS];
static uint16_t   s_active_count;   // keys currently being debounced

void debounce_init(void) {
    memset(s_deadline, 0, sizeof(s_deadline));
    s_active_count = 0;
}

bool debounce(matrix_row_t raw[], matrix_row_t cooked[],
              uint8_t num_rows, bool changed) {
    (void)changed;   // per-key: we diff raw vs cooked ourselves
    TickType_t now = xTaskGetTickCount();
    bool cooked_changed = false;

    for (uint8_t r = 0; r < num_rows; r++) {
        matrix_row_t delta = raw[r] ^ cooked[r];
        for (uint8_t c = 0; c < MATRIX_COLS; c++) {
            matrix_row_t mask = (matrix_row_t)((matrix_row_t)1u << c);
            if (delta & mask) {
                // Key differs from its cooked state: (re)arm / check its timer.
                if (s_deadline[r][c] == 0) {
                    s_deadline[r][c] = now + pdMS_TO_TICKS(DEBOUNCE);
                    if (s_deadline[r][c] == 0) s_deadline[r][c] = 1;  // avoid the idle marker
                    s_active_count++;
                } else if ((int32_t)(now - s_deadline[r][c]) >= 0) {
                    // Stable for DEBOUNCE ms → commit this key only.
                    cooked[r] = (matrix_row_t)((cooked[r] & ~mask) | (raw[r] & mask));
                    s_deadline[r][c] = 0;
                    s_active_count--;
                    cooked_changed = true;
                }
            } else if (s_deadline[r][c] != 0) {
                // Bounced back to the cooked state before the deadline → cancel.
                s_deadline[r][c] = 0;
                s_active_count--;
            }
        }
    }

    return cooked_changed;
}

bool debounce_is_active(void) {
    return s_active_count != 0;
}
