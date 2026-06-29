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
// matrix.h — Key matrix scan module (Phase 6)
//
// Supports two modes (configured in cmk_config.h):
//   - DIRECT_PINS: each key on its own GPIO pin
//   - Matrix: rows × columns with diodes (COL2ROW or ROW2COL)
//

#ifndef CMK_MATRIX_H
#define CMK_MATRIX_H

#include "config/cmk_config.h"
#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// matrix_row_t — auto-sized bitmask for one row of the matrix
// ---------------------------------------------------------------------------

#if MATRIX_COLS <= 8
typedef uint8_t matrix_row_t;
#elif MATRIX_COLS <= 16
typedef uint16_t matrix_row_t;
#elif MATRIX_COLS <= 32
typedef uint32_t matrix_row_t;
#else
#error "MATRIX_COLS > 32 not supported"
#endif

// ---------------------------------------------------------------------------
// Diode direction (matrix mode only)
// ---------------------------------------------------------------------------

#ifndef COL2ROW
#define COL2ROW  0
#endif
#ifndef ROW2COL
#define ROW2COL  1
#endif

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Initialise GPIO pins for the matrix (or direct pins).
void matrix_init(void);

// Scan the matrix hardware into raw_matrix[].
// Returns true if raw_matrix changed since the last call.
bool matrix_scan(void);

// Get the current raw (un-debounced) row bitmask.
matrix_row_t matrix_get_raw_row(uint8_t row);

// Get the debounced row bitmask (managed externally by debounce module,
// but stored here for convenience).
matrix_row_t matrix_get_row(uint8_t row);

// Pointer to the raw matrix array (for debounce module).
matrix_row_t *matrix_get_raw(void);

// Pointer to the debounced matrix array (for debounce module).
matrix_row_t *matrix_get_debounced(void);

// ---------------------------------------------------------------------------
// Low-power wake support (DEEP sleep — see low_power / keyscan)
// ---------------------------------------------------------------------------

// Drive all the "select" lines active (rows LOW in COL2ROW) so that ANY pressed
// key pulls its sense line LOW — ready for a falling-edge GPIO IRQ wake. Pass
// false to restore the normal scan idle state (select lines released).
void matrix_wake_irq_drive(bool enable);

// The sense pins (input + pull-up, go LOW on a key press) that serve as the
// falling-edge wake-IRQ sources. Returns the array; *count = how many.
const uint8_t *matrix_get_input_pins(uint8_t *count);

#endif // CMK_MATRIX_H
