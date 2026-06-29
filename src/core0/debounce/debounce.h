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
// debounce.h — Symmetric defer global debounce (Phase 6)
//
// Algorithm: sym_defer_g (same as QMK default).
// After any raw state change, waits DEBOUNCE ms of silence before
// propagating raw_matrix into the debounced matrix.
//

#ifndef CMK_DEBOUNCE_H
#define CMK_DEBOUNCE_H

#include "matrix.h"
#include <stdint.h>
#include <stdbool.h>

// Initialise debounce state.
void debounce_init(void);

// Process debounce.
// raw[]    = current raw matrix (from matrix_scan)
// cooked[] = debounced output matrix
// num_rows = MATRIX_ROWS
// changed  = true if raw[] changed this scan
// Returns true if cooked[] was updated.
bool debounce(matrix_row_t raw[], matrix_row_t cooked[],
              uint8_t num_rows, bool changed);

// Returns true if debounce timer is still active (waiting for silence).
bool debounce_is_active(void);

#endif // CMK_DEBOUNCE_H
