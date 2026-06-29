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
// matrix.c — Key matrix scan module (Phase 6)
//
// Supports DIRECT_PINS and COL2ROW/ROW2COL matrix modes.
// All pins are active-low with internal pull-up (pressed = GPIO LOW).
//

#include "matrix.h"
#include "config/cmk_config.h"

#include "hardware/gpio.h"
#include "hardware/timer.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

static matrix_row_t s_raw_matrix[MATRIX_ROWS];
static matrix_row_t s_debounced_matrix[MATRIX_ROWS];

// ---------------------------------------------------------------------------
// Direct pins mode
// ---------------------------------------------------------------------------

#ifdef DIRECT_PINS

static const uint8_t s_direct_pins[DIRECT_PINS_COUNT] = DIRECT_PINS;

void matrix_init(void) {
    memset(s_raw_matrix, 0, sizeof(s_raw_matrix));
    memset(s_debounced_matrix, 0, sizeof(s_debounced_matrix));

    for (int i = 0; i < DIRECT_PINS_COUNT; i++) {
        gpio_init(s_direct_pins[i]);
        gpio_set_dir(s_direct_pins[i], GPIO_IN);
        gpio_pull_up(s_direct_pins[i]);
    }
}

bool matrix_scan(void) {
    matrix_row_t row = 0;

    for (int col = 0; col < DIRECT_PINS_COUNT; col++) {
        if (!gpio_get(s_direct_pins[col])) {   // active-low
            row |= (matrix_row_t)(1u << col);
        }
    }

    bool changed = (row != s_raw_matrix[0]);
    s_raw_matrix[0] = row;
    return changed;
}

// Direct pins are always the sense lines; there is nothing to drive.
void matrix_wake_irq_drive(bool enable) { (void)enable; }

const uint8_t *matrix_get_input_pins(uint8_t *count) {
    if (count) *count = DIRECT_PINS_COUNT;
    return s_direct_pins;
}

// ---------------------------------------------------------------------------
// Matrix mode (COL2ROW / ROW2COL)
// ---------------------------------------------------------------------------

#else  // matrix mode

static const uint8_t s_row_pins[MATRIX_ROWS] = MATRIX_ROW_PINS;
static const uint8_t s_col_pins[MATRIX_COLS] = MATRIX_COL_PINS;

static inline void select_row(uint8_t row) {
    gpio_set_dir(s_row_pins[row], GPIO_OUT);
    gpio_put(s_row_pins[row], 0);   // drive LOW
}

static inline void unselect_row(uint8_t row) {
    gpio_set_dir(s_row_pins[row], GPIO_IN);
    gpio_pull_up(s_row_pins[row]);
}

static inline void select_col(uint8_t col) {
    gpio_set_dir(s_col_pins[col], GPIO_OUT);
    gpio_put(s_col_pins[col], 0);   // drive LOW
}

static inline void unselect_col(uint8_t col) {
    gpio_set_dir(s_col_pins[col], GPIO_IN);
    gpio_pull_up(s_col_pins[col]);
}

static void unselect_all_rows(void) {
    for (int r = 0; r < MATRIX_ROWS; r++) {
        unselect_row(r);
    }
}

static void unselect_all_cols(void) {
    for (int c = 0; c < MATRIX_COLS; c++) {
        unselect_col(c);
    }
}

void matrix_init(void) {
    memset(s_raw_matrix, 0, sizeof(s_raw_matrix));
    memset(s_debounced_matrix, 0, sizeof(s_debounced_matrix));

#if !defined(DIODE_DIRECTION)
#error "Matrix mode requires DIODE_DIRECTION (COL2ROW or ROW2COL)"
#endif

#if DIODE_DIRECTION == COL2ROW
    // Rows = output (unselected = input pull-up), Cols = input pull-up
    for (int r = 0; r < MATRIX_ROWS; r++) {
        gpio_init(s_row_pins[r]);
        unselect_row(r);
    }
    for (int c = 0; c < MATRIX_COLS; c++) {
        gpio_init(s_col_pins[c]);
        gpio_set_dir(s_col_pins[c], GPIO_IN);
        gpio_pull_up(s_col_pins[c]);
    }
#else  // ROW2COL
    // Cols = output (unselected = input pull-up), Rows = input pull-up
    for (int c = 0; c < MATRIX_COLS; c++) {
        gpio_init(s_col_pins[c]);
        unselect_col(c);
    }
    for (int r = 0; r < MATRIX_ROWS; r++) {
        gpio_init(s_row_pins[r]);
        gpio_set_dir(s_row_pins[r], GPIO_IN);
        gpio_pull_up(s_row_pins[r]);
    }
#endif
}

bool matrix_scan(void) {
    bool changed = false;

#if DIODE_DIRECTION == COL2ROW
    for (int r = 0; r < MATRIX_ROWS; r++) {
        select_row(r);
        busy_wait_us(MATRIX_IO_DELAY);

        matrix_row_t row = 0;
        for (int c = 0; c < MATRIX_COLS; c++) {
            if (!gpio_get(s_col_pins[c])) {    // active-low
                row |= (matrix_row_t)(1u << c);
            }
        }

        unselect_row(r);

        if (row != s_raw_matrix[r]) {
            s_raw_matrix[r] = row;
            changed = true;
        }
    }

#else  // ROW2COL
    // Temporary buffer — build column by column
    matrix_row_t tmp_matrix[MATRIX_ROWS];
    memset(tmp_matrix, 0, sizeof(tmp_matrix));

    for (int c = 0; c < MATRIX_COLS; c++) {
        select_col(c);
        busy_wait_us(MATRIX_IO_DELAY);

        for (int r = 0; r < MATRIX_ROWS; r++) {
            if (!gpio_get(s_row_pins[r])) {    // active-low
                tmp_matrix[r] |= (matrix_row_t)(1u << c);
            }
        }

        unselect_col(c);
    }

    for (int r = 0; r < MATRIX_ROWS; r++) {
        if (tmp_matrix[r] != s_raw_matrix[r]) {
            s_raw_matrix[r] = tmp_matrix[r];
            changed = true;
        }
    }
#endif

    return changed;
}

// Low-power wake: drive every select line active so ANY key shorts its sense
// line to LOW, then a falling-edge IRQ on the sense pins wakes the core. Pass
// false to restore the normal idle state (select lines released to pull-up).
void matrix_wake_irq_drive(bool enable) {
#if DIODE_DIRECTION == COL2ROW
    if (enable) { for (int r = 0; r < MATRIX_ROWS; r++) select_row(r); }
    else        { unselect_all_rows(); }
#else  // ROW2COL
    if (enable) { for (int c = 0; c < MATRIX_COLS; c++) select_col(c); }
    else        { unselect_all_cols(); }
#endif
}

const uint8_t *matrix_get_input_pins(uint8_t *count) {
#if DIODE_DIRECTION == COL2ROW
    if (count) *count = MATRIX_COLS;
    return s_col_pins;
#else  // ROW2COL
    if (count) *count = MATRIX_ROWS;
    return s_row_pins;
#endif
}

#endif  // DIRECT_PINS vs matrix

// ---------------------------------------------------------------------------
// Common accessors
// ---------------------------------------------------------------------------

matrix_row_t matrix_get_raw_row(uint8_t row) {
    return s_raw_matrix[row];
}

matrix_row_t matrix_get_row(uint8_t row) {
    return s_debounced_matrix[row];
}

matrix_row_t *matrix_get_raw(void) {
    return s_raw_matrix;
}

matrix_row_t *matrix_get_debounced(void) {
    return s_debounced_matrix;
}
