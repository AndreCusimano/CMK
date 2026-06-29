/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
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
// keymap.c — Per-side keymap and LED layout data + runtime side selection.
//
// One firmware runs on both halves, so BOTH the left and right keymaps (and LED
// maps) are compiled in. At boot keymap_init() points the runtime selectors
// (`keymaps`, `g_led_config`) at the layout matching the detected board side.
// Indexing a pointer is identical to indexing the old array, so the consumers
// (action_layer.c, rgb_matrix.c, led_matrix.c) are unchanged.
//
// Current hardware: prototype board, 4x6 matrix, COL2ROW.
//   Cols 0..5 -> GPIO 4..9
//   Rows 0..3 -> GPIO 25..28
//
// KC_FN (= MO(LAYER_SETTINGS)) is a regular keycode: place it in the base layer
// at the desired position; KC_TRNS in higher layers makes it reachable from any
// layer at the same cell via fall-through.
//

#include "keymap.h"
#include "keycode.h"
#include "config/cmk_config.h"

#if defined(RGB_MATRIX_ENABLE)
#include "rgb_matrix.h"
#elif defined(LED_MATRIX_ENABLE)
#include "led_matrix.h"
#endif

// ===========================================================================
//  UNIFIED KEYMAP — 4 rows x 12 logical cols (0..5 = LEFT, 6..11 = RIGHT)
// ===========================================================================
// Both halves compile and store this whole layout (Phase 18). A STANDALONE half
// uses only its own 6-column slice (selected by keymap_local_col_base()); a
// MASTER resolves all 12 columns. Each row below is LEFT 6 cells then RIGHT 6.

static const uint16_t keymap_full[NUM_LAYERS][MATRIX_ROWS][KEYMAP_COLS] = {

    // Layer 0 — Qwerty Normal
    [0] = {
        { KC_ESC,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,   KC_U,     KC_I,    KC_O,    KC_P,    KC_BSPC },
        { KC_BSLS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,   KC_J,     KC_K,    KC_L,    KC_NO,   KC_NO   },
        { KC_LSFT, KC_NUBS, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,   KC_N,     KC_M,    KC_COMM, KC_DOT,  KC_ENT  },
        { KC_LCTL, KC_LALT, KC_FN,   KC_LGUI, MO(1),   KC_SPC,  KC_SPC, MO(1),    KC_LGUI, KC_FN,   KC_NO,   KC_NO   },
    },

    // Layer 1 — Qwerty Momentary
    [1] = {
        { KC_TRNS, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_DEL  },
        { KC_TAB,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F5   },
        { KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_UP,   KC_TRNS },
        { KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_LEFT, KC_DOWN, KC_RGHT },
    },

    // Layer 2 — Colemak Normal
    [2] = {
        { KC_ESC,  KC_Q,    KC_W,    KC_F,    KC_P,    KC_G,    KC_J,   KC_L,     KC_U,    KC_Y,    KC_NO,   KC_BSPC },
        { KC_BSLS, KC_A,    KC_R,    KC_S,    KC_T,    KC_D,    KC_H,   KC_N,     KC_E,    KC_I,    KC_O,    KC_NO   },
        { KC_LSFT, KC_NUBS, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,   KC_K,     KC_M,    KC_COMM, KC_DOT,  KC_ENT  },
        { KC_LCTL, KC_LALT, KC_FN,   KC_LGUI, MO(3),   KC_SPC,  KC_SPC, MO(3),    KC_LGUI, KC_FN,   KC_NO,   KC_NO   },
    },

    // Layer 3 — Colemak Momentary
    [3] = {
        { KC_TRNS, KC_1,    KC_2,    KC_3,    KC_4,    KC_5,    KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    KC_DEL  },
        { KC_TAB,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,   KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  KC_F5   },
        { KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_UP,   KC_TRNS },
        { KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_LEFT, KC_DOWN, KC_RGHT },
    },

    // Layer 4 — US Safe (protected ANSI fallback)
    [LAYER_US_SAFE] = {
        { KC_ESC,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,   KC_U,     KC_I,    KC_O,    KC_P,    KC_BSPC },
        { KC_BSLS, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,   KC_J,     KC_K,    KC_L,    KC_NO,   KC_NO   },
        { KC_LSFT, KC_GRV,  KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,   KC_N,     KC_M,    KC_COMM, KC_DOT,  KC_ENT  },
        { KC_LCTL, KC_LALT, KC_FN,   KC_LGUI, KC_NO,   KC_SPC,  KC_SPC, KC_NO,    KC_LGUI, KC_FN,   KC_NO,   KC_NO   },
    },

    // Layer 5 — Settings
    [LAYER_SETTINGS] = {
        { CMK_TD0,       CMK_TD1,        CMK_TD2,        CMK_OUT_USB,    KC_NO,          CMK_TU,         CMK_TD0,             CMK_TD1,        CMK_TD2,        CMK_OUT_USB,    KC_NO,          CMK_TU        },
        { CMK_RGB_TOG,   CMK_RGB_RMOD,   CMK_RGB_MOD,    CMK_RGB_VAL_R,  CMK_RGB_HUE_R,  CMK_RGB_SAT_R,  CMK_RGB_TOG,         CMK_RGB_RMOD,   CMK_RGB_MOD,    CMK_RGB_VAL_R,  CMK_RGB_HUE_R,  CMK_RGB_SAT_R },
        { CMK_RGB_SPD_R, CMK_RGB_WSPD_R, CMK_RGB_WWID_R, CMK_RGB_FADE_R, CMK_RGB_HSFT_R, CMK_SPLIT_TOGG, CMK_RGB_SPD_R,       CMK_RGB_WSPD_R, CMK_RGB_WWID_R, CMK_RGB_FADE_R, CMK_RGB_HSFT_R, CMK_SPLIT_TOGG },
        { DF(0),         DF(2),          KC_FN,          CMK_CFG_PUSH,   CMK_CFG_RECV,   DF(LAYER_US_SAFE), DF(LAYER_US_SAFE), CMK_CFG_PUSH,   CMK_CFG_RECV,   KC_FN,          DF(2),          DF(0)         },
    },
};

// ===========================================================================
//  LED config — maps each LED to a key matrix position + physical coordinate
// ===========================================================================
// Used by RGB Matrix (Phase 13b) and LED Matrix (Phase 13c).
// Placeholder for breadboard — update coordinates when PCB is designed.
//
// Serpentine wiring (DIN -> DOUT), seen from the user side of the PCB.
//
// LEFT  — LED 0 is top-RIGHT
//   row 0 (top):     R -> L   LED  0.. 5
//   row 1:           L -> R   LED  6..11
//   row 2:           R -> L   LED 12..17
//   row 3 (bottom):  L -> R   LED 18..23
//
// RIGHT — LED 0 is top-LEFT (horizontally mirrored PCB trace)
//   row 0 (top):     L -> R   LED  0.. 5
//   row 1:           R -> L   LED  6..11
//   row 2:           L -> R   LED 12..17
//   row 3 (bottom):  R -> L   LED 18..23
//
// Physical X spans 0..224 over 6 columns (step ~45), Y spans 0..64 over 4 rows
// (step ~21). Adjust the absolute X/Y values when the final PCB dimensions are
// known — only the (row,col) mapping affects which LED reacts to a given
// keypress; X/Y are used by the splash effects for the radial maths.

#if defined(RGB_MATRIX_ENABLE) || defined(LED_MATRIX_ENABLE)

static const led_config_entry_t led_config_left[RGB_LED_COUNT] = {
    // Row 0 (top) - right to left (LED 0..5)
    { 0, 5, 224,  0, LED_FLAG_KEYLIGHT },
    { 0, 4, 179,  0, LED_FLAG_KEYLIGHT },
    { 0, 3, 134,  0, LED_FLAG_KEYLIGHT },
    { 0, 2,  90,  0, LED_FLAG_KEYLIGHT },
    { 0, 1,  45,  0, LED_FLAG_KEYLIGHT },
    { 0, 0,   0,  0, LED_FLAG_KEYLIGHT },

    // Row 1 - left to right (LED 6..11)
    { 1, 0,   0, 21, LED_FLAG_KEYLIGHT },
    { 1, 1,  45, 21, LED_FLAG_KEYLIGHT },
    { 1, 2,  90, 21, LED_FLAG_KEYLIGHT },
    { 1, 3, 134, 21, LED_FLAG_KEYLIGHT },
    { 1, 4, 179, 21, LED_FLAG_KEYLIGHT },
    { 1, 5, 224, 21, LED_FLAG_KEYLIGHT },

    // Row 2 - right to left (LED 12..17)
    { 2, 5, 224, 43, LED_FLAG_KEYLIGHT },
    { 2, 4, 179, 43, LED_FLAG_KEYLIGHT },
    { 2, 3, 134, 43, LED_FLAG_KEYLIGHT },
    { 2, 2,  90, 43, LED_FLAG_KEYLIGHT },
    { 2, 1,  45, 43, LED_FLAG_KEYLIGHT },
    { 2, 0,   0, 43, LED_FLAG_KEYLIGHT },

    // Row 3 (bottom) - left to right (LED 18..23)
    { 3, 0,   0, 64, LED_FLAG_KEYLIGHT },
    { 3, 1,  45, 64, LED_FLAG_KEYLIGHT },
    { 3, 2,  90, 64, LED_FLAG_KEYLIGHT },
    { 3, 3, 134, 64, LED_FLAG_KEYLIGHT },
    { 3, 4, 179, 64, LED_FLAG_KEYLIGHT },
    { 3, 5, 224, 64, LED_FLAG_KEYLIGHT },
};

static const led_config_entry_t led_config_right[RGB_LED_COUNT] = {
    // Row 0 (top) - left to right (LED 0..5) - mirrored vs LEFT
    { 0, 0,   0,  0, LED_FLAG_KEYLIGHT },
    { 0, 1,  45,  0, LED_FLAG_KEYLIGHT },
    { 0, 2,  90,  0, LED_FLAG_KEYLIGHT },
    { 0, 3, 134,  0, LED_FLAG_KEYLIGHT },
    { 0, 4, 179,  0, LED_FLAG_KEYLIGHT },
    { 0, 5, 224,  0, LED_FLAG_KEYLIGHT },

    // Row 1 - right to left (LED 6..11)
    { 1, 5, 224, 21, LED_FLAG_KEYLIGHT },
    { 1, 4, 179, 21, LED_FLAG_KEYLIGHT },
    { 1, 3, 134, 21, LED_FLAG_KEYLIGHT },
    { 1, 2,  90, 21, LED_FLAG_KEYLIGHT },
    { 1, 1,  45, 21, LED_FLAG_KEYLIGHT },
    { 1, 0,   0, 21, LED_FLAG_KEYLIGHT },

    // Row 2 - left to right (LED 12..17)
    { 2, 0,   0, 43, LED_FLAG_KEYLIGHT },
    { 2, 1,  45, 43, LED_FLAG_KEYLIGHT },
    { 2, 2,  90, 43, LED_FLAG_KEYLIGHT },
    { 2, 3, 134, 43, LED_FLAG_KEYLIGHT },
    { 2, 4, 179, 43, LED_FLAG_KEYLIGHT },
    { 2, 5, 224, 43, LED_FLAG_KEYLIGHT },

    // Row 3 (bottom) - right to left (LED 18..23)
    { 3, 5, 224, 64, LED_FLAG_KEYLIGHT },
    { 3, 4, 179, 64, LED_FLAG_KEYLIGHT },
    { 3, 3, 134, 64, LED_FLAG_KEYLIGHT },
    { 3, 2,  90, 64, LED_FLAG_KEYLIGHT },
    { 3, 1,  45, 64, LED_FLAG_KEYLIGHT },
    { 3, 0,   0, 64, LED_FLAG_KEYLIGHT },
};

#endif // RGB_MATRIX_ENABLE || LED_MATRIX_ENABLE

// ---------------------------------------------------------------------------
// Runtime selectors — default to LEFT; keymap_init() flips them at boot.
// ---------------------------------------------------------------------------

const uint16_t (*keymaps)[MATRIX_ROWS][KEYMAP_COLS] = keymap_full;

// This half's logical column base (0 = LEFT, MATRIX_COLS = RIGHT). Cached at boot.
static uint8_t s_local_col_base = 0;

#if defined(RGB_MATRIX_ENABLE) || defined(LED_MATRIX_ENABLE)
const led_config_entry_t *g_led_config = led_config_left;
#endif

void keymap_init(cmk_side_t side) {
    s_local_col_base = (side == CMK_SIDE_RIGHT) ? MATRIX_COLS : 0;
    keymaps = keymap_full;   // unified 4×12; both halves use the whole layout
#if defined(RGB_MATRIX_ENABLE) || defined(LED_MATRIX_ENABLE)
    g_led_config = (side == CMK_SIDE_RIGHT) ? led_config_right : led_config_left;
#endif
}

uint8_t keymap_local_col_base(void) {
    return s_local_col_base;
}

#if defined(RGB_MATRIX_ENABLE) || defined(LED_MATRIX_ENABLE)
void keymap_unified_xy(cmk_side_t side, uint8_t row, uint8_t phys_col,
                       uint16_t *x, uint16_t *y) {
    const led_config_entry_t *map = (side == CMK_SIDE_RIGHT) ? led_config_right
                                                             : led_config_left;
    uint16_t off = (side == CMK_SIDE_RIGHT) ? RGB_MATRIX_SPLIT_X_OFFSET : 0;
    for (uint16_t i = 0; i < RGB_LED_COUNT; i++) {
        if (map[i].row == row && map[i].col == phys_col) {
            *x = (uint16_t)(map[i].x + off);
            *y = map[i].y;
            return;
        }
    }
    *x = off;   // no LED for this key — fall back to the half's left edge
    *y = 0;
}
#endif

// Compiled default unified keymap — used by dynamic_keymap to seed/reset the
// EEPROM-backed keymap. Contiguous in flash, so it can be memcpy'd wholesale.
const uint16_t (*keymap_default_full(void))[MATRIX_ROWS][KEYMAP_COLS] {
    return keymap_full;
}
