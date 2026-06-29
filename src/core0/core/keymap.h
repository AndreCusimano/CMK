/*
 * This file is part of CMK - Custom Mechanical Keyboard Firmware.
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
// keymap.h - Multi-layer keymap [NUM_LAYERS][MATRIX_ROWS][MATRIX_COLS] (Phase 7)
//
// Layer 0 = Base (user default)
// Layer 1-3 = User layers (all KC_TRNS by default)
// Layer 4 = US Safe - protected US ANSI layout, non-modifiable
// Layer 5 = Settings - firmware-owned: CMK_TD*, TG(), CMK_EE_CLR, etc.
//
// The actual layout data lives in keymap.c, which compiles BOTH the left and
// right keymaps. keymap_init() selects the one matching the detected board side
// at boot (see board_side.h). `keymaps` is a runtime pointer indexed exactly
// like the old 3D array — keymaps[layer][row][col].
//

#ifndef CMK_KEYMAP_H
#define CMK_KEYMAP_H

#include <stdint.h>
#include "keycode.h"
#include "config/cmk_config.h"
#include "board_side.h"

// Phase 18: the keymap is a UNIFIED 4×12 logical layout shared by both halves —
// cols 0..5 = LEFT slice, 6..11 = RIGHT slice (fixed convention). Both halves
// store the complete keymap so a MASTER can resolve the peer's keys locally and
// the config view is a single 4×12. KEYMAP_COLS is the logical width (= 12).
#define KEYMAP_COLS  (2 * MATRIX_COLS)

// Runtime keymap pointer, indexed keymaps[layer][row][logical_col] with
// logical_col in 0..KEYMAP_COLS-1. In STANDALONE the action pipeline passes
// PHYSICAL cols (0..MATRIX_COLS-1) and layer_lookup() adds keymap_local_col_base()
// to reach this half's slice; the configurator addresses logical cols directly.
extern const uint16_t (*keymaps)[MATRIX_ROWS][KEYMAP_COLS];

// Select the per-side LED layout (when an LED/RGB matrix is enabled) and cache
// this half's logical column base. Call once at boot, after board_side_init()
// and before action_init() (rgb_matrix_init() reads g_led_config).
void keymap_init(cmk_side_t side);

// This half's logical column base: 0 for LEFT, MATRIX_COLS (6) for RIGHT. Added
// to a physical column to address the local slice of the unified keymap.
uint8_t keymap_local_col_base(void);

// Compiled default unified keymap (flash, 4×12). Used by dynamic_keymap to
// seed/reset the EEPROM-backed keymap. The block is contiguous in flash.
const uint16_t (*keymap_default_full(void))[MATRIX_ROWS][KEYMAP_COLS];

#if defined(RGB_MATRIX_ENABLE) || defined(LED_MATRIX_ENABLE)
// Phase 24b — UNIFIED physical position of any key on either half. side selects
// the LED map (CMK_SIDE_LEFT/RIGHT), (row, phys_col) the key within it; the RIGHT
// half is shifted by RGB_MATRIX_SPLIT_X_OFFSET so both halves share one X space.
// Lets each half place a reactive hit for a keypress on the OTHER half.
void keymap_unified_xy(cmk_side_t side, uint8_t row, uint8_t phys_col,
                       uint16_t *x, uint16_t *y);
#endif

#endif // CMK_KEYMAP_H
