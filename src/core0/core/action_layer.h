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
// action_layer.h — Layer state management (Phase 7)
//
// Layer stack implemented as a 32-bit bitmask. layer_lookup() resolves
// the effective keycode for a matrix position by walking layers top-down.
//
// FN is no longer hardcoded — place KC_FN (= MO(LAYER_SETTINGS)) in the
// keymap wherever the layout requires it.
//

#ifndef CMK_ACTION_LAYER_H
#define CMK_ACTION_LAYER_H

#include <stdint.h>
#include <stdbool.h>

// Initialise layer state (default_layer = 0, layer_state = 0).
void action_layer_init(void);

// Resolve the effective 16-bit keycode at a LOGICAL column (0..KEYMAP_COLS-1) of
// the unified 4×12 keymap. Phase 18: callers pass logical columns — keyscan adds
// this half's slice base for local keys; the split MASTER uses the peer's slice
// for remote keys. Walks active layers top-down; returns KC_NO if nothing.
uint16_t layer_lookup(uint8_t row, uint8_t col);

// Layer bitmask manipulation
void layer_on(uint8_t layer);
void layer_off(uint8_t layer);
void layer_toggle(uint8_t layer);
void layer_set(uint32_t mask);        // replace entire bitmask

// Default layer (base — always implicitly active)
void     default_layer_set(uint8_t layer);
uint8_t  default_layer_get(void);

// Read current state
uint32_t layer_state_get(void);
bool     layer_is_on(uint8_t layer);

// One-shot layer support
void layer_osl_set(uint8_t layer);    // arm one-shot
void layer_osl_clear(void);           // called after next key release
bool layer_osl_active(void);

#endif // CMK_ACTION_LAYER_H
