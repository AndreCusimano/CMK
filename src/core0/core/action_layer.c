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
// action_layer.c — Layer state management (Phase 7)
//

#include "action_layer.h"
#include "keycode.h"
#include "keymap.h"
#include "process_layer_lock.h"
#include "process_tri_layer.h"
#include "eeconfig.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include <stdio.h>

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint32_t s_layer_state;     // bitmask of active layers (bit N = layer N)
static uint8_t  s_default_layer;   // base layer (always implicitly active)

static uint8_t  s_osl_layer;       // one-shot layer number
static bool     s_osl_active;      // true between OSL press and next key release

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void action_layer_init(void) {
    s_layer_state  = 0;
    s_osl_layer    = 0;
    s_osl_active   = false;

    // Restore default layer from eeconfig (Phase 11)
    uint8_t dl = eeconfig_read_default_layer();
    s_default_layer = (dl < NUM_LAYERS) ? dl : 0;
}

// ---------------------------------------------------------------------------
// Layer lookup — resolve keycode for [row][col]
// ---------------------------------------------------------------------------

// Resolve the keycode at a LOGICAL column (0..KEYMAP_COLS-1) of the unified 4×12
// keymap. Phase 18: every event in the action pipeline carries a logical column
// (keyscan adds this half's slice base for local keys; the split MASTER uses the
// peer's slice for remote keys), so this walk is purely logical — no base offset.
uint16_t layer_lookup(uint8_t row, uint8_t col) {
    // Walk active layers top-down (highest numbered first)
    uint32_t mask = s_layer_state;
    for (int8_t l = NUM_LAYERS - 1; l >= 0; l--) {
        if (!(mask & (1u << l))) continue;

        uint16_t kc = keymaps[l][row][col];
        if (kc != KC_TRNS) return kc;
    }

    // Fallback: default layer
    uint16_t kc = keymaps[s_default_layer][row][col];
    if (kc != KC_TRNS) return kc;

    return KC_NO;
}

// ---------------------------------------------------------------------------
// Layer bitmask manipulation
// ---------------------------------------------------------------------------

void layer_on(uint8_t layer) {
    if (layer >= NUM_LAYERS) return;
    s_layer_state |= (1u << layer);
    DBG_KEY("[LAYER] on %d  state=0x%02lx\n", layer, (unsigned long)s_layer_state);
    tri_layer_update();
}

void layer_off(uint8_t layer) {
    if (layer >= NUM_LAYERS) return;
    // Layer Lock: skip if locked (Phase 11)
    if (layer_lock_is_locked(layer)) return;
    s_layer_state &= ~(1u << layer);
    DBG_KEY("[LAYER] off %d  state=0x%02lx\n", layer, (unsigned long)s_layer_state);
    tri_layer_update();
}

void layer_toggle(uint8_t layer) {
    if (layer >= NUM_LAYERS) return;
    s_layer_state ^= (1u << layer);
    DBG_KEY("[LAYER] toggle %d  state=0x%02lx\n", layer, (unsigned long)s_layer_state);
    tri_layer_update();
}

void layer_set(uint32_t mask) {
    s_layer_state = mask;
    DBG_KEY("[LAYER] set  state=0x%02lx\n", (unsigned long)s_layer_state);
}

// ---------------------------------------------------------------------------
// Default layer
// ---------------------------------------------------------------------------

void default_layer_set(uint8_t layer) {
    if (layer >= NUM_LAYERS) return;
    s_default_layer = layer;
    eeconfig_update_default_layer(layer);
    DBG_KEY("[LAYER] default → %d\n", layer);
}

uint8_t default_layer_get(void) {
    return s_default_layer;
}

// ---------------------------------------------------------------------------
// Read state
// ---------------------------------------------------------------------------

uint32_t layer_state_get(void) {
    return s_layer_state;
}

bool layer_is_on(uint8_t layer) {
    if (layer >= NUM_LAYERS) return false;
    return (s_layer_state & (1u << layer)) != 0;
}

// ---------------------------------------------------------------------------
// One-shot layer
// ---------------------------------------------------------------------------

void layer_osl_set(uint8_t layer) {
    s_osl_layer  = layer;
    s_osl_active = true;
    layer_on(layer);
    DBG_KEY("[LAYER] OSL armed → %d\n", layer);
}

void layer_osl_clear(void) {
    if (s_osl_active) {
        layer_off(s_osl_layer);
        s_osl_active = false;
        DBG_KEY("[LAYER] OSL cleared → %d\n", s_osl_layer);
    }
}

bool layer_osl_active(void) {
    return s_osl_active;
}
