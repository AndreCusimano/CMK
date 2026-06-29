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
// process_layer_lock.c — Layer Lock (Phase 11)
//
// CMK_LAYER_LOCK toggles the lock on the highest active layer.
// Locked layers are immune to layer_off() from MO/LT/OSL release.
//

#include "process_layer_lock.h"
#include "keycode.h"
#include "action_layer.h"
#include "config/debug_config.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint32_t s_locked_layers;  // bitmask: bit N = layer N locked

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void layer_lock_init(void) {
    s_locked_layers = 0;
}

// ---------------------------------------------------------------------------
// Query
// ---------------------------------------------------------------------------

bool layer_lock_is_locked(uint8_t layer) {
    return (s_locked_layers & (1u << layer)) != 0;
}

// ---------------------------------------------------------------------------
// Process
// ---------------------------------------------------------------------------

bool process_layer_lock(const keyevent_t *event) {
    if (event->keycode != CMK_LAYER_LOCK) return true;  // pass through

    if (!event->pressed) return false;  // consume release

    // Find highest active layer (excluding default)
    uint32_t state = layer_state_get();
    uint8_t def = default_layer_get();

    int8_t highest = -1;
    for (int8_t l = 31; l >= 0; l--) {
        if (l == def) continue;
        if (state & (1u << l)) {
            highest = l;
            break;
        }
    }

    if (highest < 0) {
        DBG_KEY("[LLOCK] no active layer to lock\n");
        return false;
    }

    // Toggle lock
    s_locked_layers ^= (1u << highest);
    bool locked = (s_locked_layers & (1u << highest)) != 0;
    DBG_KEY("[LLOCK] layer %d %s\n", highest, locked ? "LOCKED" : "UNLOCKED");

    return false;  // consumed
}
