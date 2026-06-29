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
// process_tri_layer.c — Tri-Layer: L1+L2 active → auto-activate L3 (Phase 11)
//

#include "process_tri_layer.h"
#include "action_layer.h"
#include "process_layer_lock.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

void tri_layer_update(void) {
    uint32_t state = layer_state_get();
    bool lower_on = (state & (1u << TRI_LAYER_LOWER)) != 0;
    bool upper_on = (state & (1u << TRI_LAYER_UPPER)) != 0;

    if (lower_on && upper_on) {
        if (!layer_is_on(TRI_LAYER_ADJUST)) {
            // Use layer_set to avoid recursion (layer_on calls tri_layer_update)
            layer_set(state | (1u << TRI_LAYER_ADJUST));
            DBG_KEY("[TRI] adjust layer %d ON\n", TRI_LAYER_ADJUST);
        }
    } else {
        if (layer_is_on(TRI_LAYER_ADJUST) && !layer_lock_is_locked(TRI_LAYER_ADJUST)) {
            layer_set(state & ~(1u << TRI_LAYER_ADJUST));
            DBG_KEY("[TRI] adjust layer %d OFF\n", TRI_LAYER_ADJUST);
        }
    }
}
