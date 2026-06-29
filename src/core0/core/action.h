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
// action.h — Central key action dispatcher (Phase 7)
//
// Pipeline:
//   keyscan → action_exec(row, col, pressed, timestamp)
//     → layer_lookup → tapping / exec_inner → register/unregister
//   action_tick(now) — timers for tapping + CMK long-press
//

#ifndef CMK_ACTION_H
#define CMK_ACTION_H

#include <stdint.h>
#include <stdbool.h>

// Initialise all action subsystems (layer, util, tapping, cmk_keycodes).
void action_init(void);

// Process a key event: lookup keycode, dispatch through tapping/process_record.
void action_exec(uint8_t row, uint8_t col, bool pressed, uint32_t timestamp);

// Pressed-key cache (R2). The keycode resolved at PRESS time is cached per
// position and reused at RELEASE, so a layer change between press and release
// can no longer break the register/unregister symmetry (stuck keys with MO/LT/
// TG/OSL/tri-layer active — the "lost release with multi-layer" bug class).
// Replay paths that re-resolve a buffered PRESS on a new layer (tapping hold,
// combo) must keep the cache in sync via this updater.
void action_pressed_cache_update(uint8_t row, uint8_t col, uint16_t keycode);

// Tick all timers (tapping timeout, CMK long-press). Call every scan cycle.
void action_tick(uint32_t now_ticks);

#endif // CMK_ACTION_H
