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
// process_combo.h — Combo: simultaneous key press → action (Phase 9)
//

#ifndef CMK_PROCESS_COMBO_H
#define CMK_PROCESS_COMBO_H

#include "process_record.h"
#include <stdbool.h>
#include <stdint.h>

// Combo definition (user provides an array of these)
#define COMBO_KEY_END  0xFF   // terminator for keys[] array

typedef struct {
    uint8_t  keys[4];    // matrix positions as (row<<4|col), terminated by COMBO_KEY_END
    uint16_t result;     // keycode to output when combo fires
} combo_def_t;

void combo_init(void);

// Pre-process hook — called from action_exec() BEFORE tapping.
// Returns true if event was consumed (buffered for combo detection).
bool combo_process(const keyevent_t *event);

// Tick — check COMBO_TERM timeout, replay buffer if no combo matched.
void combo_tick(uint32_t now_ticks);

#endif // CMK_PROCESS_COMBO_H
