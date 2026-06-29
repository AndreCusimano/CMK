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
// process_tap_dance.h — Tap Dance: multi-tap counting (Phase 9)
//

#ifndef CMK_PROCESS_TAP_DANCE_H
#define CMK_PROCESS_TAP_DANCE_H

#include "process_record.h"
#include <stdbool.h>
#include <stdint.h>

// Tap dance state (resolved outcome)
typedef enum {
    TD_NONE = 0,
    TD_SINGLE_TAP,
    TD_SINGLE_HOLD,
    TD_DOUBLE_TAP,
    TD_DOUBLE_HOLD,
    TD_TRIPLE_TAP,
    TD_TRIPLE_HOLD,
} td_state_t;

// Data-driven tap dance action (user-defined per TD index)
typedef struct {
    uint16_t single_tap;    // keycode for single tap (KC_NO = unused)
    uint16_t single_hold;   // keycode for single hold
    uint16_t double_tap;    // keycode for double tap
    uint16_t double_hold;   // keycode for double hold
} td_action_t;

void tap_dance_init(void);
bool process_tap_dance(const keyevent_t *event);
void tap_dance_tick(uint32_t now_ticks);

#endif // CMK_PROCESS_TAP_DANCE_H
