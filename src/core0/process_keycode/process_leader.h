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
// process_leader.h — Leader Key: sequence → action (Phase 10)
//

#ifndef PROCESS_LEADER_H
#define PROCESS_LEADER_H

#include "process_record.h"
#include <stdbool.h>
#include <stdint.h>

void leader_init(void);
bool process_leader(const keyevent_t *event);
void leader_tick(uint32_t now_ticks);

// Sequence matching helpers (call from leader_end_user)
bool leader_sequence_one_key(uint16_t kc1);
bool leader_sequence_two_keys(uint16_t kc1, uint16_t kc2);
bool leader_sequence_three_keys(uint16_t kc1, uint16_t kc2, uint16_t kc3);
bool leader_sequence_four_keys(uint16_t kc1, uint16_t kc2, uint16_t kc3, uint16_t kc4);
bool leader_sequence_five_keys(uint16_t kc1, uint16_t kc2, uint16_t kc3, uint16_t kc4, uint16_t kc5);

// Weak user hooks
void leader_start_user(void);
void leader_end_user(void);

#endif // PROCESS_LEADER_H
