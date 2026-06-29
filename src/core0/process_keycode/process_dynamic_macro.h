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
// process_dynamic_macro.h — Dynamic Macro: record and replay (Phase 10)
//

#ifndef PROCESS_DYNAMIC_MACRO_H
#define PROCESS_DYNAMIC_MACRO_H

#include "process_record.h"
#include <stdbool.h>
#include <stdint.h>

void dynamic_macro_init(void);
bool process_dynamic_macro(const keyevent_t *event);
void dynamic_macro_tick(uint32_t now_ticks);

// Weak user hooks
void dynamic_macro_record_start_user(int8_t macro_id);
void dynamic_macro_record_end_user(int8_t macro_id);
void dynamic_macro_play_user(int8_t macro_id);

#endif // PROCESS_DYNAMIC_MACRO_H
