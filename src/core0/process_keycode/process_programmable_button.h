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
// process_programmable_button.h — 32 Programmable HID Buttons (Phase 12)
//
// PB_1 through PB_32: each maps to a bit in a 32-bit bitmask.
// Sent as HID Report ID 3 (Consumer Control, 4 bytes).
//

#ifndef PROCESS_PROGRAMMABLE_BUTTON_H
#define PROCESS_PROGRAMMABLE_BUTTON_H

#include "process_record.h"
#include <stdbool.h>
#include <stdint.h>

void programmable_button_init(void);
bool process_programmable_button(const keyevent_t *event);

#endif // PROCESS_PROGRAMMABLE_BUTTON_H
