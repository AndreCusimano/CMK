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
// send_string.h — Send String: type out character sequences (Phase 9)
//

#ifndef CMK_SEND_STRING_H
#define CMK_SEND_STRING_H

#include <stdbool.h>
#include <stdint.h>

void send_string_init(void);

// Start sending a string. The pointer must remain valid (static/const lifetime).
// If a string is already in progress, it is replaced (last-wins).
void send_string(const char *str);

// Non-blocking tick — sends one character per SEND_STRING_DELAY interval.
// Called from action_tick().
void send_string_tick(uint32_t now_ticks);

// Returns true if a string send is in progress.
bool send_string_is_active(void);

// ASCII → LUT entry for external interpreters (process_macro, Phase 15e):
// bits 7:0 = HID keycode, bit 15 = needs Shift, 0 = unsupported / c >= 128.
uint16_t send_string_ascii_entry(char c);

#endif // CMK_SEND_STRING_H
