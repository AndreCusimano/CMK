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
// process_record.h — Key event processing chain (Phase 7)
//
// Chain: process_record() → process_record_cmk() → process_record_user()
// Return false = consumed (stop chain), true = continue to next handler.
//

#ifndef CMK_PROCESS_RECORD_H
#define CMK_PROCESS_RECORD_H

#include <stdint.h>
#include <stdbool.h>

// Key event passed through the processing chain.
typedef struct {
    uint16_t keycode;     // resolved 16-bit keycode
    uint8_t  row;
    uint8_t  col;
    bool     pressed;     // true = press, false = release
    uint32_t timestamp;   // tick count at event time
} keyevent_t;

// Process a key event through the chain.
// Returns false if consumed (no further action needed).
bool process_record(const keyevent_t *event);

// Weak function — override in user code for custom key handling.
// Default implementation returns true (pass-through).
bool process_record_user(const keyevent_t *event);

#endif // CMK_PROCESS_RECORD_H
