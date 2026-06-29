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
// keyscan.h — Key scanner task (Phase 6)
//
// FreeRTOS task on Core 0 that scans the key matrix (or direct pins),
// applies debounce, builds HID keyboard reports, and sends them to
// Core 1 via IPC. Handles FN combo logic for connection management.
//

#ifndef CMK_KEYSCAN_H
#define CMK_KEYSCAN_H

// Initialise matrix GPIO pins and debounce state.
void keyscan_init(void);

// Create and start the FreeRTOS scanner task.
void keyscan_create_task(void);

#endif // CMK_KEYSCAN_H
