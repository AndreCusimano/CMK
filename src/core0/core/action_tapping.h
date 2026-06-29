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
// action_tapping.h — Tap/hold state machine for LT() and MT() (Phase 7)
//
// Buffers key events while waiting for tap/hold resolution.
// - Release before TAPPING_TERM → TAP (send keycode)
// - Hold past TAPPING_TERM → HOLD (activate layer or modifier)
//

#ifndef CMK_ACTION_TAPPING_H
#define CMK_ACTION_TAPPING_H

#include <stdint.h>
#include <stdbool.h>
#include "process_record.h"

// Initialise tapping state.
void action_tapping_init(void);

// Process a key event through the tapping system.
// Returns true if the event was consumed (buffered or resolved internally).
// Returns false if it should be handled as a normal key immediately.
bool action_tapping_process(const keyevent_t *event);

// Tick — check TAPPING_TERM timeout. Call every scan cycle.
void action_tapping_tick(uint32_t now_ticks);

#endif // CMK_ACTION_TAPPING_H
