/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
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
// cmk_keycodes.h — CMK custom keycode handlers (Phase 7)
//
// Handles CMK_TD0/TD1/TD2, CMK_TU, CMK_EE_CLR, CMK_RESET.
// Long-press state machine (3s threshold) for BLE slot combos.
//

#ifndef CMK_CMK_KEYCODES_H
#define CMK_CMK_KEYCODES_H

#include <stdint.h>
#include <stdbool.h>
#include "process_record.h"

// Initialise CMK keycode state.
void cmk_keycodes_init(void);

// Process a key event for CMK custom keycodes.
// Returns false if consumed, true if not a CMK key.
bool process_record_cmk(const keyevent_t *event);

// Tick — poll long-press timers. Call every scan cycle.
void cmk_keycodes_tick(uint32_t now_ticks);

#endif // CMK_CMK_KEYCODES_H
