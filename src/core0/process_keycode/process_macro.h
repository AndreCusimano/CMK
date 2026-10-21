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
// process_macro.h — MACRO(n) playback of EEPROM-stored macros (Phase 15e).
//
// Tick-based non-blocking interpreter of the macro_storage byte stream
// (format in macro_storage.h): one press/release phase per MACRO_PLAY_DELAY.
// A MACRO(n) press while a playback (or send_string) is active is ignored
// (no queue, no nesting).
//

#ifndef CMK_PROCESS_MACRO_H
#define CMK_PROCESS_MACRO_H

#include <stdint.h>
#include <stdbool.h>

#include "process_record.h"   // keyevent_t

void macro_play_init(void);

// false = consumed (MACRO(n) keycode), true = pass through.
bool process_macro(const keyevent_t *event);

// One FSM step per MACRO_PLAY_DELAY (or a pending DELAY escape). Called from
// action_tick() in keyscan context.
void macro_play_tick(uint32_t now_ticks);

bool macro_play_is_active(void);

// Ask the keyscan task to stop the playback (release the held key, go idle).
// MUST be called by the config protocol BEFORE mutating the macro buffer
// (SET_MACRO_BUFFER / RESET_MACROS / EEPROM_RESET). Only sets a flag: the
// release goes through action_util, owned by keyscan. Keyscan (prio 3) runs
// above USB/ih_link on the same core, so every tick after the flag is set
// sees it before touching the buffer again.
void macro_play_request_abort(void);

#endif // CMK_PROCESS_MACRO_H
