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
// macro_storage.h — EEPROM-stored user macros (Phase 15e).
//
// EEPROM region @ MACRO_STORAGE_ADDR (0x4000): 4-byte header (magic+version)
// + MACRO_BUFFER_SIZE raw bytes holding MACRO_COUNT concatenated macros, each
// terminated by 0x00 (factory state: all zeros = 16 empty macros).
//
// Byte-stream format (VIA-compatible; normative spec in
// tools/protocols/CMK_Protocol.md §8):
//   0x02..0x7F        ASCII char (typed via the send_string US-ANSI LUT)
//   0x01 0x01 kc      TAP  basic 8-bit HID keycode
//   0x01 0x02 kc      DOWN (register, stays held)
//   0x01 0x03 kc      UP   (unregister)
//   0x01 0x04 b1 b2   DELAY of (b1-1) + (b2-1)*255 ms   (b1,b2 in 1..255)
//   0x00              end of macro
//
// Pattern: RAM cache + paged flush (mirror of dynamic_keymap). Written only
// from the config protocol (USB/ih_link task); read by the playback FSM
// (keyscan task) — writers must call macro_play_request_abort() first.
//

#ifndef CMK_MACRO_STORAGE_H
#define CMK_MACRO_STORAGE_H

#include <stdint.h>
#include <stdbool.h>

// Load/validate the region (R4: retried read; read error → RAM-only defaults,
// no flush; magic/version mismatch → factory reset of this region only).
void macro_storage_init(void);

uint16_t macro_storage_size(void);    // MACRO_BUFFER_SIZE
uint8_t  macro_storage_count(void);   // MACRO_COUNT

// Raw byte-buffer access (offset/size in bytes). set flushes the touched range.
bool macro_storage_get_buffer(uint16_t offset, uint8_t size, uint8_t *dst);
bool macro_storage_set_buffer(uint16_t offset, uint8_t size, const uint8_t *src);

// Read-only view of the RAM cache (stable lifetime — playback FSM).
const uint8_t *macro_storage_data(void);

// Locate macro n: pointer to its first byte (an empty macro points at 0x00),
// NULL if n out of range or fewer than n terminators exist in the buffer.
const uint8_t *macro_storage_find(uint8_t n);

// Zero the buffer + full flush (factory reset of this region).
void macro_storage_reset(void);

#endif // CMK_MACRO_STORAGE_H
