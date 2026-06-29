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
// dynamic_keymap.h — Runtime-editable keymap backed by EEPROM (Phase 15a).
//
// The keymap is held in a RAM cache and mirrored to the EEPROM region at
// DYNAMIC_KEYMAP_ADDR (header: magic + version, then the raw 16-bit keycodes).
// At boot dynamic_keymap_init() loads the cache from EEPROM (or seeds it from
// the compiled default for the detected side on magic/version mismatch) and
// points the global `keymaps` pointer at the RAM cache, so layer_lookup() and
// every keymap consumer transparently use the editable copy.
//
// The CMK Config Protocol (src/usb/cmk_config_protocol.c) drives the get/set
// entry points; each set updates the cache and flushes to EEPROM.
//
// Keycodes are stored native (little-endian on this ARM core), so the raw byte
// view exposed by the buffer accessors matches the protocol's wire format.
//
// Compiled only when DYNAMIC_KEYMAP_ENABLE is defined.
//

#ifndef CMK_DYNAMIC_KEYMAP_H
#define CMK_DYNAMIC_KEYMAP_H

#include <stdint.h>
#include <stdbool.h>
#include "board_side.h"

// Load the keymap cache from EEPROM (or seed from the compiled default for
// `side` if the region header is invalid) and point `keymaps` at the cache.
// Call once at boot, after eeconfig_init() and keymap_init().
void dynamic_keymap_init(cmk_side_t side);

// Total size of the keymap in bytes (NUM_LAYERS * ROWS * COLS * 2).
uint16_t dynamic_keymap_size(void);

// Single keycode access. Out-of-range indices: get returns KC_NO (0), set
// returns false without touching the cache.
uint16_t dynamic_keymap_get_keycode(uint8_t layer, uint8_t row, uint8_t col);
bool     dynamic_keymap_set_keycode(uint8_t layer, uint8_t row, uint8_t col, uint16_t keycode);

// Raw byte-buffer access over the keymap (offset/size in bytes). Returns false
// if offset+size exceeds the keymap size. set_buffer flushes to EEPROM.
bool dynamic_keymap_get_buffer(uint16_t offset, uint8_t size, uint8_t *dst);
bool dynamic_keymap_set_buffer(uint16_t offset, uint8_t size, const uint8_t *src);

// Whole-keymap blob access — dst/src must hold dynamic_keymap_size() bytes.
// set_all flushes once (used by the inter-half config transfer, Phase 20,
// which moves the keymap as a single blob; avoids the uint8_t size cap of
// set_buffer for keymaps larger than 255 bytes). Return false on NULL.
bool dynamic_keymap_get_all(uint8_t *dst);
bool dynamic_keymap_set_all(const uint8_t *src);

// Restore the cache to the compiled default for `side` and flush to EEPROM.
void dynamic_keymap_reset(cmk_side_t side);

#endif // CMK_DYNAMIC_KEYMAP_H
