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
// dynamic_keymap.c — see dynamic_keymap.h for the design rationale.
//

#include "dynamic_keymap.h"

#include "config/cmk_config.h"
#include "keycode.h"
#include "keymap.h"
#include "eeprom.h"

#include <stdio.h>
#include <string.h>

#ifdef DYNAMIC_KEYMAP_ENABLE

// Keymap size in bytes (one uint16 per cell). Phase 18: the unified 4×12 layout
// (KEYMAP_COLS = 2*MATRIX_COLS) is stored whole on BOTH halves.
#define DK_KEYMAP_BYTES  (NUM_LAYERS * MATRIX_ROWS * KEYMAP_COLS * 2)

// EEPROM region layout @ DYNAMIC_KEYMAP_ADDR:
//   [0]   magic low      \
//   [1]   magic high      | 4-byte header
//   [2]   version         |
//   [3]   reserved (0)    /
//   [4..] raw keymap bytes (DK_KEYMAP_BYTES)
#define DK_HEADER_SIZE   4
#define DK_TOTAL_SIZE    (DK_HEADER_SIZE + DK_KEYMAP_BYTES)

// RAM cache (unified 4×12) — `keymaps` is pointed here by dynamic_keymap_init().
static uint16_t s_keymap[NUM_LAYERS][MATRIX_ROWS][KEYMAP_COLS];

static inline uint8_t *cache_bytes(void) {
    return (uint8_t *)s_keymap;
}

// EEPROM write that respects the 256-byte page boundary (eeprom_write() does
// not auto-wrap across pages). Splits len into page-aligned chunks.
static void dk_eeprom_write_paged(uint16_t addr, const uint8_t *src, size_t len) {
    while (len > 0) {
        uint16_t page_off = (uint16_t)(addr % EEPROM_PAGE_SIZE);
        size_t   chunk    = EEPROM_PAGE_SIZE - page_off;
        if (chunk > len) chunk = len;
        eeprom_write(addr, src, chunk);
        addr += (uint16_t)chunk;
        src  += chunk;
        len  -= chunk;
    }
}

// Flush header + cache to EEPROM (full region — init/seed/reset/set_all only).
static void dk_flush(void) {
    uint8_t buf[DK_TOTAL_SIZE];
    buf[0] = (uint8_t)(DYNAMIC_KEYMAP_MAGIC & 0xFF);
    buf[1] = (uint8_t)(DYNAMIC_KEYMAP_MAGIC >> 8);
    buf[2] = DYNAMIC_KEYMAP_VERSION;
    buf[3] = 0;
    memcpy(&buf[DK_HEADER_SIZE], cache_bytes(), DK_KEYMAP_BYTES);
    dk_eeprom_write_paged(DYNAMIC_KEYMAP_ADDR, buf, DK_TOTAL_SIZE);
}

// R6: persist ONLY the touched byte range. Every per-keycode/per-chunk SET used
// to rewrite the whole 1156-byte region (~50 ms of blocking EEPROM I/O + 5-page
// wear, ×43 chunks for a full keymap write). A 27-byte chunk now costs 1-2 small
// page writes (~3 ms).
static void dk_persist_range(uint16_t offset, size_t len) {
    dk_eeprom_write_paged((uint16_t)(DYNAMIC_KEYMAP_ADDR + DK_HEADER_SIZE + offset),
                          cache_bytes() + offset, len);
}

static void dk_seed_from_default(cmk_side_t side) {
    (void)side;   // unified 4×12 default is side-independent (both halves share it)
    memcpy(cache_bytes(), keymap_default_full(), DK_KEYMAP_BYTES);
}

void dynamic_keymap_init(cmk_side_t side) {
    // R4: retried reads + explicit error path — a bus glitch must not look like
    // a stale region (which would overwrite the user keymap with the default).
    uint8_t hdr[DK_HEADER_SIZE];
    if (!eeprom_read_retry(DYNAMIC_KEYMAP_ADDR, hdr, DK_HEADER_SIZE, 3)) {
        printf("[DK] ERRORE lettura EEPROM — keymap default in RAM (no flush)\n");
        dk_seed_from_default(side);   // RAM only — never flush over a possibly-valid region
        keymaps = (const uint16_t (*)[MATRIX_ROWS][KEYMAP_COLS]) s_keymap;
        return;
    }

    uint16_t magic = (uint16_t)hdr[0] | ((uint16_t)hdr[1] << 8);
    if (magic == (uint16_t)DYNAMIC_KEYMAP_MAGIC && hdr[2] == DYNAMIC_KEYMAP_VERSION) {
        if (!eeprom_read_retry(DYNAMIC_KEYMAP_ADDR + DK_HEADER_SIZE,
                               cache_bytes(), DK_KEYMAP_BYTES, 3)) {
            printf("[DK] ERRORE lettura keymap — default in RAM\n");
            dk_seed_from_default(side);
        }
    } else {
        // Uninitialised or stale region → seed from the compiled default and persist.
        dk_seed_from_default(side);
        dk_flush();
    }

    // Route all keymap consumers through the editable RAM cache (unified 4×12).
    keymaps = (const uint16_t (*)[MATRIX_ROWS][KEYMAP_COLS]) s_keymap;
}

uint16_t dynamic_keymap_size(void) {
    return (uint16_t)DK_KEYMAP_BYTES;
}

uint16_t dynamic_keymap_get_keycode(uint8_t layer, uint8_t row, uint8_t col) {
    if (layer >= NUM_LAYERS || row >= MATRIX_ROWS || col >= KEYMAP_COLS) return KC_NO;
    return s_keymap[layer][row][col];   // col is the GLOBAL logical column 0..11
}

bool dynamic_keymap_set_keycode(uint8_t layer, uint8_t row, uint8_t col, uint16_t keycode) {
    if (layer >= NUM_LAYERS || row >= MATRIX_ROWS || col >= KEYMAP_COLS) return false;
    s_keymap[layer][row][col] = keycode;   // col is the GLOBAL logical column 0..11
    uint16_t off = (uint16_t)(((size_t)&s_keymap[layer][row][col]) - (size_t)cache_bytes());
    dk_persist_range(off, sizeof(uint16_t));
    return true;
}

bool dynamic_keymap_get_buffer(uint16_t offset, uint8_t size, uint8_t *dst) {
    if (!dst || (uint32_t)offset + size > DK_KEYMAP_BYTES) return false;
    memcpy(dst, cache_bytes() + offset, size);
    return true;
}

bool dynamic_keymap_set_buffer(uint16_t offset, uint8_t size, const uint8_t *src) {
    if (!src || (uint32_t)offset + size > DK_KEYMAP_BYTES) return false;
    memcpy(cache_bytes() + offset, src, size);
    dk_persist_range(offset, size);
    return true;
}

bool dynamic_keymap_get_all(uint8_t *dst) {
    if (!dst) return false;
    memcpy(dst, cache_bytes(), DK_KEYMAP_BYTES);
    return true;
}

bool dynamic_keymap_set_all(const uint8_t *src) {
    if (!src) return false;
    memcpy(cache_bytes(), src, DK_KEYMAP_BYTES);
    dk_flush();
    return true;
}

void dynamic_keymap_reset(cmk_side_t side) {
    dk_seed_from_default(side);
    dk_flush();
}

#endif // DYNAMIC_KEYMAP_ENABLE
