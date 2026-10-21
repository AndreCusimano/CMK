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
// macro_storage.c — see macro_storage.h for the design.
//

#include "macro_storage.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"
#include "eeprom.h"

#include <stdio.h>
#include <string.h>

#ifdef MACRO_STORAGE_ENABLE

// EEPROM region layout @ MACRO_STORAGE_ADDR:
//   [0] magic low   \
//   [1] magic high   | 4-byte header
//   [2] version      |
//   [3] reserved (0) /
//   [4..] raw macro bytes (MACRO_BUFFER_SIZE)
#define MS_HEADER_SIZE  4
#define MS_TOTAL_SIZE   (MS_HEADER_SIZE + MACRO_BUFFER_SIZE)

// RAM cache — factory state: all zeros = MACRO_COUNT empty macros.
static uint8_t s_macro_buf[MACRO_BUFFER_SIZE];

// EEPROM write that respects the 256-byte page boundary (mirror of
// dk_eeprom_write_paged in dynamic_keymap.c).
static void ms_eeprom_write_paged(uint16_t addr, const uint8_t *src, size_t len) {
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

// Flush header + full buffer (init factory reset / RESET_MACROS only).
static void ms_flush(void) {
    uint8_t hdr[MS_HEADER_SIZE];
    hdr[0] = (uint8_t)(MACRO_STORAGE_MAGIC & 0xFF);
    hdr[1] = (uint8_t)(MACRO_STORAGE_MAGIC >> 8);
    hdr[2] = MACRO_STORAGE_VERSION;
    hdr[3] = 0;
    ms_eeprom_write_paged(MACRO_STORAGE_ADDR, hdr, MS_HEADER_SIZE);
    ms_eeprom_write_paged(MACRO_STORAGE_ADDR + MS_HEADER_SIZE,
                          s_macro_buf, MACRO_BUFFER_SIZE);
}

// Persist only the touched byte range (per-chunk SET — R6 pattern).
static void ms_persist_range(uint16_t offset, size_t len) {
    ms_eeprom_write_paged((uint16_t)(MACRO_STORAGE_ADDR + MS_HEADER_SIZE + offset),
                          s_macro_buf + offset, len);
}

void macro_storage_init(void) {
    // R4: retried reads + explicit error path — a bus glitch must not look like
    // a stale region (which would wipe the user's macros with zeros).
    uint8_t hdr[MS_HEADER_SIZE];
    if (!eeprom_read_retry(MACRO_STORAGE_ADDR, hdr, MS_HEADER_SIZE, 3)) {
        printf("[MS] ERRORE lettura EEPROM — macro vuote in RAM (no flush)\n");
        memset(s_macro_buf, 0, sizeof(s_macro_buf));
        return;
    }

    uint16_t magic = (uint16_t)hdr[0] | ((uint16_t)hdr[1] << 8);
    if (magic == (uint16_t)MACRO_STORAGE_MAGIC && hdr[2] == MACRO_STORAGE_VERSION) {
        if (!eeprom_read_retry(MACRO_STORAGE_ADDR + MS_HEADER_SIZE,
                               s_macro_buf, MACRO_BUFFER_SIZE, 3)) {
            printf("[MS] ERRORE lettura buffer macro — vuote in RAM\n");
            memset(s_macro_buf, 0, sizeof(s_macro_buf));
        }
    } else {
        // Uninitialised or stale region → factory reset (this region only).
        DBG_MEM("[MS] magic/version mismatch — factory reset macro region\n");
        memset(s_macro_buf, 0, sizeof(s_macro_buf));
        ms_flush();
    }
}

uint16_t macro_storage_size(void)  { return (uint16_t)MACRO_BUFFER_SIZE; }
uint8_t  macro_storage_count(void) { return (uint8_t)MACRO_COUNT; }

bool macro_storage_get_buffer(uint16_t offset, uint8_t size, uint8_t *dst) {
    if (!dst || (uint32_t)offset + size > MACRO_BUFFER_SIZE) return false;
    memcpy(dst, s_macro_buf + offset, size);
    return true;
}

bool macro_storage_set_buffer(uint16_t offset, uint8_t size, const uint8_t *src) {
    if (!src || (uint32_t)offset + size > MACRO_BUFFER_SIZE) return false;
    memcpy(s_macro_buf + offset, src, size);
    ms_persist_range(offset, size);
    return true;
}

const uint8_t *macro_storage_data(void) {
    return s_macro_buf;
}

const uint8_t *macro_storage_find(uint8_t n) {
    if (n >= MACRO_COUNT) return NULL;
    const uint8_t *p   = s_macro_buf;
    const uint8_t *end = s_macro_buf + MACRO_BUFFER_SIZE;
    while (n > 0) {
        while (p < end && *p != 0x00) p++;   // skip to this macro's terminator
        if (p >= end) return NULL;           // fewer than n terminators
        p++;                                 // past the NUL
        n--;
    }
    return (p < end) ? p : NULL;
}

void macro_storage_reset(void) {
    memset(s_macro_buf, 0, sizeof(s_macro_buf));
    ms_flush();
    DBG_MEM("[MS] macro region reset\n");
}

#else  // !MACRO_STORAGE_ENABLE — inert stubs

void     macro_storage_init(void)  { }
uint16_t macro_storage_size(void)  { return 0; }
uint8_t  macro_storage_count(void) { return 0; }
bool macro_storage_get_buffer(uint16_t offset, uint8_t size, uint8_t *dst) {
    (void)offset; (void)size; (void)dst; return false;
}
bool macro_storage_set_buffer(uint16_t offset, uint8_t size, const uint8_t *src) {
    (void)offset; (void)size; (void)src; return false;
}
const uint8_t *macro_storage_data(void)        { return 0; }
const uint8_t *macro_storage_find(uint8_t n)   { (void)n; return 0; }
void macro_storage_reset(void)                 { }

#endif // MACRO_STORAGE_ENABLE
