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
// eeprom.h — Generic I2C EEPROM driver
//
// Hardware-specific parameters are configured via defines below.
// Change the defines to match your EEPROM chip.
//
// Usage:
//   bool ok = eeprom_init();          // init I2C + probe → false if absent
//   eeprom_write(0x0000, buf, len);   // write (max one page, no auto-wrap)
//   eeprom_read (0x0000, buf, len);   // sequential read (any length)
//

#ifndef CMK_EEPROM_H
#define CMK_EEPROM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// EEPROM chip configuration — change these for a different chip
// ---------------------------------------------------------------------------

// Current chip: ST M24M01-R (1 Mbit = 128 KB)
#define EEPROM_SIZE             131072u   // total bytes (128 KB)
#define EEPROM_PAGE_SIZE        256u      // page write buffer size (bytes)
#define EEPROM_ADDR_BYTES       2         // address width: 2 = 16-bit, 1 = 8-bit
#define EEPROM_I2C_ADDR         0x50u     // 7-bit I2C address (1010 A2 B1, E2=0, block=0)
#define EEPROM_I2C_HZ           400000u   // I2C clock frequency (400 kHz = fast mode)
#define EEPROM_WRITE_TIMEOUT_MS 10        // max write cycle time (spec: 5ms, margin: 10ms)

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Initialise I2C bus and probe the EEPROM.
// Returns true if the EEPROM responds, false if absent.
// Call before any other eeprom_* function.
bool eeprom_init(void);

// Write len bytes from src to EEPROM address addr.
// Data must fit within one page (EEPROM_PAGE_SIZE) — no automatic wrap.
// Blocks until the write cycle completes (ACK polling).
// Call only if eeprom_init() returned true.
// Returns 0 on success, <0 on I2C error (R4 — errors are no longer silent:
// before, a NACKed transfer was indistinguishable from success and a glitch
// could masquerade as a magic mismatch → spurious factory reset).
int eeprom_write(uint16_t addr, const uint8_t *src, size_t len);

// Read len bytes from EEPROM address addr into dst.
// Sequential read — can span multiple pages.
// Call only if eeprom_init() returned true.
// Returns 0 on success, <0 on I2C error. On error dst content is UNDEFINED —
// callers must NOT interpret it (e.g. as a magic mismatch).
int eeprom_read(uint16_t addr, uint8_t *dst, size_t len);

// Read with retry (R4): up to `tries` attempts before giving up. Boot-time
// region loads use this so a single bus glitch can't trigger a factory reset.
bool eeprom_read_retry(uint16_t addr, uint8_t *dst, size_t len, int tries);

#endif // CMK_EEPROM_H
