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
// eeprom.c — Generic I2C EEPROM driver
//
// Chip-specific parameters (page size, address, etc.) are in eeprom.h.
//

#include "eeprom.h"
#include "i2c0_bus.h"
#include "diag.h"
#include "config/cmk_config.h"

#include "hardware/i2c.h"
#include "pico/time.h"

#include <string.h>

// ---------------------------------------------------------------------------
// eeprom_init — bus hardware is owned by i2c0_bus_init(); we only probe here.
// EEPROM is the priority client on I2C0: every read/write takes the bus
// mutex with WAIT_FOREVER. Other clients (charger/fuel gauge) use NO_WAIT.
// ---------------------------------------------------------------------------

bool eeprom_init(void) {
    // Probe: read 1 byte from current address.
    // ACK = EEPROM present, NACK = absent.
    uint8_t dummy;
    bool ok = false;
    if (i2c0_bus_take(I2C0_BUS_WAIT_FOREVER)) {
        ok = (i2c_read_blocking(i2c0, EEPROM_I2C_ADDR, &dummy, 1, false) == 1);
        i2c0_bus_give();
    }
    return ok;
}

// ---------------------------------------------------------------------------
// Wait for write cycle completion (ACK polling)
// ---------------------------------------------------------------------------

// Caller must already hold i2c0_bus. Returns true when the write cycle is done.
static bool eeprom_wait_ready(void) {
    absolute_time_t deadline = make_timeout_time_ms(EEPROM_WRITE_TIMEOUT_MS);
    uint8_t dummy;
    while (!time_reached(deadline)) {
        if (i2c_read_blocking(i2c0, EEPROM_I2C_ADDR, &dummy, 1, false) == 1) {
            return true;
        }
    }
    return false;   // timeout — EEPROM did not respond
}

// ---------------------------------------------------------------------------
// eeprom_write
// ---------------------------------------------------------------------------

int eeprom_write(uint16_t addr, const uint8_t *src, size_t len) {
    if (len == 0) return 0;

    // Clamp to one page
    if (len > EEPROM_PAGE_SIZE) len = EEPROM_PAGE_SIZE;

    // Buffer: address bytes + data
    uint8_t txbuf[EEPROM_ADDR_BYTES + EEPROM_PAGE_SIZE];

#if EEPROM_ADDR_BYTES == 2
    txbuf[0] = (uint8_t)(addr >> 8);
    txbuf[1] = (uint8_t)(addr & 0xFF);
#else
    txbuf[0] = (uint8_t)(addr & 0xFF);
#endif
    memcpy(&txbuf[EEPROM_ADDR_BYTES], src, len);

    if (!i2c0_bus_take(I2C0_BUS_WAIT_FOREVER)) return -1;
    int w = i2c_write_blocking(i2c0, EEPROM_I2C_ADDR, txbuf,
                               EEPROM_ADDR_BYTES + len, false);
    // Wait for internal write cycle to complete (still holding the bus)
    bool ready = eeprom_wait_ready();
    i2c0_bus_give();

    if (w != (int)(EEPROM_ADDR_BYTES + len) || !ready) {
        DIAG_INC(eeprom_err);
        return -1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// eeprom_read
// ---------------------------------------------------------------------------

int eeprom_read(uint16_t addr, uint8_t *dst, size_t len) {
    if (len == 0) return 0;

    // Set internal address pointer via dummy write
    uint8_t addr_bytes[EEPROM_ADDR_BYTES];
#if EEPROM_ADDR_BYTES == 2
    addr_bytes[0] = (uint8_t)(addr >> 8);
    addr_bytes[1] = (uint8_t)(addr & 0xFF);
#else
    addr_bytes[0] = (uint8_t)(addr & 0xFF);
#endif
    if (!i2c0_bus_take(I2C0_BUS_WAIT_FOREVER)) return -1;
    // nostop=true → repeated START between write and read
    int w = i2c_write_blocking(i2c0, EEPROM_I2C_ADDR, addr_bytes, EEPROM_ADDR_BYTES, true);
    // Sequential read
    int r = i2c_read_blocking(i2c0, EEPROM_I2C_ADDR, dst, len, false);
    i2c0_bus_give();

    if (w != EEPROM_ADDR_BYTES || r != (int)len) {
        DIAG_INC(eeprom_err);
        return -1;
    }
    return 0;
}

bool eeprom_read_retry(uint16_t addr, uint8_t *dst, size_t len, int tries) {
    for (int i = 0; i < tries; i++) {
        if (eeprom_read(addr, dst, len) == 0) return true;
    }
    return false;
}
