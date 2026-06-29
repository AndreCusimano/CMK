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
// i2c0_bus.h — Owner of I2C0 hardware bus and shared FreeRTOS mutex.
//
// I2C0 is physically shared between three devices:
//   - M24M01-R EEPROM     (0x50)  — persistent settings, bonding, eeconfig
//   - BQ24296 charger     (0x6B)  — Phase 17
//   - BQ27441 fuel gauge  (0x55)  — Phase 17
//
// The Pico SDK i2c_*_blocking() APIs are NOT internally serialised, so a
// pre-emptive scheduler with multiple tasks touching the same bus would
// corrupt transactions. This module exposes a mutex that callers must hold
// for the duration of an EEPROM-style read/write OR an I2C transaction
// pair (write address, repeated start, read).
//
// Priority policy:
//   - EEPROM reads/writes  → I2C0_BUS_WAIT_FOREVER (must always succeed)
//   - Charger / fuel gauge → I2C0_BUS_NO_WAIT (skip the cycle if EEPROM
//                            currently holds the bus; power data is sampled
//                            every second anyway, missing one tick is fine)
//
// Pre-scheduler behaviour:
//   The boot sequence calls eeprom_*() before vTaskStartScheduler() (bonding
//   store init, eeconfig init, btstack_tlv init, bootmagic). At that point
//   FreeRTOS is single-threaded so the mutex is a no-op — take/give return
//   immediately without touching the semaphore.
//

#ifndef CMK_I2C0_BUS_H
#define CMK_I2C0_BUS_H

#include <stdbool.h>
#include <stdint.h>

// Sentinel timeouts (in FreeRTOS ticks). Defined as macros so callers don't
// need to pull in FreeRTOS.h just to specify a timeout.
#define I2C0_BUS_NO_WAIT        0u
#define I2C0_BUS_WAIT_FOREVER   0xFFFFFFFFu

// One-shot init: hardware bus (i2c_init + GPIO pin function + pull-ups) and
// FreeRTOS mutex. Must be called BEFORE any I2C transaction on this bus —
// i.e. before eeprom_init(), which is the first I2C consumer in main().
void i2c0_bus_init(void);

// Acquire the bus. Returns true on success, false on timeout.
// timeout_ticks: I2C0_BUS_NO_WAIT (0), I2C0_BUS_WAIT_FOREVER, or any
// FreeRTOS tick count. Before the scheduler starts the function is a no-op
// and always returns true.
bool i2c0_bus_take(uint32_t timeout_ticks);

// Release the bus. Must be called exactly once after a successful take().
void i2c0_bus_give(void);

#endif // CMK_I2C0_BUS_H
