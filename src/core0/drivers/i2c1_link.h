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
// i2c1_link.h — Owner of the I2C1 inter-device link (magnetic connector).
//
// I2C1 is a dedicated, hot-pluggable bus separate from I2C0 (which carries the
// EEPROM, charger and fuel gauge). It is designed to be multi-purpose — a
// future trackpad accessory could stream mouse data over it — but today its
// only consumer is the inter-half configuration transfer (Phase 20,
// inter_half_sync.{h,c}).
//
// The same I2C1 peripheral is switched between roles at runtime:
//   - IDLE   : peripheral disabled, pins released to inputs/pull-up.
//   - MASTER : i2c_init() — used to push config to the peer half.
//   - SLAVE  : i2c_slave_init() with an ISR handler — used to receive a push.
// At boot both halves stay IDLE; a role is entered only on an explicit user
// keycode (arm-receive → slave, push → master) and dropped back to IDLE when
// the transfer ends, times out or aborts.
//
// Hot-plug safety: there is NO blocking-forever path here. Master transactions
// use i2c_*_timeout_us(), so a detached connector or an absent peer returns an
// error (NACK/timeout) instead of hanging the keyscan task.
//
// Single owner: only inter_half_sync touches I2C1, so unlike i2c0_bus there is
// no mutex — serialisation is implicit in the caller's state machine.
//

#ifndef CMK_I2C1_LINK_H
#define CMK_I2C1_LINK_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "pico/i2c_slave.h"   // i2c_slave_handler_t

// One-shot pin setup (GPIO function + pull-ups on PIN_I2C1_SDA/SCL). Does NOT
// enable the I2C peripheral — call enter_master()/enter_slave() for that.
void i2c1_link_init(void);

// Current peripheral role: 0 = idle, 1 = master, 2 = slave (debug snapshot).
int i2c1_link_role(void);

// Enter MASTER mode (i2c_init at I2C1_BUS_HZ). Idempotent across roles: if a
// role is already active it is dropped first.
void i2c1_link_enter_master(void);

// Enter SLAVE mode at the given 7-bit address; `handler` runs in ISR context.
void i2c1_link_enter_slave(uint8_t addr, i2c_slave_handler_t handler);

// Drop to IDLE: disable slave/master and release the peripheral.
void i2c1_link_idle(void);

// MASTER transactions (timeout-based, hot-plug safe). Return the number of
// bytes transferred, or a negative pico error code (e.g. NACK/timeout).
int i2c1_link_write(uint8_t addr, const uint8_t *src, size_t len, uint32_t timeout_us);
int i2c1_link_read (uint8_t addr, uint8_t *dst, size_t len, uint32_t timeout_us);

// R6: GPIO bus recovery — if the peer holds SDA low (wedged mid-bit by a
// connector glitch), bit-bang up to 9 SCL pulses + a STOP to release it.
// No-op when SDA is already high or a role is active. Call from IDLE, before
// re-entering master after a failed transaction.
void i2c1_link_bus_recover(void);

// Low-power cross-half wake (sub-fase 23d): emit one short LOW pulse on SDA so a
// sibling sleeping in DEEP (falling-edge GPIO IRQ armed on the same pin) wakes.
// Called by the keyscan task right after a local key wakes this half. Restores
// the I2C function afterwards; harmless no-op if the halves are disconnected.
void i2c1_link_wake_pulse(void);

#endif // CMK_I2C1_LINK_H
