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
// i2c1_link.c — see i2c1_link.h for the design rationale.
//

#include "i2c1_link.h"

#include "config/board.h"
#include "config/cmk_config.h"   // LP_WAKE_PULSE_US (cross-half wake)

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/time.h"

// Current role: 0 = idle, 1 = master, 2 = slave.
static int s_role = 0;

int i2c1_link_role(void) {
    return s_role;
}

void i2c1_link_init(void) {
    gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C1_SDA);
    gpio_pull_up(PIN_I2C1_SCL);
}

void i2c1_link_idle(void) {
    if (s_role == 2) {
        i2c_slave_deinit(i2c1);
    }
    if (s_role != 0) {
        i2c_deinit(i2c1);
    }
    s_role = 0;
}

void i2c1_link_enter_master(void) {
    if (s_role != 0) i2c1_link_idle();
    i2c_init(i2c1, I2C1_BUS_HZ);
    s_role = 1;
}

void i2c1_link_enter_slave(uint8_t addr, i2c_slave_handler_t handler) {
    if (s_role != 0) i2c1_link_idle();
    i2c_init(i2c1, I2C1_BUS_HZ);
    i2c_slave_init(i2c1, addr, handler);
    s_role = 2;
}

int i2c1_link_write(uint8_t addr, const uint8_t *src, size_t len, uint32_t timeout_us) {
    return i2c_write_timeout_us(i2c1, addr, src, len, false, timeout_us);
}

int i2c1_link_read(uint8_t addr, uint8_t *dst, size_t len, uint32_t timeout_us) {
    return i2c_read_timeout_us(i2c1, addr, dst, len, false, timeout_us);
}

void i2c1_link_bus_recover(void) {
    // R6: standard I2C bus recovery. If a peer device was left mid-bit by a
    // connector glitch it can hold SDA low forever; no peripheral re-init on
    // OUR side can release the line — only clocking the peer's shifter out.
    if (s_role != 0) return;                 // call only from idle
    if (gpio_get(PIN_I2C1_SDA)) return;      // SDA released — nothing to do

    // Emulate open-drain on SIO: OUT(latched 0) = drive low, IN = released.
    gpio_set_function(PIN_I2C1_SCL, GPIO_FUNC_SIO);
    gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_SIO);
    gpio_put(PIN_I2C1_SCL, 0);
    gpio_put(PIN_I2C1_SDA, 0);
    gpio_set_dir(PIN_I2C1_SCL, GPIO_IN);
    gpio_set_dir(PIN_I2C1_SDA, GPIO_IN);

    // Up to 9 SCL pulses (~10 kHz) until the peer releases SDA…
    for (int i = 0; i < 9 && !gpio_get(PIN_I2C1_SDA); i++) {
        gpio_set_dir(PIN_I2C1_SCL, GPIO_OUT);   // SCL low
        busy_wait_us(50);
        gpio_set_dir(PIN_I2C1_SCL, GPIO_IN);    // SCL high (pull-up)
        busy_wait_us(50);
    }
    // …then a STOP condition (SDA low→high while SCL high).
    gpio_set_dir(PIN_I2C1_SDA, GPIO_OUT);
    busy_wait_us(50);
    gpio_set_dir(PIN_I2C1_SDA, GPIO_IN);
    busy_wait_us(50);

    gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C1_SDA);
    gpio_pull_up(PIN_I2C1_SCL);
}

void i2c1_link_wake_pulse(void) {
    // Low-power cross-half wake (sub-fase 23d): the half that just woke (own key)
    // emits one short LOW pulse on SDA so the sibling — which armed a falling-edge
    // GPIO IRQ on the same pin while in DEEP — wakes too. Same open-drain-on-SIO
    // pattern as i2c1_link_bus_recover; restores the I2C function afterwards.
    // Quiescent in DEEP (the link only transacts on a config push), so this pulse
    // is the only edge the sleeping sibling sees. Harmless no-op if disconnected.
    gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_I2C1_SDA, GPIO_OUT);
    gpio_put(PIN_I2C1_SDA, 0);             // drive LOW → falling edge on the sibling
    busy_wait_us(LP_WAKE_PULSE_US);
    gpio_set_dir(PIN_I2C1_SDA, GPIO_IN);   // release (pull-up restores HIGH)
    gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C1_SDA);
}
