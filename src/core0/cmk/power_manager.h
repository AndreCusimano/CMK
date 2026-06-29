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
// power_manager.h — Phase 17 (HW: BQ27441 fuel gauge + BQ24296 charger)
//
// Provides ADC readings for USB-C sense (CC1/CC2/VBUS) and I2C drivers for the
// fuel gauge (0x55) and charger IC (0x6B). With the battery wired, the poll
// task samples battery + charger every second (always-on); the SoC/charging
// state feed the FN status overlay, the critical RGB blank, and the BLE
// Battery Service. DEBUG_POWER / DEBUG_BATTERY only gate logging, not sampling.
//

#ifndef CMK_POWER_MANAGER_H
#define CMK_POWER_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

// One-shot init: configures charger status GPIOs, ADC channels for USB-C
// sense, and (eventually) probes the fuel gauge / charger over I2C0.
// Must be called AFTER eeprom_init() — both share I2C0.
void power_manager_init(void);

// Spawn the FreeRTOS poll task on Core 0 (1 Hz cadence).
// Must be called AFTER vTaskStartScheduler() preparation, like the other
// connection_manager_create_task() / keyscan_create_task() calls.
void power_manager_create_task(void);

// ---------------------------------------------------------------------------
// USB-C sense (ADC) — active from day one
// ---------------------------------------------------------------------------

// RP2350B ADC is 12-bit, 3.3V reference. The VBUS rail goes through an
// external divider on the proto board; the returned millivolt value already
uint16_t pm_read_usb_cc1_mv(void);
uint16_t pm_read_usb_cc2_mv(void);
uint16_t pm_read_vbus_mv(void);

// ---------------------------------------------------------------------------
// Charger IC — TI BQ24296MRGER (I2C 0x6B)
// ---------------------------------------------------------------------------

typedef struct {
    bool     power_good;        // 5V input present (REG08 PG_STAT, GPIO /PGN fallback)
    bool     charging;          // actively charging (REG08 CHRG_STAT 01/10)
    uint8_t  status_reg;        // REG08 — last raw read
    uint8_t  fault_reg;         // REG09 — last raw read
    uint8_t  input_reg;         // REG00 — last raw read (EN_HIZ bit7, IINLIM[2:0])
    uint8_t  cfg_reg;           // REG01 — last raw read (CHG_CONFIG[5:4])
    uint8_t  timer_reg;         // REG05 — last raw read (WATCHDOG[5:4])
} pm_charger_status_t;

// Returns last cached snapshot (refreshed by the poll task at 1 Hz).
void pm_get_charger_status(pm_charger_status_t *out);

// True when the charger reports an active charge cycle. Thin accessor used by
// the FN overlay to blink the battery LED while charging.
bool pm_is_charging(void);

// Direct register access — used by the task and the charger interpretation.
bool pm_charger_read_reg (uint8_t reg, uint8_t *out);
bool pm_charger_write_reg(uint8_t reg, uint8_t value);

// ---------------------------------------------------------------------------
// Fuel gauge — TI BQ27441-G1A (I2C 0x55)
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  soc_pct;           // State of Charge 0..100 %
    uint16_t voltage_mv;        // Cell voltage in mV
    int16_t  current_ma;        // Signed: + charging, - discharging
    int16_t  temperature_c10;   // Tenths of °C (e.g. 251 = 25.1 °C)
    bool     valid;             // true once at least one good fuel-gauge read landed
} pm_battery_status_t;

void pm_get_battery_status(pm_battery_status_t *out);

// True when SoC has dropped below PM_BATTERY_CRITICAL_PCT (with hysteresis up to
// PM_BATTERY_CRITICAL_RECOVER_PCT). False until a valid SoC reading exists, so a
// missing/unwired cell never trips it. Drives the RGB matrix power-save blank.
bool pm_battery_is_critical(void);

// 16-bit little-endian register read on BQ27441 (standard command set).
bool pm_fuel_gauge_read_word(uint8_t cmd, uint16_t *out);

#endif // CMK_POWER_MANAGER_H
