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
// power_manager.c — Phase 17 skeleton
//
// Active today:
//   - ADC init + USB-C CC1/CC2/VBUS readings (RP2350B ADC5/6/7 on GP45/46/47).
//   - GPIO config for charger interrupt / status / power-good / source-select.
//   - I2C0 register access helpers for BQ24296 (charger, 0x6B) and BQ27441
//     (fuel gauge, 0x55). The bus is already brought up by eeprom_init();
//     this file only issues transactions on it.
//
// Commented out until the battery is wired:
//   - The poll task only reads USB-C ADC values for now. Battery / charger
//     I2C reads sit inside a guarded #if 0 block so they do not run on a
//     bare proto board where the chips may behave unpredictably.
//

#include "power_manager.h"
#include "i2c0_bus.h"

#include "config/board.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/time.h"        // sleep_ms() — config-update runs pre-scheduler
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "diag.h"

#include <string.h>

// 1024 words (4KB): the DEBUG_POWER two-line printf with ~12 conversions needs
// more stack than the original 512-word skeleton budget — at 512 the task
// overflowed during the first print and never resumed (no further [PM] ticks).
#define PM_TASK_STACK_WORDS  1024
// Priority 2 (same as keyscan/connection_manager): at the original idle+1 the
// task printed once at boot then went silent — it never got the CPU again once
// the priority-2 tasks were up. Round-robin time-slicing at 2 guarantees it runs.
#define PM_TASK_PRIORITY     2
#define PM_POLL_MS           1000
#define PM_LOG_EVERY_N       5        // DBG_POWER full-dump cadence: every 5 ticks = 5 s
#define PM_BATTERY_LOG_EVERY_N 3      // DBG_BATTERY cadence: every 3 ticks = 3 s
#define CHG_MAINTAIN_EVERY_N 5        // charger re-assert + watchdog kick: every 5 ticks = 5 s
#define PM_I2C_TIMEOUT_US    2000

// ADC reference voltage in mV (RP2350B ties AREF to 3V3).
#define PM_ADC_VREF_MV       3300
#define PM_ADC_FULL_SCALE    4095u   // 12-bit

// VBUS sense uses an external 10k/10k resistor divider — node = VBUS/2,
// so the pin voltage must be doubled to recover the bus voltage.
#define PM_VBUS_DIVIDER_NUM  2
#define PM_VBUS_DIVIDER_DEN  1

// --- BQ27441-G1A fuel gauge — standard commands (16-bit little-endian) ------
#define FG_CMD_CONTROL        0x00
#define FG_CMD_TEMPERATURE    0x02   // 0.1 K
#define FG_CMD_VOLTAGE        0x04   // mV
#define FG_CMD_FLAGS          0x06
#define FG_CMD_AVG_CURRENT    0x10   // signed mA
#define FG_CMD_SOC            0x1C   // %

// Control() subcommands (written LE16 to FG_CMD_CONTROL).
#define FG_SUBCMD_UNSEAL         0x8000   // written twice to leave the SEALED state
#define FG_SUBCMD_SET_CFGUPDATE  0x0013
#define FG_SUBCMD_SOFT_RESET     0x0042

// Flags() bits.
#define FG_FLAG_CFGUPMODE     (1u << 4)

// Extended data-memory (block data) access registers.
#define FG_EXT_DATA_CONTROL   0x61   // BlockDataControl (write 0x00 to enable)
#define FG_EXT_DATA_CLASS     0x3E   // DataBlockClass (subclass select)
#define FG_EXT_DATA_BLOCK     0x3F   // DataBlock (32-byte block offset)
#define FG_EXT_BLOCK_START    0x40   // first byte of the 32-byte block window
#define FG_EXT_CHECKSUM       0x60   // BlockDataChecksum
#define FG_BLOCK_SIZE         32

#define FG_SUBCLASS_STATE     0x52   // subclass 82 — holds Design Capacity/Energy
#define FG_OFF_DESIGN_CAP     10     // byte offset within State block (mAh, BE16)
#define FG_OFF_DESIGN_ENERGY  12     // byte offset within State block (mWh, BE16)

// --- BQ24296 charger — register fields --------------------------------------
#define CHG_REG_INPUT_SRC     0x00   // Input Source Control (EN_HIZ, VINDPM, IINLIM)
#define CHG_REG_POWER_ON      0x01   // Power-On Config (RESET, CHG_CONFIG, SYS_MIN)
#define CHG_REG_CHARGE_TERM   0x05   // Charge Term/Timer (WATCHDOG, safety timer)
#define CHG_REG_STATUS        0x08   // System Status
#define CHG_REG_FAULT         0x09   // New Fault
#define CHG_STATUS_PG         (1u << 2)        // PG_STAT: power good
#define CHG_STATUS_CHRG_MASK  (0x3u << 4)      // CHRG_STAT[1:0]
#define CHG_STATUS_CHRG_PRE   (0x1u << 4)      // pre-charge
#define CHG_STATUS_CHRG_FAST  (0x2u << 4)      // fast charging
#define CHG_STATUS_CHRG_DONE  (0x3u << 4)      // charge termination done (battery full)

// REG00 IINLIM[2:0] codes (input current limit) + EN_HIZ (bit7).
#define CHG_IINLIM_MASK       0x07u
#define CHG_IINLIM_500MA      0x02u  // 010
#define CHG_IINLIM_1500MA     0x05u  // 101
#define CHG_IINLIM_2000MA     0x06u  // 110
#define CHG_EN_HIZ            (1u << 7)        // REG00 bit7: High-Impedance (input FET off)
// REG01 fields. NEVER set bit7 (REGISTER_RESET) — it reverts everything.
#define CHG_REG_RESET         (1u << 7)
#define CHG_WD_RESET          (1u << 6)        // REG01 bit6: I2C watchdog timer reset (kick, auto-clear)
#define CHG_CHGCONFIG_MASK    (0x3u << 4)      // OTG_CONFIG[5] | CHG_CONFIG[4]; 01 = charge
#define CHG_CHGCONFIG_CHARGE  (0x1u << 4)      // 01 = charge battery
// REG05 WATCHDOG[5:4]: 00 = disabled, 01 = 40 s, 10 = 80 s, 11 = 160 s.
#define CHG_WATCHDOG_MASK     (0x3u << 4)
#define CHG_WATCHDOG_40S      (0x1u << 4)      // keep the watchdog ENABLED (40 s safety fallback)

// USB-C CC voltage thresholds (sink side) → advertised source current.
#define CC_THRESH_CONNECT_MV  250    // below = no Type-C source advertising
#define CC_THRESH_1500MA_MV   700    // 0.70..1.31 V = 1.5 A
#define CC_THRESH_3000MA_MV   1310   // >= 1.31 V = 3.0 A

// Cached snapshots populated by the poll task.
static pm_charger_status_t s_charger = {0};
static pm_battery_status_t s_battery = {0};

// Critical-battery latch with hysteresis (set/cleared in the poll task).
static volatile bool s_battery_critical = false;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static inline uint16_t adc_raw_to_mv(uint16_t raw) {
    return (uint16_t)((uint32_t)raw * PM_ADC_VREF_MV / PM_ADC_FULL_SCALE);
}

static uint16_t adc_read_channel_mv(uint8_t channel) {
    adc_select_input(channel);
    uint16_t raw = adc_read();
    return adc_raw_to_mv(raw);
}

// ---------------------------------------------------------------------------
// USB-C sense
// ---------------------------------------------------------------------------

uint16_t pm_read_usb_cc1_mv(void) {
    return adc_read_channel_mv(ADC_CHAN_USB_CC1);
}

uint16_t pm_read_usb_cc2_mv(void) {
    return adc_read_channel_mv(ADC_CHAN_USB_CC2);
}

uint16_t pm_read_vbus_mv(void) {
    uint16_t pin_mv = adc_read_channel_mv(ADC_CHAN_USB_VUSB);
    return (uint16_t)((uint32_t)pin_mv * PM_VBUS_DIVIDER_NUM / PM_VBUS_DIVIDER_DEN);
}

// ---------------------------------------------------------------------------
// I2C register access — BQ24296 charger (0x6B)
// ---------------------------------------------------------------------------

// Charger/fuel gauge are low-priority clients on I2C0 — they back off
// immediately (NO_WAIT) if the EEPROM is currently using the bus. Missing
// one 1 Hz sample is harmless; bus integrity for persistence is not.
bool pm_charger_read_reg(uint8_t reg, uint8_t *out) {
    if (!out) return false;
    if (!i2c0_bus_take(I2C0_BUS_NO_WAIT)) return false;
    int w = i2c_write_timeout_us(i2c0, I2C_ADDR_CHARGER, &reg, 1, true,
                                 PM_I2C_TIMEOUT_US);
    int r = -1;
    if (w == 1) {
        r = i2c_read_timeout_us(i2c0, I2C_ADDR_CHARGER, out, 1, false,
                                PM_I2C_TIMEOUT_US);
    }
    i2c0_bus_give();
    return r == 1;
}

bool pm_charger_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    if (!i2c0_bus_take(I2C0_BUS_NO_WAIT)) return false;
    int w = i2c_write_timeout_us(i2c0, I2C_ADDR_CHARGER, buf, 2, false,
                                 PM_I2C_TIMEOUT_US);
    i2c0_bus_give();
    return w == 2;
}

void pm_get_charger_status(pm_charger_status_t *out) {
    if (out) *out = s_charger;
}

bool pm_is_charging(void) {
    return s_charger.charging;
}

// Refresh s_charger from REG08 (System Status). CHRG_STAT[1:0] distinguishes
// not-charging / pre-charge / fast-charge / done; PG_STAT marks 5V present.
//
// On a skipped/failed I2C read we KEEP the previous charging/power_good values
// rather than falling back to the open-drain GPIO STAT pin. The STAT pin is
// noisy (blinks on fault, floats between states) and the old fallback made the
// FN battery LED flicker between steady and blinking. A missed 1 Hz sample is
// harmless; the next successful REG08 read corrects the state.
static void charger_refresh(void) {
    uint8_t reg = 0;
    bool ok08 = pm_charger_read_reg(CHG_REG_STATUS, &reg);
    if (ok08) {
        s_charger.status_reg = reg;
        s_charger.power_good = (reg & CHG_STATUS_PG) != 0;
        uint8_t chrg = reg & CHG_STATUS_CHRG_MASK;
        s_charger.charging  = (chrg == CHG_STATUS_CHRG_PRE) || (chrg == CHG_STATUS_CHRG_FAST);
    }
    // else: keep last known state (no GPIO fallback).

    // Config registers — captured for the [PM] CHG diagnostic; keep-last-on-fail.
    if (pm_charger_read_reg(CHG_REG_FAULT,       &reg)) s_charger.fault_reg = reg;
    if (pm_charger_read_reg(CHG_REG_INPUT_SRC,   &reg)) s_charger.input_reg = reg;
    if (pm_charger_read_reg(CHG_REG_POWER_ON,    &reg)) s_charger.cfg_reg   = reg;
    if (pm_charger_read_reg(CHG_REG_CHARGE_TERM, &reg)) s_charger.timer_reg = reg;
}

// Re-assert the charge configuration and KICK the I2C watchdog. Called on the
// CHG_MAINTAIN_EVERY_N cadence (~5 s) whenever USB power is present.
//
// Design (watchdog kept ENABLED as a safety net):
//   1. REG05 — watchdog set to 40 s. If the MCU ever hangs/dies, the watchdog
//      expires and the charger reverts to POR defaults (EN_HIZ=0, IINLIM≈100 mA,
//      charge on) so the cell keeps charging unattended.
//   2. REG00 — FORCE EN_HIZ=0 (never just preserve it) + apply the dynamic IINLIM
//      from the USB-C CC advertisement (preserve VINDPM).
//   3. REG01 — assert CHG_CONFIG=charge, keep REGISTER_RESET(bit7)=0, and set the
//      watchdog-reset bit6 to KICK the timer. Written EVERY cycle: the kick keeps
//      the watchdog from expiring during normal operation, so the settings stick.
//
// CRITICAL (Bug C): the BQ24296 AUTO-SETS EN_HIZ=1 (REG00[7]) in hardware when the
// input is seen as a 100 mA USB host with a good battery, and LATCHES it in host
// mode — it stays in HIZ (REG08 reads 0x00, no charging) until the host writes
// REG00[7]=0, even across an MCU reboot (separate IC). TI's datasheet (§8.3.1.3.5,
// §8.4.1.1) explicitly recommends the host clear EN_HIZ on wake-up. The previous
// code only PRESERVED bit7, so once latched the cell never charged again.
static void charger_maintain(uint16_t cc_mv, uint16_t vbus_mv) {
    if (vbus_mv < 4000) return;   // no USB input → nothing to configure

    // 1. Keep the I2C watchdog ENABLED at 40 s (preserve EN_TERM/EN_TIMER/CHG_TIMER).
    uint8_t reg05 = 0;
    if (pm_charger_read_reg(CHG_REG_CHARGE_TERM, &reg05)) {
        uint8_t w = (uint8_t)((reg05 & ~CHG_WATCHDOG_MASK) | CHG_WATCHDOG_40S);
        if (w != reg05) {
            pm_charger_write_reg(CHG_REG_CHARGE_TERM, w);
            DBG_POWER("[PM] CHG  set: watchdog 40s (REG05 0x%02X -> 0x%02X)\n", reg05, w);
        }
    }

    // 2. REG00: force EN_HIZ=0 + input current limit from the USB-C CC advertisement
    //    (preserve VINDPM). Write whenever EN_HIZ is set or IINLIM differs.
    uint8_t want_iinlim;
    if      (cc_mv >= CC_THRESH_3000MA_MV) want_iinlim = CHG_IINLIM_2000MA; // 3A adv → cap 2A (proto-safe)
    else if (cc_mv >= CC_THRESH_1500MA_MV) want_iinlim = CHG_IINLIM_1500MA;
    else                                   want_iinlim = CHG_IINLIM_500MA;  // Default USB / unknown

    uint8_t reg00 = 0;
    if (pm_charger_read_reg(CHG_REG_INPUT_SRC, &reg00)) {
        uint8_t w = (uint8_t)((reg00 & ~(CHG_IINLIM_MASK | CHG_EN_HIZ)) | want_iinlim);
        if (w != reg00) {
            pm_charger_write_reg(CHG_REG_INPUT_SRC, w);
            DBG_POWER("[PM] CHG  set: REG00 0x%02X -> 0x%02X (IINLIM=0x%X%s)\n",
                      reg00, w, (unsigned)want_iinlim,
                      (reg00 & CHG_EN_HIZ) ? ", EN_HIZ 1->0" : "");
        }
    }

    // 3. REG01: assert CHG_CONFIG=charge, keep REGISTER_RESET(bit7)=0, and KICK the
    //    watchdog (bit6=1). Preserve SYS_MIN. Written every cycle for the kick;
    //    re-asserting charge while the battery is full is idempotent (the charger
    //    handles termination/recharge itself).
    uint8_t reg01 = 0;
    if (pm_charger_read_reg(CHG_REG_POWER_ON, &reg01)) {
        uint8_t w = (uint8_t)((reg01 & ~(CHG_CHGCONFIG_MASK | CHG_REG_RESET))
                              | CHG_CHGCONFIG_CHARGE | CHG_WD_RESET);
        pm_charger_write_reg(CHG_REG_POWER_ON, w);   // always: this is the watchdog kick
        DBG_POWER("[PM] CHG  set: REG01 0x%02X -> 0x%02X (charge on, WD kick)\n", reg01, w);
    }
}

// ---------------------------------------------------------------------------
// I2C register access — BQ27441 fuel gauge (0x55)
// ---------------------------------------------------------------------------
//
// Standard commands are 16-bit little-endian: write the 1-byte command, then
// read two bytes (LSB first).
//
bool pm_fuel_gauge_read_word(uint8_t cmd, uint16_t *out) {
    if (!out) return false;
    if (!i2c0_bus_take(I2C0_BUS_NO_WAIT)) return false;
    int w = i2c_write_timeout_us(i2c0, I2C_ADDR_FUEL_GAUGE, &cmd, 1, true,
                                 PM_I2C_TIMEOUT_US);
    uint8_t buf[2] = {0};
    int r = -1;
    if (w == 1) {
        r = i2c_read_timeout_us(i2c0, I2C_ADDR_FUEL_GAUGE, buf, 2, false,
                                PM_I2C_TIMEOUT_US);
    }
    i2c0_bus_give();
    if (r != 2) return false;
    *out = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return true;
}

void pm_get_battery_status(pm_battery_status_t *out) {
    if (out) *out = s_battery;
}

bool pm_battery_is_critical(void) {
    return s_battery_critical;
}

// Refresh s_battery from the fuel gauge and update the critical latch with
// hysteresis. Each field keeps its previous value if its read is skipped.
static void battery_refresh(void) {
    uint16_t v_mv = 0, soc = 0;
    int16_t  i_ma = 0, temp_dk = 0;
    bool got_soc = false;

    if (pm_fuel_gauge_read_word(FG_CMD_VOLTAGE, &v_mv))               s_battery.voltage_mv = v_mv;
    if (pm_fuel_gauge_read_word(FG_CMD_SOC, &soc))                  { s_battery.soc_pct = (uint8_t)(soc > 100 ? 100 : soc); got_soc = true; }
    if (pm_fuel_gauge_read_word(FG_CMD_AVG_CURRENT, (uint16_t *)&i_ma)) s_battery.current_ma = i_ma;
    if (pm_fuel_gauge_read_word(FG_CMD_TEMPERATURE, (uint16_t *)&temp_dk))
        s_battery.temperature_c10 = (int16_t)(temp_dk - 2731);     // 0.1 K → 0.1 °C

    if (got_soc) {
        s_battery.valid = true;
        if (s_battery.soc_pct <= PM_BATTERY_CRITICAL_PCT)            s_battery_critical = true;
        else if (s_battery.soc_pct >= PM_BATTERY_CRITICAL_RECOVER_PCT) s_battery_critical = false;
        // Between the two thresholds: keep the latch (hysteresis).
    }
}

// ---------------------------------------------------------------------------
// Fuel gauge config-update — set Design Capacity / Energy for the real cell
// ---------------------------------------------------------------------------
//
// The BQ27441-G1A ships UNSEALED with a 1340 mAh default profile. To match the
// 2500 mAh cell we rewrite Design Capacity/Energy in the State subclass (82)
// data block. Sequence per the BQ27441-G1A TRM (SLUUAC9), mirrored from the
// Zephyr bq274xx driver. Run once at boot (pre-scheduler, single-threaded) and
// only when the values actually differ — the data memory is flash-backed, so
// we must not rewrite it on every boot.
//
// Note: Control() subcommands are LITTLE-endian; data-block values are BIG-endian.

// Low-level block-data primitives (each takes/gives the I2C0 mutex; pre-scheduler
// the mutex is a no-op, so NO_WAIT always succeeds during init).
static bool fg_write_u8(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    if (!i2c0_bus_take(I2C0_BUS_NO_WAIT)) return false;
    int w = i2c_write_timeout_us(i2c0, I2C_ADDR_FUEL_GAUGE, buf, 2, false, PM_I2C_TIMEOUT_US);
    i2c0_bus_give();
    return w == 2;
}

static bool fg_write_control(uint16_t subcmd) {
    uint8_t buf[3] = { FG_CMD_CONTROL, (uint8_t)(subcmd & 0xFF), (uint8_t)(subcmd >> 8) };
    if (!i2c0_bus_take(I2C0_BUS_NO_WAIT)) return false;
    int w = i2c_write_timeout_us(i2c0, I2C_ADDR_FUEL_GAUGE, buf, 3, false, PM_I2C_TIMEOUT_US);
    i2c0_bus_give();
    return w == 3;
}

static bool fg_open_state_block(void) {
    if (!fg_write_u8(FG_EXT_DATA_CONTROL, 0x00)) { DBG_POWER("[PM]   FG: BlockDataControl write NAK\n"); return false; }
    if (!fg_write_u8(FG_EXT_DATA_CLASS, FG_SUBCLASS_STATE)) { DBG_POWER("[PM]   FG: DataBlockClass write NAK\n"); return false; }
    if (!fg_write_u8(FG_EXT_DATA_BLOCK, 0x00)) { DBG_POWER("[PM]   FG: DataBlock write NAK\n"); return false; }
    sleep_ms(5);                                                        // mandatory settle
    return true;
}

// Read the 32-byte block one byte at a time (write pointer + read 1 byte) — the
// same transaction shape as the working standard-command reads. The BQ27441
// NAKs a 32-byte burst read (observed r=-2), so we never burst.
static bool fg_read_byte(uint8_t reg, uint8_t *out) {
    if (!i2c0_bus_take(I2C0_BUS_NO_WAIT)) return false;
    int w = i2c_write_timeout_us(i2c0, I2C_ADDR_FUEL_GAUGE, &reg, 1, true, PM_I2C_TIMEOUT_US);
    int r = -1;
    if (w == 1) r = i2c_read_timeout_us(i2c0, I2C_ADDR_FUEL_GAUGE, out, 1, false, PM_I2C_TIMEOUT_US);
    i2c0_bus_give();
    return r == 1;
}

static bool fg_read_block(uint8_t *buf32) {
    for (uint8_t i = 0; i < FG_BLOCK_SIZE; i++) {
        if (!fg_read_byte((uint8_t)(FG_EXT_BLOCK_START + i), &buf32[i])) {
            DBG_POWER("[PM]   FG: block byte 0x%02X read failed\n", FG_EXT_BLOCK_START + i);
            return false;
        }
    }
    return true;
}

// Poll Flags() until CFGUPMODE reaches the requested state (100 x 25ms = 2.5s).
static bool fg_wait_cfgupmode(bool want_set) {
    for (int i = 0; i < 100; i++) {
        uint16_t flags = 0;
        if (pm_fuel_gauge_read_word(FG_CMD_FLAGS, &flags)) {
            if (((flags & FG_FLAG_CFGUPMODE) != 0) == want_set) return true;
        }
        sleep_ms(25);
    }
    return false;
}

static bool fg_read_design_capacity(uint16_t *cap) {
    uint8_t block[FG_BLOCK_SIZE];
    if (!fg_open_state_block() || !fg_read_block(block)) return false;
    *cap = ((uint16_t)block[FG_OFF_DESIGN_CAP] << 8) | block[FG_OFF_DESIGN_CAP + 1];
    return true;
}

// The BQ27441-G1A ships SEALED: Voltage()/SoC() and other standard commands
// answer, but data-memory (block data) access is blocked. Defensive unseal (key
// 0x8000 written twice) before touching the State subclass — a no-op on a part
// that ships unsealed (as ours does), needed if a unit ships sealed. Left
// unsealed afterwards (the firmware never writes data memory at runtime).
static void fg_unseal(void) {
    fg_write_control(FG_SUBCMD_UNSEAL);
    fg_write_control(FG_SUBCMD_UNSEAL);
    sleep_ms(5);
}

static void pm_fuel_gauge_configure(void) {
    fg_unseal();

    uint16_t cur_cap = 0;
    if (!fg_read_design_capacity(&cur_cap)) {
        DBG_POWER("[PM] fuel gauge: data-memory read failed (bus busy / no cell?)\n");
        return;
    }
    if (cur_cap == PM_BATTERY_DESIGN_CAPACITY_MAH) {
        DBG_POWER("[PM] fuel gauge: design capacity already %u mAh - skip\n", cur_cap);
        return;
    }
    DBG_POWER("[PM] fuel gauge: design capacity %u -> %u mAh (config update)\n",
              cur_cap, (unsigned)PM_BATTERY_DESIGN_CAPACITY_MAH);

    if (!fg_write_control(FG_SUBCMD_SET_CFGUPDATE) || !fg_wait_cfgupmode(true)) {
        DBG_POWER("[PM] fuel gauge: CFGUPDATE entry failed\n");
        return;
    }

    uint8_t block[FG_BLOCK_SIZE];
    if (!fg_open_state_block() || !fg_read_block(block)) {
        DBG_POWER("[PM] fuel gauge: block read failed\n");
        fg_write_control(FG_SUBCMD_SOFT_RESET);
        return;
    }

    block[FG_OFF_DESIGN_CAP]        = (uint8_t)(PM_BATTERY_DESIGN_CAPACITY_MAH >> 8);
    block[FG_OFF_DESIGN_CAP + 1]    = (uint8_t)(PM_BATTERY_DESIGN_CAPACITY_MAH & 0xFF);
    block[FG_OFF_DESIGN_ENERGY]     = (uint8_t)(PM_BATTERY_DESIGN_ENERGY_MWH >> 8);
    block[FG_OFF_DESIGN_ENERGY + 1] = (uint8_t)(PM_BATTERY_DESIGN_ENERGY_MWH & 0xFF);

    // Write only the 4 changed bytes (each a 2-byte reg+val write — the proven
    // shape; no burst). Then write the checksum over the full 32-byte block.
    bool wok = true;
    for (uint8_t off = FG_OFF_DESIGN_CAP; off <= FG_OFF_DESIGN_ENERGY + 1; off++)
        wok &= fg_write_u8((uint8_t)(FG_EXT_BLOCK_START + off), block[off]);
    if (!wok) {
        DBG_POWER("[PM] fuel gauge: block write failed\n");
        fg_write_control(FG_SUBCMD_SOFT_RESET);
        return;
    }
    sleep_ms(5);

    // Checksum = 0xFF - (sum of all 32 block bytes).
    uint8_t sum = 0;
    for (int i = 0; i < FG_BLOCK_SIZE; i++) sum += block[i];
    fg_write_u8(FG_EXT_CHECKSUM, (uint8_t)(0xFF - sum));
    sleep_ms(5);

    if (!fg_write_control(FG_SUBCMD_SOFT_RESET) || !fg_wait_cfgupmode(false)) {
        DBG_POWER("[PM] fuel gauge: CFGUPDATE exit failed\n");
        return;
    }

    uint16_t verify = 0;
    if (fg_read_design_capacity(&verify))
        DBG_POWER("[PM] fuel gauge: design capacity now %u mAh\n", verify);
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void power_manager_init(void) {
    // --- Charger IC status / control pins -----------------------------------
    // BC_INT  (GP37): interrupt output from charger, active LOW → input + pull-up
    gpio_init(PIN_BC_INT);
    gpio_set_dir(PIN_BC_INT, GPIO_IN);
    gpio_pull_up(PIN_BC_INT);

    // BC_STAT (GP38): charging status (open-drain on BQ24296) → input + pull-up
    gpio_init(PIN_BC_STAT);
    gpio_set_dir(PIN_BC_STAT, GPIO_IN);
    gpio_pull_up(PIN_BC_STAT);

    // BC_PGN  (GP39): power-good, active LOW → input + pull-up
    gpio_init(PIN_BC_PGN);
    gpio_set_dir(PIN_BC_PGN, GPIO_IN);
    gpio_pull_up(PIN_BC_PGN);

    // BC_PSEL (GP40): input source select. BQ24296 default:
    //   PSEL HIGH → 500 mA cap (USB SDP / host computer)
    //   PSEL LOW  → 3 A cap (DCP / CDP / dedicated adapter)
    // Without a battery, every mA of load (MCU + CYW43 + WS2812) must come
    // from the USB input. The WS2812 strip alone can pull ~480 mA at peak,
    // which together with the CYW43 TX bursts blows past 500 mA → SYS rail
    // browns out → MCU resets. For development with a 1.5A+ adapter we
    // default LOW. A future revision should detect adapter type via the
    // BQ24296 D+/D- engine and set this dynamically.
    gpio_init(PIN_BC_PSEL);
    gpio_set_dir(PIN_BC_PSEL, GPIO_OUT);
    gpio_put(PIN_BC_PSEL, 0);

    // --- ADC — USB-C CC1 / CC2 / VBUS sense ---------------------------------
    adc_init();
    adc_gpio_init(PIN_USB_CC1);
    adc_gpio_init(PIN_USB_CC2);
    adc_gpio_init(PIN_USB_VUSB);

    // I2C0 is already initialised by eeprom_init() (same bus, 400 kHz). We
    // intentionally do NOT call i2c_init() here so the EEPROM driver stays
    // the single owner of bus configuration.

    // Probe the two power-management ICs (gated logging only).
    uint8_t  in = 0, c1 = 0, t5 = 0, st = 0, fl = 0;
    uint16_t fg_voltage = 0;
    bool charger_ok = pm_charger_read_reg(CHG_REG_INPUT_SRC, &in);
    pm_charger_read_reg(CHG_REG_POWER_ON,    &c1);
    pm_charger_read_reg(CHG_REG_CHARGE_TERM, &t5);
    pm_charger_read_reg(CHG_REG_STATUS,      &st);
    pm_charger_read_reg(CHG_REG_FAULT,       &fl);
    bool fg_ok = pm_fuel_gauge_read_word(FG_CMD_VOLTAGE, &fg_voltage);
    DBG_POWER("[PM] charger=%s fuel_gauge=%s (v=%u mV)\n",
              charger_ok ? "OK" : "MISSING",
              fg_ok      ? "OK" : "MISSING", fg_voltage);
    // Initial charger state — reveals whether EN_HIZ is already latched at boot
    // (host-mode HIZ survives an MCU reboot; charger_maintain clears it).
    DBG_POWER("[PM] CHG (init)  REG00=0x%02X[HIZ=%u IINLIM=0x%X]  REG01=0x%02X[CHG_CFG=%u]  "
              "REG05=0x%02X[WD=%u]  REG08=0x%02X  REG09=0x%02X\n",
              in, (in >> 7) & 1u, in & CHG_IINLIM_MASK,
              c1, (c1 >> 4) & 3u, t5, (t5 >> 4) & 3u, st, fl);

    // Reprogram the fuel gauge for the 2500 mAh cell (no-op if already set, or
    // if no cell answers). Safe here: pre-scheduler, single-threaded, I2C0 idle.
    pm_fuel_gauge_configure();
}

// ---------------------------------------------------------------------------
// Poll task — 1 Hz
// ---------------------------------------------------------------------------

static void power_manager_task(void *arg) {
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(PM_POLL_MS);
    uint32_t   tick_count = 0;

    for (;;) {
        // One-shot stack high-watermark on the first pass (matches the other
        // tasks' boot diagnostics; gated by DEBUG_POWER).
        static bool s_hwm_logged = false;
        if (!s_hwm_logged) {
            s_hwm_logged = true;
            DBG_POWER("[PM] Stack high-watermark: %lu words free (stack=%u words)\n",
                      (unsigned long)uxTaskGetStackHighWaterMark(NULL),
                      (unsigned)PM_TASK_STACK_WORDS);
        }

        // --- USB-C sense (ADC is local, no bus) ---------------------------
        uint16_t cc1_mv  = pm_read_usb_cc1_mv();
        uint16_t cc2_mv  = pm_read_usb_cc2_mv();
        uint16_t vbus_mv = pm_read_vbus_mv();

        // --- Battery + charger (always sampled; updates the cache that the FN
        //     overlay, the critical RGB blank and the BLE battery service read) ---
        charger_refresh();
        battery_refresh();

        // Keep the charger alive on the ~5 s maintain cadence: re-assert the charge
        // config (force EN_HIZ=0, dynamic IINLIM, charge on) and kick the 40 s I2C
        // watchdog, so it charges instead of draining.
        if ((tick_count % CHG_MAINTAIN_EVERY_N) == 0) {
            charger_maintain((cc1_mv > cc2_mv ? cc1_mv : cc2_mv), vbus_mv);
        }

#if DEBUG_BATTERY
        // Compact battery-only line at 3 s cadence (independent of DEBUG_POWER).
        if ((tick_count % PM_BATTERY_LOG_EVERY_N) == 0) {
            DBG_BATTERY("[BAT] soc=%u%% v=%u mV i=%d mA chg=%d%s\n",
                        s_battery.soc_pct, s_battery.voltage_mv, s_battery.current_ma,
                        s_charger.charging, s_battery_critical ? " CRIT" : "");
        }
#endif

#if DEBUG_POWER
        // Full dump (USB-C + charger registers) at 5 s cadence. Single printf so
        // PICO_STDOUT_MUTEX serialises the whole block.
        if ((tick_count % PM_LOG_EVERY_N) == 0) {
            uint8_t in = s_charger.input_reg, c1 = s_charger.cfg_reg, t5 = s_charger.timer_reg;
            uint8_t st = s_charger.status_reg, fl = s_charger.fault_reg;
            DBG_POWER("[PM] USB  CC1=%u mV  CC2=%u mV  VBUS=%u mV\n"
                      "[PM] BAT  v=%u mV  soc=%u%%  i=%d mA  t=%d.%d C   pg=%d chg=%d  st=0x%02X fault=0x%02X\n"
                      "[PM] CHG  REG00=0x%02X[HIZ=%u IINLIM=0x%X]  REG01=0x%02X[CHG_CFG=%u]  "
                      "REG05=0x%02X[WD=%u]  REG08=0x%02X[VBUS=%u CHRG=%u PG=%u]  REG09=0x%02X\n",
                      cc1_mv, cc2_mv, vbus_mv,
                      s_battery.voltage_mv, s_battery.soc_pct, s_battery.current_ma,
                      s_battery.temperature_c10 / 10, s_battery.temperature_c10 % 10,
                      s_charger.power_good, s_charger.charging, st, fl,
                      in, (in >> 7) & 1u, in & CHG_IINLIM_MASK,
                      c1, (c1 >> 4) & 3u,
                      t5, (t5 >> 4) & 3u,
                      st, (st >> 6) & 3u, (st >> 4) & 3u, (st >> 2) & 1u, fl);
        }
#else
        (void)cc1_mv; (void)cc2_mv; (void)vbus_mv;
#endif

        tick_count++;
        (void)tick_count;   // referenced only by the (optional) debug log gates
        DIAG_TASK_ALIVE(DIAG_TASK_PM);
        vTaskDelay(period);
    }
}

void power_manager_create_task(void) {
    xTaskCreate(power_manager_task,
                "PowerMgr",
                PM_TASK_STACK_WORDS,
                NULL,
                PM_TASK_PRIORITY,
                NULL);
}
