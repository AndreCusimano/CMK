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
// board.h — Hardware definition for the CMK prototype board.
//
// This file is the single source of truth for board-specific hardware:
// GPIO pinout, peripheral selection, I2C device addresses, LED counts.
// It is included first by cmk_config.h, which adds firmware-level
// parameters (debounce, layers, tapping, feature flags).
//
// Target MCU:  Raspberry Pi RP2350B (QFN-80, 48 GPIO).
// Form factor: hand-wired prototype, single half, 4x6 matrix.
//

#ifndef CMK_BOARD_H
#define CMK_BOARD_H

// Diode direction tokens (kept here so board.h has no dependencies on
// firmware modules; matrix.h re-defines the same values defensively).
#ifndef COL2ROW
#define COL2ROW 0
#endif
#ifndef ROW2COL
#define ROW2COL 1
#endif

// ---------------------------------------------------------------------------
// Board identification
// ---------------------------------------------------------------------------

#define CMK_BOARD_NAME       "CMK Proto v1"
#define CMK_BOARD_MCU        "RP2350B"
#define CMK_BOARD_GPIO_COUNT 48

// ---------------------------------------------------------------------------
// Key Matrix — 4 rows x 6 cols (24 keys), COL2ROW with diodes
// ---------------------------------------------------------------------------
//
// Wiring: row pins drive LOW one at a time (output), column pins are inputs
// with pull-up. With COL2ROW the diode cathode points toward the row, so a
// pressed key pulls its column LOW only while its row is selected.
//

#define MATRIX_ROWS         4
#define MATRIX_COLS         6
#define DIODE_DIRECTION     COL2ROW

#define MATRIX_ROW_PINS     { 25, 26, 27, 28 }                  // ROW_0..ROW_3
#define MATRIX_COL_PINS     {  4,  5,  6,  7,  8,  9 }          // COL_0..COL_5

#define MATRIX_IO_DELAY     30      // µs between row select and column read

// ---------------------------------------------------------------------------
// I2C0 — shared bus: EEPROM + charger + fuel gauge
// ---------------------------------------------------------------------------
//
// The GPIO pins (SDA=GP0, SCL=GP1) are declared in boards/cmk_proto_v1.h as
// PICO_DEFAULT_I2C_SDA_PIN/_SCL_PIN so the pico-sdk picks them up natively.
// CMK firmware uses the aliases below for readability.

#define PIN_I2C_SDA         PICO_DEFAULT_I2C_SDA_PIN
#define PIN_I2C_SCL         PICO_DEFAULT_I2C_SCL_PIN
#define I2C_BUS_HZ          400000  // 400 kHz fast mode

// I2C device addresses on this bus
#define I2C_ADDR_EEPROM     0x50    // ST M24M01-R (128 KB, 1 Mbit)
#define I2C_ADDR_CHARGER    0x6B    // TI BQ24296MRGER (charger IC)
#define I2C_ADDR_FUEL_GAUGE 0x55    // TI BQ27441-G1A (fuel gauge)

// ---------------------------------------------------------------------------
// I2C1 — inter-device link (magnetic connector), Phase 20
// ---------------------------------------------------------------------------
//
// A dedicated, hot-pluggable bus, physically separate from I2C0. Today its only
// use is the inter-half configuration transfer (src/core0/cmk/inter_half_sync,
// src/core0/drivers/i2c1_link), but it is meant to be multi-purpose — a future
// trackpad accessory could stream mouse data over it. Devices are produced
// in-house, so the slave address is chosen freely.
//
// GP42/GP43 carry the I2C1 SDA/SCL function on the RP2350B (pin%4 == 2 → I2C1
// SDA, == 3 → I2C1 SCL); both are otherwise unused. Run at 100 kHz for noise
// margin over a magnetic connector. Internal pull-ups act as a fallback when
// the connector board has none.

#define PIN_I2C1_SDA        42      // GP42: I2C1 SDA
#define PIN_I2C1_SCL        43      // GP43: I2C1 SCL
#define I2C1_BUS_HZ         100000  // 100 kHz
#define CMK_I2C1_ADDR       0x42    // 7-bit slave address of a CMK half (free choice)

// ---------------------------------------------------------------------------
// UART0 — debug console (Core 0 only; Core 1 uses IPC log ring buffer)
// ---------------------------------------------------------------------------
//
// TX/RX pins are declared in boards/cmk_proto_v1.h as
// PICO_DEFAULT_UART_TX_PIN=2 / _RX_PIN=3 so stdio_init_all() routes the
// console correctly.

#define UART_BAUD           115200

// ---------------------------------------------------------------------------
// CYW43 (Infineon CYW43439) — wireless module (BLE/Wi-Fi)
// ---------------------------------------------------------------------------
//
// Pin overrides (REG_ON=31, CS=32, DATA=33, CLOCK=34) live in
// boards/cmk_proto_v1.h as CYW43_DEFAULT_PIN_WL_* so the pico-sdk
// cyw43-driver consumes them at compile time.

// ---------------------------------------------------------------------------
// RGB WS2812 strip — 24 LEDs, one per key, serpentine wiring
// ---------------------------------------------------------------------------
//
// Physical wiring (DIN -> DOUT):
//   row 0 (top):     L -> R (LED  0 ..  5)
//   row 1:           R -> L (LED  6 .. 11)
//   row 2:           L -> R (LED 12 .. 17)
//   row 3 (bottom):  R -> L (LED 18 .. 23)
//
// Mapping (logical row,col) -> LED index is centralised in g_led_config
// inside rgb_matrix.c; this file declares only count and data pin.
//
// VLED_EN gates the 5V LED rail and must be driven HIGH before the strip is
// addressed; pull LOW to save power when LEDs are disabled.
//

#define PIN_RGB             35      // GP35: WS2812 DIN
#define PIN_VLED_EN         36      // GP36: 5V LED rail enable (active HIGH)
#define RGB_LED_COUNT       24      // 4 rows x 6 cols, one LED per key

// ---------------------------------------------------------------------------
// Battery charger — TI BQ24296MRGER (I2C addr 0x6B)
// ---------------------------------------------------------------------------

#define PIN_BC_INT          37      // GP37: charger interrupt (active LOW)
#define PIN_BC_STAT         38      // GP38: charging status pin
#define PIN_BC_PGN          39      // GP39: power-good (active LOW)
#define PIN_BC_PSEL         40      // GP40: input source select

// ---------------------------------------------------------------------------
// Fuel gauge — TI BQ27441-G1A (I2C addr 0x55, no dedicated GPIO)
// ---------------------------------------------------------------------------

// (state-of-charge, voltage, current read via I2C only)

// ---------------------------------------------------------------------------
// USB-C sense (ADC) — CC1, CC2, VBUS
// ---------------------------------------------------------------------------
//
// USB role / orientation detection and VBUS presence. The RP2350B exposes ADC
// channels 5/6/7 on GP45/46/47 respectively.
//

#define PIN_USB_CC2         45      // GP45 / ADC5
#define PIN_USB_CC1         46      // GP46 / ADC6
#define PIN_USB_VUSB        47      // GP47 / ADC7

#define ADC_CHAN_USB_CC2    5
#define ADC_CHAN_USB_CC1    6
#define ADC_CHAN_USB_VUSB   7

// ---------------------------------------------------------------------------
// Board side identity sense (ADC) — distinguishes Left vs Right half
// ---------------------------------------------------------------------------
//
// One firmware runs on both halves; which half this is, is sensed at boot from
// an analog level on GP41 / ADC1. The right half ties the pin to an external
// ~1V source; the left half leaves it floating (an internal pull-down holds it
// near 0V). Read as a voltage (not a logic level: ~1V is in the RP2350
// indeterminate input band) and compared to SIDE_SENSE_THRESHOLD_MV.
// See src/core0/cmk/board_side.{h,c}.
//

#define PIN_SIDE_SENSE          41      // GP41 / ADC1
#define ADC_CHAN_SIDE_SENSE     1
#define SIDE_SENSE_THRESHOLD_MV 500     // pin > 500 mV → RIGHT; else LEFT

// ---------------------------------------------------------------------------
// Status indicators
// ---------------------------------------------------------------------------
//
// This board has NO discrete status LEDs. Power / BLE / connection / relay
// state is shown via the RGB matrix as an overlay activated while FN is held
// (implemented through led_indicators + rgb_matrix). The legacy macros
// PIN_LED_POWER / PIN_LED_BLE / PIN_LED_CONN / PIN_LED_RELAY are
// intentionally undefined; code that uses them must guard with #ifdef.
//

#endif // CMK_BOARD_H
