/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
 *
 * Custom board header consumed by the Pico SDK when PICO_BOARD=cmk_proto_v1.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY THE ASSEMBLER
//       SO IT MUST ONLY CONTAIN PREPROCESSOR DIRECTIVES
//       (no C declarations, no comments-as-code).
// -----------------------------------------------------

#ifndef _BOARDS_CMK_PROTO_V1_H
#define _BOARDS_CMK_PROTO_V1_H

pico_board_cmake_set(PICO_PLATFORM, rp2350)
pico_board_cmake_set(PICO_CYW43_SUPPORTED, 1)

// Board identification.
#define CMK_PROTO_V1

// --- RP2350 VARIANT ---
// 0 = RP2350B (QFN-80 package, 48 GPIO) — used by this board.
// 1 = RP2350A (QFN-60 package, 30 GPIO).
// The SDK reads this as `#if PICO_RP2350A` to size GPIO-related tables.
#define PICO_RP2350A 0

// --- UART (debug console) ---
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 2
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 3
#endif

// --- I2C0 (shared bus: M24M01 EEPROM, BQ24296 charger, BQ27441 fuel gauge) ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 0
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 1
#endif

// --- LED (no onboard status LED on this board: status via RGB overlay) ---
// no PICO_DEFAULT_LED_PIN
// no PICO_DEFAULT_WS2812_PIN (driven by the firmware on GP35 — see config/board.h)

// --- FLASH (matches Pico 2W stock W25Q080 4 MiB) ---
#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1
#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif
pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (4 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (4 * 1024 * 1024)
#endif

// --- CYW43 (Infineon CYW43439 BLE/Wi-Fi module) ---
// CYW43 SPI pins cannot be changed at runtime — must be set in this header
// so the pico-sdk consumes them at compile time.
#ifndef CYW43_PIN_WL_DYNAMIC
#define CYW43_PIN_WL_DYNAMIC 0
#endif

#ifndef CYW43_DEFAULT_PIN_WL_REG_ON
#define CYW43_DEFAULT_PIN_WL_REG_ON 31u
#endif
#ifndef CYW43_DEFAULT_PIN_WL_CS
#define CYW43_DEFAULT_PIN_WL_CS 32u
#endif
#ifndef CYW43_DEFAULT_PIN_WL_DATA_OUT
#define CYW43_DEFAULT_PIN_WL_DATA_OUT 33u
#endif
#ifndef CYW43_DEFAULT_PIN_WL_DATA_IN
#define CYW43_DEFAULT_PIN_WL_DATA_IN 33u
#endif
#ifndef CYW43_DEFAULT_PIN_WL_HOST_WAKE
#define CYW43_DEFAULT_PIN_WL_HOST_WAKE 33u
#endif
#ifndef CYW43_DEFAULT_PIN_WL_CLOCK
#define CYW43_DEFAULT_PIN_WL_CLOCK 34u
#endif

// CYW43 internal GPIO mapping (same as Pico 2W stock — VBUS / SMPS / LED).
#ifndef CYW43_WL_GPIO_COUNT
#define CYW43_WL_GPIO_COUNT 3
#endif
#ifndef CYW43_WL_GPIO_LED_PIN
#define CYW43_WL_GPIO_LED_PIN 0
#endif
#ifndef CYW43_WL_GPIO_SMPS_PIN
#define CYW43_WL_GPIO_SMPS_PIN 1
#endif

// RP2350 A2 silicon support.
pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 1)
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

#endif // _BOARDS_CMK_PROTO_V1_H
