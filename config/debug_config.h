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
// debug_config.h — Compile-time debug logging switches
//
// Set to 1 to enable a category, 0 to disable.
// Disabled categories compile to nothing — zero runtime overhead.
//
// Categories:
//   DEBUG_STATUS       BLE connection events, ADV/SCAN state, timeouts, pairing
//   DEBUG_UPSTREAM_K   Keyboard HID reports received from upstream device
//   DEBUG_UPSTREAM_M   Mouse HID reports received from upstream device
//   DEBUG_DOWNSTREAM_K Keyboard HID reports sent downstream (merged upstream+local)
//   DEBUG_DOWNSTREAM_M Mouse HID reports sent downstream (normalised relay)
//   DEBUG_KEY          Local key press/release events (action.c / dynamic macro)
//   DEBUG_MEMORY       EEPROM/bonding_store read/write operations
//   DEBUG_BOOT_TRACE   Step-by-step init progress in main() and action_init()
//   DEBUG_POWER        Power Manager full dump: USB-C ADC + charger + fuel gauge (5 s)
//   DEBUG_BATTERY      Battery level line only: SoC/voltage/current/charging (3 s)
//   DEBUG_SPLIT        Split keyboard: slave TX matrix events, master RX + 4x12 resolve
//

#ifndef CMK_DEBUG_CONFIG_H
#define CMK_DEBUG_CONFIG_H

// All DBG_* macros below expand to printf() when their category is enabled,
// so consumers don't need to remember to #include <stdio.h> themselves.
#include <stdio.h>

// ---------------------------------------------------------------------------
// User settings
// ---------------------------------------------------------------------------

#define DEBUG_STATUS        1   // Connection, ADV/SCAN, pairing, timeout events
#define DEBUG_UPSTREAM_K    0   // Keyboard reports from upstream BLE device
#define DEBUG_UPSTREAM_M    0   // Mouse reports from upstream BLE device
#define DEBUG_DOWNSTREAM_K  0   // Keyboard reports sent to downstream BLE device
#define DEBUG_DOWNSTREAM_M  0   // Mouse reports sent to downstream BLE device
#define DEBUG_KEY           0   // Per-key press/release (action.c) + dynamic macro
#define DEBUG_MEMORY        0   // EEPROM/bonding_store read/write operations (Core 0)
#define DEBUG_BOOT_TRACE    0   // [BOOT]/[ACT] step trace during main() and action_init()
#define DEBUG_POWER         0   // Power Manager: USB-C ADC + fuel gauge / charger full dump (5 s cadence)
#define DEBUG_BATTERY       0   // Battery level only: [BAT] soc/voltage/current/charging (3 s cadence)
#define DEBUG_INTER_HALF    0   // Inter-half I2C config transfer: arm/push/apply, frame status (Phase 20)
#define DEBUG_SPLIT         0   // Split keyboard (Phase 18): slave TX events, master RX + 4x12 resolve
#define DEBUG_DIAG          0   // [DIAG] drop/error counters dump (10 s cadence, only when non-zero) — counters still readable on-demand via GET_DIAG

// ---------------------------------------------------------------------------
// Macros — do not edit below this line
// ---------------------------------------------------------------------------

#if DEBUG_STATUS
#  define DBG_STATUS(...)    printf(__VA_ARGS__)
#else
#  define DBG_STATUS(...)    ((void)0)
#endif

#if DEBUG_UPSTREAM_K
#  define DBG_UP_K(...)      printf(__VA_ARGS__)
#else
#  define DBG_UP_K(...)      ((void)0)
#endif

#if DEBUG_UPSTREAM_M
#  define DBG_UP_M(...)      printf(__VA_ARGS__)
#else
#  define DBG_UP_M(...)      ((void)0)
#endif

#if DEBUG_DOWNSTREAM_K
#  define DBG_DN_K(...)      printf(__VA_ARGS__)
#else
#  define DBG_DN_K(...)      ((void)0)
#endif

#if DEBUG_DOWNSTREAM_M
#  define DBG_DN_M(...)      printf(__VA_ARGS__)
#else
#  define DBG_DN_M(...)      ((void)0)
#endif

#if DEBUG_KEY
#  define DBG_KEY(...)       printf(__VA_ARGS__)
#else
#  define DBG_KEY(...)       ((void)0)
#endif

#if DEBUG_MEMORY
#  define DBG_MEM(...)     printf(__VA_ARGS__)
#else
#  define DBG_MEM(...)     ((void)0)
#endif

#if DEBUG_BOOT_TRACE
#  define DBG_BOOT(...)    printf(__VA_ARGS__)
#else
#  define DBG_BOOT(...)    ((void)0)
#endif

#if DEBUG_POWER
#  define DBG_POWER(...)   printf(__VA_ARGS__)
#else
#  define DBG_POWER(...)   ((void)0)
#endif

#if DEBUG_BATTERY
#  define DBG_BATTERY(...) printf(__VA_ARGS__)
#else
#  define DBG_BATTERY(...) ((void)0)
#endif

#if DEBUG_INTER_HALF
#  define DBG_IH(...)     printf(__VA_ARGS__)
#else
#  define DBG_IH(...)     ((void)0)
#endif

#if DEBUG_SPLIT
#  define DBG_SPLIT(...)  printf(__VA_ARGS__)
#else
#  define DBG_SPLIT(...)  ((void)0)
#endif

// ---------------------------------------------------------------------------
// Core 1 variants — route through IPC log ring buffer (no UART, no mutex).
// Use these in ble_manager.c and core1_entry.c (Core 1 only files).
// Requires ipc/ipc_core.h to be included before using.
// ---------------------------------------------------------------------------

#if DEBUG_STATUS
#  define C1_DBG_STATUS(...)    core1_log(__VA_ARGS__)
#else
#  define C1_DBG_STATUS(...)    ((void)0)
#endif

#if DEBUG_DOWNSTREAM_K
#  define C1_DBG_DN_K(...)      core1_log(__VA_ARGS__)
#else
#  define C1_DBG_DN_K(...)      ((void)0)
#endif

#if DEBUG_DOWNSTREAM_M
#  define C1_DBG_DN_M(...)      core1_log(__VA_ARGS__)
#else
#  define C1_DBG_DN_M(...)      ((void)0)
#endif

#if DEBUG_SPLIT
#  define C1_DBG_SPLIT(...)     core1_log(__VA_ARGS__)
#else
#  define C1_DBG_SPLIT(...)     ((void)0)
#endif

#endif // CMK_DEBUG_CONFIG_H
