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
// btstack_config.h — BTstack configuration for CMK dual-role BLE test
//

#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// Dynamic allocation via malloc (uses pico-sdk system heap, NOT FreeRTOS heap)
#define HAVE_MALLOC

// BLE only — no Classic BT
// pico_btstack_ble already passes -DENABLE_BLE=1 on the command line; guard
// to avoid -Wmacro-redefined when both are present.
#ifndef ENABLE_BLE
#define ENABLE_BLE
#endif
#define ENABLE_LE_PERIPHERAL          // Peripheral role: advertise as HID keyboard
#define ENABLE_LE_CENTRAL             // Central role: connect to slave/mouse
#define ENABLE_LE_SECURE_CONNECTIONS  // BLE SC pairing

// Logging (forwarded to UART via stdio)
#define ENABLE_LOG_ERROR
#define ENABLE_LOG_INFO
#define ENABLE_PRINTF_HEXDUMP



// HCI buffer sizing
#define HCI_ACL_PAYLOAD_SIZE          251
#define HCI_INCOMING_PRE_BUFFER_SIZE  6
#define HCI_OUTGOING_PRE_BUFFER_SIZE  4    // CYW43 requires >= 4 (CYW43_PACKET_HEADER_SIZE)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT  4    // CYW43 requires multiple of 4

// Bonding storage (device DB entries in NVM/flash)
#define NVM_NUM_DEVICE_DB_ENTRIES     8

// Max simultaneous HCI connections:
//   1 = Peripheral (PC host) + 1 = Central (slave) = 2 total
// Without HAVE_MALLOC this sizes static pools; with HAVE_MALLOC it caps dynamic alloc.
#define MAX_NR_HCI_CONNECTIONS        2

// HIDS Client: max concurrent connections as Central
// We connect to one slave at a time
#define MAX_NR_HIDS_CLIENTS           1
#define MAX_NUM_HID_SERVICES          1

#endif // BTSTACK_CONFIG_H
