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
// tusb_config.h — TinyUSB configuration (Phase 14)
//
// USB HID composite: Keyboard+Mouse+PB (Boot Protocol) + Raw HID (WebHID).
// Runs on Core 0 under FreeRTOS.
//

#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#include "cmk_config.h"
#include "debug_config.h"

#ifdef USB_ENABLE

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

#define CFG_TUSB_RHPORT0_MODE   OPT_MODE_DEVICE

// ---------------------------------------------------------------------------
// OS — FreeRTOS on Core 0
// ---------------------------------------------------------------------------

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS             OPT_OS_FREERTOS
#endif

// ---------------------------------------------------------------------------
// Device class — HID only (no CDC, no MSC)
// ---------------------------------------------------------------------------

#ifdef RAW_HID_ENABLE
#define CFG_TUD_HID             2       // Interface 0: kbd/mouse/PB, Interface 1: Raw HID
#else
#define CFG_TUD_HID             1       // Interface 0: kbd/mouse/PB only
#endif
#define CFG_TUD_CDC             DEBUG_USB_SERIAL
#if DEBUG_USB_SERIAL
#define CFG_TUD_CDC_RX_BUFSIZE  64
#define CFG_TUD_CDC_TX_BUFSIZE  256
#define CFG_TUD_CDC_EP_BUFSIZE  64
#endif
#define CFG_TUD_MSC             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          0

// ---------------------------------------------------------------------------
// Endpoint / buffer sizes
// ---------------------------------------------------------------------------

#define CFG_TUD_ENDPOINT0_SIZE  64
#define CFG_TUD_HID_EP_BUFSIZE  64

// ---------------------------------------------------------------------------
// Memory section (default — no special placement)
// ---------------------------------------------------------------------------

// #define CFG_TUSB_MEM_SECTION
// #define CFG_TUSB_MEM_ALIGN     __attribute__((aligned(4)))

#endif // USB_ENABLE

#endif // TUSB_CONFIG_H
