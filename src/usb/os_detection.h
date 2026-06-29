/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
 * Copyright (C) 2016-2026 QMK Firmware contributors
 * Copyright (C) Jun Wako and TMK Keyboard Firmware contributors
 *
 * This file contains code derived from the QMK Firmware project
 * (https://github.com/qmk/qmk_firmware) and, indirectly, from the
 * TMK Keyboard Firmware by Jun Wako, both distributed under the
 * GNU General Public License version 2.
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
// os_detection.h — Detect host OS from USB string descriptor requests (Phase 14b)
//
// Tracks wLength values from GET_DESCRIPTOR requests during USB enumeration.
// Pattern matching based on QMK os_detection.c.
//

#ifndef CMK_OS_DETECTION_H
#define CMK_OS_DETECTION_H

#include "config/cmk_config.h"

#if defined(USB_ENABLE) && defined(OS_DETECTION_ENABLE)

#include <stdint.h>

typedef enum {
    OS_UNSURE = 0,
    OS_WINDOWS,
    OS_MACOS,
    OS_LINUX,
    OS_IOS,
} os_detected_t;

// Get the currently detected OS (OS_UNSURE until debounce completes).
os_detected_t os_detection_get(void);

// Record a wLength value from a GET_DESCRIPTOR STRING setup packet.
// Called from usb_descriptors.c on every string descriptor request.
void os_detection_record_wlength(uint16_t wlength, uint32_t now_ms);

// Called periodically (~10ms from CM task) to debounce detection.
void os_detection_tick(uint32_t now_ms);

// Weak callback — user can override to auto-set unicode mode, etc.
void os_detected_user(os_detected_t os);

#else

typedef enum { OS_UNSURE = 0 } os_detected_t;
static inline os_detected_t os_detection_get(void) { return OS_UNSURE; }
static inline void os_detection_record_wlength(uint16_t w, uint32_t ms) { (void)w; (void)ms; }
static inline void os_detection_tick(uint32_t now_ms) { (void)now_ms; }

#endif // USB_ENABLE && OS_DETECTION_ENABLE

#endif // CMK_OS_DETECTION_H
