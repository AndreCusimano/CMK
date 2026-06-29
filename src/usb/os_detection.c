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
// os_detection.c — Detect host OS from USB descriptors (Phase 14b)
//
// Tracks wLength from string descriptor GET_DESCRIPTOR requests.
// Pattern matching based on QMK os_detection.c.
//

#include "config/cmk_config.h"

#if defined(USB_ENABLE) && defined(OS_DETECTION_ENABLE)

#include "os_detection.h"
#include <stdbool.h>

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static struct {
    uint16_t count;       // total requests
    uint16_t cnt_02;      // wLength == 0x02
    uint16_t cnt_04;      // wLength == 0x04
    uint16_t cnt_ff;      // wLength == 0xFF (or >= 0xFF)
    uint8_t  last_wlen;   // last wLength (low byte)
    uint32_t last_req_ms; // timestamp of last request
    os_detected_t detected;
    bool     locked;      // true after debounce
} s_os;

// ---------------------------------------------------------------------------
// Record wLength (called from usb_descriptors.c string descriptor callback)
// ---------------------------------------------------------------------------

void os_detection_record_wlength(uint16_t wlength, uint32_t now_ms) {
    if (s_os.locked) return;

    s_os.count++;
    s_os.last_wlen   = (uint8_t)(wlength & 0xFF);
    s_os.last_req_ms = now_ms;

    if (wlength == 0x02)         s_os.cnt_02++;
    else if (wlength == 0x04)    s_os.cnt_04++;
    if ((wlength & 0xFF) == 0xFF) s_os.cnt_ff++;
}

// ---------------------------------------------------------------------------
// Pattern matching
// ---------------------------------------------------------------------------

static os_detected_t classify(void) {
    if (s_os.count == 0) return OS_UNSURE;

    // Windows: multiple 0xFF requests + at least one 0x04
    if (s_os.cnt_ff >= 2 && s_os.cnt_04 >= 1)
        return OS_WINDOWS;

    // Linux: all requests are 0xFF
    if (s_os.count == s_os.cnt_ff)
        return OS_LINUX;

    // macOS: >= 5 requests, last is 0xFF, has both 0xFF and 0x02
    if (s_os.count >= 5 && s_os.last_wlen == 0xFF &&
        s_os.cnt_ff >= 1 && s_os.cnt_02 >= 2)
        return OS_MACOS;

    // iOS: exactly 4 requests, no 0xFF, two 0x02
    if (s_os.count == 4 && s_os.cnt_ff == 0 && s_os.cnt_02 == 2)
        return OS_IOS;

    return OS_UNSURE;
}

// ---------------------------------------------------------------------------
// Tick — debounce (call from CM task every 10ms)
// ---------------------------------------------------------------------------

void os_detection_tick(uint32_t now_ms) {
    if (s_os.locked) return;
    if (s_os.count == 0) return;

    if ((now_ms - s_os.last_req_ms) >= OS_DETECTION_DEBOUNCE_MS) {
        s_os.detected = classify();
        s_os.locked = true;
        os_detected_user(s_os.detected);
    }
}

// ---------------------------------------------------------------------------
// Public
// ---------------------------------------------------------------------------

os_detected_t os_detection_get(void) {
    return s_os.detected;
}

__attribute__((weak))
void os_detected_user(os_detected_t os) {
    (void)os;
}

#endif // USB_ENABLE && OS_DETECTION_ENABLE
