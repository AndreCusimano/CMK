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
// rgb_driver.h — WS2812 PIO driver wrapper (Phase 13)
//
// Low-level LED buffer + PIO flush. One LED = 24-bit GRB.
// Call rgb_driver_init() once, then set_led() + flush() per frame.
//

#ifndef CMK_RGB_DRIVER_H
#define CMK_RGB_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct { uint8_t r, g, b; } rgb_t;

// Init PIO0 state machine for WS2812 on PIN_RGB.
void rgb_driver_init(void);

// Set color of a single LED in the buffer (no output until flush).
void rgb_driver_set_led(uint16_t idx, rgb_t color);

// Set all LEDs to the same color.
void rgb_driver_set_all(rgb_t color);

// Push the entire buffer to PIO (blocking, ~30us per LED).
void rgb_driver_flush(void);

// Turn off all LEDs immediately.
void rgb_driver_clear(void);

// Gate the 5V LED boost rail (PIN_VLED_EN). Pull LOW in low-power sleep to drop
// the boost regulator quiescent draw; HIGH before driving WS2812 data.
void rgb_driver_power(bool on);

// ---------------------------------------------------------------------------
// HSV type + conversion (shared by rgblight, rgb_matrix, led_matrix)
// ---------------------------------------------------------------------------

typedef struct { uint8_t h, s, v; } hsv_t;

// HSV → RGB conversion (QMK-compatible 6-sector hue wheel).
rgb_t hsv_to_rgb(hsv_t hsv);

#endif // CMK_RGB_DRIVER_H
