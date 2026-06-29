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
// rgblight.h — Underglow RGB animation engine (Phase 13)
//
// HSV-based color system with multiple animation effects.
// Config persisted via eeconfig. Velocikey modulates animation speed.
//

#ifndef CMK_RGBLIGHT_H
#define CMK_RGBLIGHT_H

#include "config/cmk_config.h"

#ifdef RGBLIGHT_ENABLE

#include "rgb_driver.h"
#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Effect modes
// ---------------------------------------------------------------------------

typedef enum {
    RGB_EFFECT_STATIC = 0,      // solid color
    RGB_EFFECT_BREATHING,       // pulse brightness
    RGB_EFFECT_RAINBOW_MOOD,    // hue rotates slowly, all LEDs same phase
    RGB_EFFECT_RAINBOW_SWIRL,   // hue rotates with per-LED offset
    RGB_EFFECT_SNAKE,           // segment scrolls along strip
    RGB_EFFECT_KNIGHT,          // knight rider bounce
    RGB_EFFECT_TWINKLE,         // random brightness pulses
    RGB_EFFECT_COUNT
} rgb_effect_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

void rgblight_init(void);
void rgblight_tick(uint32_t now_ticks);

// Toggle & mode
void rgblight_toggle(void);
void rgblight_step(void);           // next mode
void rgblight_step_reverse(void);   // prev mode

// HSV control
void rgblight_increase_hue(void);
void rgblight_decrease_hue(void);
void rgblight_increase_sat(void);
void rgblight_decrease_sat(void);
void rgblight_increase_val(void);
void rgblight_decrease_val(void);
void rgblight_set_hsv(uint8_t h, uint8_t s, uint8_t v);

// Speed control
void rgblight_increase_speed(void);
void rgblight_decrease_speed(void);

// Getters
bool    rgblight_is_enabled(void);
uint8_t rgblight_get_mode(void);

#endif // RGBLIGHT_ENABLE

#endif // CMK_RGBLIGHT_H
