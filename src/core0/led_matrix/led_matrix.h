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
// led_matrix.h — Per-key monochrome LED engine (Phase 13c)
//
// Same LED config as RGB Matrix but outputs brightness only (white).
// Mutually exclusive with RGBLIGHT_ENABLE and RGB_MATRIX_ENABLE.
//

#ifndef CMK_LED_MATRIX_H
#define CMK_LED_MATRIX_H

#include "config/cmk_config.h"

#ifdef LED_MATRIX_ENABLE

#include "rgb_driver.h"
#include <stdint.h>
#include <stdbool.h>

// LED flags — same as rgb_matrix.h
#define LED_FLAG_NONE       0x00
#define LED_FLAG_KEYLIGHT   0x01
#define LED_FLAG_UNDERGLOW  0x02
#define LED_FLAG_INDICATOR  0x04
#define LED_FLAG_ALL        0xFF

typedef struct {
    uint8_t row;
    uint8_t col;
    uint8_t x;      // physical X (0-224)
    uint8_t y;      // physical Y (0-64)
    uint8_t flags;
} led_config_entry_t;

// Per-side LED map; set at boot by keymap_init() (see core/keymap.c).
extern const led_config_entry_t *g_led_config;

// Hit tracking (reactive)
typedef struct {
    uint8_t  count;
    uint8_t  index[RGB_MATRIX_HIT_MAX];
    uint8_t  x[RGB_MATRIX_HIT_MAX];
    uint8_t  y[RGB_MATRIX_HIT_MAX];
    uint16_t tick[RGB_MATRIX_HIT_MAX];
} led_matrix_hit_t;

// Effect modes (monochrome subset)
typedef enum {
    LED_MX_SOLID = 0,
    LED_MX_BREATHING,
    LED_MX_RAIN,
    LED_MX_REACTIVE_SIMPLE,
    LED_MX_REACTIVE_SPLASH,
    LED_MX_COUNT
} led_matrix_effect_t;

// API
void    led_matrix_init(void);
void    led_matrix_tick(uint32_t now_ticks);
void    led_matrix_record_hit(uint8_t row, uint8_t col);

void    led_matrix_toggle(void);
void    led_matrix_step(void);
void    led_matrix_step_reverse(void);

void    led_matrix_increase_val(void);
void    led_matrix_decrease_val(void);
void    led_matrix_increase_speed(void);
void    led_matrix_decrease_speed(void);

bool    led_matrix_is_enabled(void);
uint8_t led_matrix_get_mode(void);

#endif // LED_MATRIX_ENABLE

#endif // CMK_LED_MATRIX_H
