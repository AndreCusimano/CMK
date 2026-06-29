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
// backlight.c — PWM single-channel backlight (Phase 13d)
//

#include "config/cmk_config.h"

#ifdef BACKLIGHT_ENABLE

#include "backlight.h"
#include "eeconfig.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static uint8_t s_level;       // 0 = off, 1-BACKLIGHT_LEVELS = brightness steps

// ---------------------------------------------------------------------------
// PWM output
// ---------------------------------------------------------------------------

static void backlight_set_pwm(uint8_t level) {
    if (level == 0) {
        pwm_set_gpio_level(PIN_BACKLIGHT, 0);
    } else {
        // Map level 1..BACKLIGHT_LEVELS → duty 1/LEVELS..1.0
        uint16_t duty = (uint16_t)level * (65535 / BACKLIGHT_LEVELS);
        pwm_set_gpio_level(PIN_BACKLIGHT, duty);
    }
}

// ---------------------------------------------------------------------------
// Persist (reuses eeconfig_rgb val field)
// ---------------------------------------------------------------------------

static void backlight_persist(void) {
    eeconfig_rgb_t ec = eeconfig_read_rgb();
    ec.enable = (s_level > 0) ? 1 : 0;
    ec.val    = s_level;
    eeconfig_update_rgb(&ec);
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void backlight_init(void) {
    // Configure GPIO for PWM
    gpio_set_function(PIN_BACKLIGHT, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_BACKLIGHT);
    pwm_set_wrap(slice, 65535);
    pwm_set_enabled(slice, true);

    // Load from eeconfig
    eeconfig_rgb_t ec = eeconfig_read_rgb();
    s_level = ec.val;
    if (s_level > BACKLIGHT_LEVELS) {
        s_level = BACKLIGHT_DEFAULT_LEVEL;
    }

    backlight_set_pwm(s_level);
}

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

void backlight_toggle(void) {
    if (s_level > 0) {
        s_level = 0;
    } else {
        s_level = BACKLIGHT_DEFAULT_LEVEL;
    }
    backlight_set_pwm(s_level);
    backlight_persist();
}

void backlight_step(void) {
    s_level = (s_level + 1) % (BACKLIGHT_LEVELS + 1);
    backlight_set_pwm(s_level);
    backlight_persist();
}

void backlight_increase(void) {
    if (s_level < BACKLIGHT_LEVELS) {
        s_level++;
        backlight_set_pwm(s_level);
        backlight_persist();
    }
}

void backlight_decrease(void) {
    if (s_level > 0) {
        s_level--;
        backlight_set_pwm(s_level);
        backlight_persist();
    }
}

void backlight_set(uint8_t level) {
    if (level > BACKLIGHT_LEVELS) level = BACKLIGHT_LEVELS;
    s_level = level;
    backlight_set_pwm(s_level);
    backlight_persist();
}

uint8_t backlight_get(void) {
    return s_level;
}

bool backlight_is_on(void) {
    return s_level > 0;
}

#endif // BACKLIGHT_ENABLE
