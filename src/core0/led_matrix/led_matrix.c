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
// led_matrix.c — Per-key monochrome LED engine (Phase 13c)
//

#include "config/cmk_config.h"

#ifdef LED_MATRIX_ENABLE

#include "led_matrix.h"
#include "rgb_driver.h"
#include "eeconfig.h"
#include "velocikey.h"
#include "led_indicators.h"

#include <string.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static struct {
    uint8_t  enable;
    uint8_t  mode;
    uint8_t  val;      // brightness
    uint8_t  speed;
} s_cfg;

static uint32_t          s_last_frame;
static uint32_t          s_frame_count;
static bool              s_dirty;
static led_matrix_hit_t  s_hits;

// ---------------------------------------------------------------------------
// Persist (reuses eeconfig_rgb_t — ignores hue/sat)
// ---------------------------------------------------------------------------

static void led_matrix_persist(void) {
    eeconfig_rgb_t ec = {
        .enable = s_cfg.enable,
        .mode   = s_cfg.mode,
        .hue    = 0,
        .sat    = 0,
        .val    = s_cfg.val,
        .speed  = s_cfg.speed,
    };
    eeconfig_update_rgb(&ec);
    s_dirty = false;
}

// ---------------------------------------------------------------------------
// Hit tracking
// ---------------------------------------------------------------------------

void led_matrix_record_hit(uint8_t row, uint8_t col) {
    for (uint16_t i = 0; i < RGB_LED_COUNT; i++) {
        if (g_led_config[i].row == row && g_led_config[i].col == col) {
            uint8_t slot = s_hits.count;
            if (slot >= RGB_MATRIX_HIT_MAX) {
                for (uint8_t j = 0; j < RGB_MATRIX_HIT_MAX - 1; j++) {
                    s_hits.index[j] = s_hits.index[j + 1];
                    s_hits.x[j]     = s_hits.x[j + 1];
                    s_hits.y[j]     = s_hits.y[j + 1];
                    s_hits.tick[j]  = s_hits.tick[j + 1];
                }
                slot = RGB_MATRIX_HIT_MAX - 1;
            } else {
                s_hits.count++;
            }
            s_hits.index[slot] = (uint8_t)i;
            s_hits.x[slot]     = g_led_config[i].x;
            s_hits.y[slot]     = g_led_config[i].y;
            s_hits.tick[slot]  = 0;
            break;
        }
    }
}

static void hits_tick(void) {
    uint8_t w = 0;
    for (uint8_t r = 0; r < s_hits.count; r++) {
        s_hits.tick[r]++;
        if (s_hits.tick[r] <= RGB_MATRIX_REACTIVE_FADE) {
            if (w != r) {
                s_hits.index[w] = s_hits.index[r];
                s_hits.x[w]     = s_hits.x[r];
                s_hits.y[w]     = s_hits.y[r];
                s_hits.tick[w]  = s_hits.tick[r];
            }
            w++;
        }
    }
    s_hits.count = w;
}

// ---------------------------------------------------------------------------
// Effects — compute brightness (0-255) per LED
// ---------------------------------------------------------------------------

static uint8_t effect_solid(uint8_t base, uint32_t frame, uint16_t led) {
    (void)frame; (void)led;
    return base;
}

static uint8_t effect_breathing(uint8_t base, uint32_t frame, uint16_t led) {
    (void)led;
    uint8_t phase = (uint8_t)(frame & 0xFF);
    uint8_t breath = (phase < 128) ? (phase * 2) : ((255 - phase) * 2);
    return (uint8_t)((uint16_t)base * breath / 255);
}

static uint8_t effect_rain(uint8_t base, uint32_t frame, uint16_t led) {
    uint16_t seed = (frame + led * 37 + g_led_config[led].x * 7) & 0xFF;
    uint8_t brightness = (seed < 32) ? 255 : (seed < 64) ? 128 : 16;
    return (uint8_t)((uint16_t)base * brightness / 255);
}

static uint8_t effect_reactive_simple(uint8_t base, uint32_t frame, uint16_t led) {
    (void)frame;
    for (uint8_t h = 0; h < s_hits.count; h++) {
        if (s_hits.index[h] == led) {
            uint8_t fade = (uint8_t)(255 - (uint16_t)s_hits.tick[h] * 255 / RGB_MATRIX_REACTIVE_FADE);
            if (fade > base) base = fade;
        }
    }
    return base;
}

static uint8_t effect_reactive_splash(uint8_t base, uint32_t frame, uint16_t led) {
    (void)frame;
    uint8_t lx = g_led_config[led].x;
    uint8_t ly = g_led_config[led].y;

    for (uint8_t h = 0; h < s_hits.count; h++) {
        int16_t dx = (int16_t)lx - (int16_t)s_hits.x[h];
        int16_t dy = (int16_t)ly - (int16_t)s_hits.y[h];
        uint16_t dist = (uint16_t)(abs(dx) + abs(dy));
        uint16_t wave = s_hits.tick[h] * 6;

        if (dist <= wave && dist > (wave > 20 ? wave - 20 : 0)) {
            uint8_t intensity = (uint8_t)(255 - (uint16_t)s_hits.tick[h] * 255 / RGB_MATRIX_REACTIVE_FADE);
            if (intensity > base) base = intensity;
        }
    }
    return base;
}

// Dispatch table
typedef uint8_t (*lm_effect_fn_t)(uint8_t base, uint32_t frame, uint16_t led);

static const lm_effect_fn_t s_effects[LED_MX_COUNT] = {
    [LED_MX_SOLID]           = effect_solid,
    [LED_MX_BREATHING]       = effect_breathing,
    [LED_MX_RAIN]            = effect_rain,
    [LED_MX_REACTIVE_SIMPLE] = effect_reactive_simple,
    [LED_MX_REACTIVE_SPLASH] = effect_reactive_splash,
};

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void led_matrix_init(void) {
    eeconfig_rgb_t ec = eeconfig_read_rgb();
    s_cfg.enable = ec.enable;
    s_cfg.mode   = ec.mode;
    s_cfg.val    = ec.val;
    s_cfg.speed  = ec.speed;

    if (s_cfg.mode >= LED_MX_COUNT) {
        s_cfg.mode = LED_MX_SOLID;
    }

    s_last_frame  = 0;
    s_frame_count = 0;
    s_dirty       = false;
    memset(&s_hits, 0, sizeof(s_hits));

    if (!s_cfg.enable) {
        rgb_driver_clear();
    }
}

// ---------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------

void led_matrix_tick(uint32_t now_ticks) {
    if (!s_cfg.enable) return;

    uint32_t interval = RGB_FRAME_INTERVAL_MS;

    if (velocikey_is_enabled()) {
        uint8_t spd = velocikey_get_speed();
        interval = interval * 256 / (spd + 256);
        if (interval < 1) interval = 1;
    }

    interval = interval * 256 / (s_cfg.speed + 128);
    if (interval < 1) interval = 1;

    if ((now_ticks - s_last_frame) < interval) return;
    s_last_frame = now_ticks;
    s_frame_count++;

    hits_tick();

    lm_effect_fn_t fn = s_effects[s_cfg.mode];

    for (uint16_t i = 0; i < RGB_LED_COUNT; i++) {
        uint8_t v = fn(s_cfg.val, s_frame_count, i);
        rgb_driver_set_led(i, (rgb_t){v, v, v});
    }

    // LED Indicator overlay
    uint8_t leds = led_indicators_get_state();
    if (leds) {
        for (uint16_t i = 0; i < RGB_LED_COUNT; i++) {
            if (g_led_config[i].flags & LED_FLAG_INDICATOR) {
                if (leds & 0x07) {  // any indicator active → full brightness
                    rgb_driver_set_led(i, (rgb_t){255, 255, 255});
                }
            }
        }
    }

    rgb_driver_flush();

    if (s_dirty) {
        led_matrix_persist();
    }
}

// ---------------------------------------------------------------------------
// Toggle & mode
// ---------------------------------------------------------------------------

void led_matrix_toggle(void) {
    s_cfg.enable = !s_cfg.enable;
    if (!s_cfg.enable) rgb_driver_clear();
    s_dirty = true;
}

void led_matrix_step(void) {
    s_cfg.mode = (s_cfg.mode + 1) % LED_MX_COUNT;
    s_dirty = true;
}

void led_matrix_step_reverse(void) {
    s_cfg.mode = (s_cfg.mode == 0) ? (LED_MX_COUNT - 1) : (s_cfg.mode - 1);
    s_dirty = true;
}

// ---------------------------------------------------------------------------
// Brightness & speed
// ---------------------------------------------------------------------------

void led_matrix_increase_val(void) {
    uint16_t val = s_cfg.val + RGB_VAL_STEP;
    s_cfg.val = (val > RGB_BRIGHTNESS_MAX) ? RGB_BRIGHTNESS_MAX : (uint8_t)val;
    s_dirty = true;
}

void led_matrix_decrease_val(void) {
    s_cfg.val = (s_cfg.val < RGB_VAL_STEP) ? 0 : (s_cfg.val - RGB_VAL_STEP);
    s_dirty = true;
}

void led_matrix_increase_speed(void) {
    uint16_t spd = s_cfg.speed + RGB_SPD_STEP;
    s_cfg.speed = (spd > 255) ? 255 : (uint8_t)spd;
    s_dirty = true;
}

void led_matrix_decrease_speed(void) {
    s_cfg.speed = (s_cfg.speed < RGB_SPD_STEP) ? 0 : (s_cfg.speed - RGB_SPD_STEP);
    s_dirty = true;
}

// ---------------------------------------------------------------------------
// Getters
// ---------------------------------------------------------------------------

bool led_matrix_is_enabled(void) {
    return s_cfg.enable;
}

uint8_t led_matrix_get_mode(void) {
    return s_cfg.mode;
}

#endif // LED_MATRIX_ENABLE
