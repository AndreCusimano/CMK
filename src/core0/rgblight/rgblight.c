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
// rgblight.c — Underglow RGB animation engine (Phase 13)
//

#include "config/cmk_config.h"

#ifdef RGBLIGHT_ENABLE

#include "rgblight.h"
#include "rgb_driver.h"
#include "eeconfig.h"
#include "velocikey.h"

#include <string.h>

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static struct {
    uint8_t  enable;
    uint8_t  mode;
    uint8_t  hue;
    uint8_t  sat;
    uint8_t  val;
    uint8_t  speed;
} s_cfg;

static uint32_t s_last_frame;   // tick of last rendered frame
static uint32_t s_frame_count;  // monotonic frame counter for effects
static bool     s_dirty;        // config changed, needs persist

// ---------------------------------------------------------------------------
// Persist
// ---------------------------------------------------------------------------

static void rgblight_persist(void) {
    eeconfig_rgb_t ec = {
        .enable = s_cfg.enable,
        .mode   = s_cfg.mode,
        .hue    = s_cfg.hue,
        .sat    = s_cfg.sat,
        .val    = s_cfg.val,
        .speed  = s_cfg.speed,
    };
    eeconfig_update_rgb(&ec);
    s_dirty = false;
}

// ---------------------------------------------------------------------------
// Effect functions — each computes HSV for one LED
// ---------------------------------------------------------------------------

static hsv_t effect_static(hsv_t base, uint32_t frame, uint16_t led) {
    (void)frame; (void)led;
    return base;
}

static hsv_t effect_breathing(hsv_t base, uint32_t frame, uint16_t led) {
    (void)led;
    // Triangle wave on value: 0→255→0 over 256 frames
    uint8_t phase = (uint8_t)(frame & 0xFF);
    uint8_t breath = (phase < 128) ? (phase * 2) : ((255 - phase) * 2);
    base.v = (uint8_t)((uint16_t)base.v * breath / 255);
    return base;
}

static hsv_t effect_rainbow_mood(hsv_t base, uint32_t frame, uint16_t led) {
    (void)led;
    base.h = (uint8_t)(frame & 0xFF);
    return base;
}

static hsv_t effect_rainbow_swirl(hsv_t base, uint32_t frame, uint16_t led) {
    base.h = (uint8_t)((frame + led * (256 / RGB_LED_COUNT)) & 0xFF);
    return base;
}

static hsv_t effect_snake(hsv_t base, uint32_t frame, uint16_t led) {
    uint16_t head = (frame / 2) % RGB_LED_COUNT;
    uint16_t tail_len = (RGB_LED_COUNT > 3) ? 3 : 1;
    // Distance from head (wrapping)
    uint16_t dist = (led >= head) ? (led - head) : (RGB_LED_COUNT - head + led);
    if (dist > tail_len) {
        base.v = 0;
    } else {
        base.v = (uint8_t)((uint16_t)base.v * (tail_len - dist) / tail_len);
    }
    return base;
}

static hsv_t effect_knight(hsv_t base, uint32_t frame, uint16_t led) {
    // Bounce position: 0 → (N-1) → 0 over 2*(N-1) frames
    uint16_t period = (RGB_LED_COUNT > 1) ? (RGB_LED_COUNT - 1) * 2 : 1;
    uint16_t pos = (frame / 2) % period;
    if (pos >= RGB_LED_COUNT) pos = period - pos;
    int16_t dist = (int16_t)led - (int16_t)pos;
    if (dist < 0) dist = -dist;
    if (dist > 2) {
        base.v = 0;
    } else {
        base.v = (uint8_t)((uint16_t)base.v * (3 - dist) / 3);
    }
    return base;
}

static hsv_t effect_twinkle(hsv_t base, uint32_t frame, uint16_t led) {
    // Simple pseudo-random per-LED brightness variation
    uint16_t seed = (frame + led * 37) & 0xFF;
    uint8_t brightness = (seed < 64) ? 255 : (seed < 128) ? 128 : 32;
    base.v = (uint8_t)((uint16_t)base.v * brightness / 255);
    return base;
}

// Effect dispatch table
typedef hsv_t (*effect_fn_t)(hsv_t base, uint32_t frame, uint16_t led);

static const effect_fn_t s_effects[RGB_EFFECT_COUNT] = {
    [RGB_EFFECT_STATIC]       = effect_static,
    [RGB_EFFECT_BREATHING]    = effect_breathing,
    [RGB_EFFECT_RAINBOW_MOOD] = effect_rainbow_mood,
    [RGB_EFFECT_RAINBOW_SWIRL]= effect_rainbow_swirl,
    [RGB_EFFECT_SNAKE]        = effect_snake,
    [RGB_EFFECT_KNIGHT]       = effect_knight,
    [RGB_EFFECT_TWINKLE]      = effect_twinkle,
};

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void rgblight_init(void) {
    eeconfig_rgb_t ec = eeconfig_read_rgb();
    s_cfg.enable = ec.enable;
    s_cfg.mode   = ec.mode;
    s_cfg.hue    = ec.hue;
    s_cfg.sat    = ec.sat;
    s_cfg.val    = ec.val;
    s_cfg.speed  = ec.speed;

    if (s_cfg.mode >= RGB_EFFECT_COUNT) {
        s_cfg.mode = RGB_EFFECT_STATIC;
    }

    s_last_frame  = 0;
    s_frame_count = 0;
    s_dirty       = false;

    if (!s_cfg.enable) {
        rgb_driver_clear();
    }
}

// ---------------------------------------------------------------------------
// Tick — called at 1kHz from action_tick()
// ---------------------------------------------------------------------------

void rgblight_tick(uint32_t now_ticks) {
    if (!s_cfg.enable) return;

    // Frame rate limiting
    uint32_t interval = RGB_FRAME_INTERVAL_MS;

    // Velocikey: scale interval by typing speed
    if (velocikey_is_enabled()) {
        uint8_t spd = velocikey_get_speed();
        // Faster typing → shorter interval (faster animation)
        // spd=0 → interval*1, spd=255 → interval/256 (clamped to 1)
        interval = interval * 256 / (spd + 256);
        if (interval < 1) interval = 1;
    }

    // Speed config: scale interval (higher speed → shorter interval)
    // speed=0 → 4x slower, speed=128 → 1x, speed=255 → 0.5x
    interval = interval * 256 / (s_cfg.speed + 128);
    if (interval < 1) interval = 1;

    if ((now_ticks - s_last_frame) < interval) return;
    s_last_frame = now_ticks;
    s_frame_count++;

    // Render frame
    hsv_t base = { .h = s_cfg.hue, .s = s_cfg.sat, .v = s_cfg.val };
    effect_fn_t fn = s_effects[s_cfg.mode];

    for (uint16_t i = 0; i < RGB_LED_COUNT; i++) {
        hsv_t led_hsv = fn(base, s_frame_count, i);
        rgb_t led_rgb = hsv_to_rgb(led_hsv);
        rgb_driver_set_led(i, led_rgb);
    }

    rgb_driver_flush();

    // Deferred persist (avoid EEPROM write during rendering)
    if (s_dirty) {
        rgblight_persist();
    }
}

// ---------------------------------------------------------------------------
// Toggle & mode
// ---------------------------------------------------------------------------

void rgblight_toggle(void) {
    s_cfg.enable = !s_cfg.enable;
    if (!s_cfg.enable) {
        rgb_driver_clear();
    }
    s_dirty = true;
}

void rgblight_step(void) {
    s_cfg.mode = (s_cfg.mode + 1) % RGB_EFFECT_COUNT;
    s_dirty = true;
}

void rgblight_step_reverse(void) {
    s_cfg.mode = (s_cfg.mode == 0) ? (RGB_EFFECT_COUNT - 1) : (s_cfg.mode - 1);
    s_dirty = true;
}

// ---------------------------------------------------------------------------
// HSV control
// ---------------------------------------------------------------------------

void rgblight_increase_hue(void) {
    s_cfg.hue += RGB_HUE_STEP;  // wraps naturally at 255→0
    s_dirty = true;
}

void rgblight_decrease_hue(void) {
    s_cfg.hue -= RGB_HUE_STEP;
    s_dirty = true;
}

void rgblight_increase_sat(void) {
    uint16_t sat = s_cfg.sat + RGB_SAT_STEP;
    s_cfg.sat = (sat > 255) ? 255 : (uint8_t)sat;
    s_dirty = true;
}

void rgblight_decrease_sat(void) {
    s_cfg.sat = (s_cfg.sat < RGB_SAT_STEP) ? 0 : (s_cfg.sat - RGB_SAT_STEP);
    s_dirty = true;
}

void rgblight_increase_val(void) {
    uint16_t val = s_cfg.val + RGB_VAL_STEP;
    s_cfg.val = (val > RGB_BRIGHTNESS_MAX) ? RGB_BRIGHTNESS_MAX : (uint8_t)val;
    s_dirty = true;
}

void rgblight_decrease_val(void) {
    s_cfg.val = (s_cfg.val < RGB_VAL_STEP) ? 0 : (s_cfg.val - RGB_VAL_STEP);
    s_dirty = true;
}

void rgblight_set_hsv(uint8_t h, uint8_t s, uint8_t v) {
    s_cfg.hue = h;
    s_cfg.sat = s;
    s_cfg.val = v;
    s_dirty = true;
}

// ---------------------------------------------------------------------------
// Speed control
// ---------------------------------------------------------------------------

void rgblight_increase_speed(void) {
    uint16_t spd = s_cfg.speed + RGB_SPD_STEP;
    s_cfg.speed = (spd > 255) ? 255 : (uint8_t)spd;
    s_dirty = true;
}

void rgblight_decrease_speed(void) {
    s_cfg.speed = (s_cfg.speed < RGB_SPD_STEP) ? 0 : (s_cfg.speed - RGB_SPD_STEP);
    s_dirty = true;
}

// ---------------------------------------------------------------------------
// Getters
// ---------------------------------------------------------------------------

bool rgblight_is_enabled(void) {
    return s_cfg.enable;
}

uint8_t rgblight_get_mode(void) {
    return s_cfg.mode;
}

#endif // RGBLIGHT_ENABLE
