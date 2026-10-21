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
// rgb_matrix.c — Per-key RGB animation engine (Phase 13b)
//

#include "config/cmk_config.h"

#ifdef RGB_MATRIX_ENABLE

#include "rgb_matrix.h"
#include "rgb_driver.h"
#include "eeconfig.h"
#include "velocikey.h"
#include "led_indicators.h"
#include "fn_overlay.h"
#include "power_manager.h"   // pm_battery_is_critical()
#include "board_side.h"      // board_side_get() — unified-X offset for split sync
#include "ipc/ipc_core.h"    // CMK_RGB_SYNC_* frame layout

#include <string.h>
#include <stdlib.h>   // abs()

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
    // Reactive-effect tunables (apply to REACTIVE_SIMPLE / SPLASH / MULTINEXUS)
    uint8_t  wave_speed;   // wave-front propagation (px per tick)
    uint8_t  wave_width;   // ring width (px)
    uint8_t  fade;         // frames until hit fully fades
    uint8_t  hue_shift;    // hue change per tick along the wave
} s_cfg;

static uint32_t          s_last_frame;
static uint32_t          s_frame_count;
static bool              s_dirty;
// Phase 24: unified physical-X offset added to positional effects while the split
// is engaged (RIGHT half only), so the rainbow/swirl/rain gradient is continuous
// across both halves. 0 = local coordinates (standalone or LEFT half).
static uint16_t          s_x_offset;
static bool              s_blanked;   // matrix currently cleared (RGB off, no overlay)
static rgb_matrix_hit_t  s_hits;

// Low-power suspend (set by the low_power module on LIGHT/DEEP idle). While
// suspended the matrix renders nothing and the 5V LED boost rail is cut, unless
// the FN dashboard is held (overlay). s_vled_powered tracks the rail so we
// toggle PIN_VLED_EN only on transitions, not every frame.
static bool              s_suspended;
static bool              s_vled_powered = true;   // VLED_EN HIGH after rgb_driver_init

void rgb_matrix_set_suspended(bool suspended) {
    s_suspended = suspended;
}

// ---------------------------------------------------------------------------
// Persist
// ---------------------------------------------------------------------------

static void rgb_matrix_persist(void) {
    eeconfig_rgb_t ec = {
        .enable     = s_cfg.enable,
        .mode       = s_cfg.mode,
        .hue        = s_cfg.hue,
        .sat        = s_cfg.sat,
        .val        = s_cfg.val,
        .speed      = s_cfg.speed,
        .wave_speed = s_cfg.wave_speed,
        .wave_width = s_cfg.wave_width,
        .fade       = s_cfg.fade,
        .hue_shift  = s_cfg.hue_shift,
    };
    eeconfig_update_rgb(&ec);
    s_dirty = false;
}

// ---------------------------------------------------------------------------
// Hit tracking (reactive effects)
// ---------------------------------------------------------------------------

// Insert a hit at a UNIFIED position (oldest evicted if the ring is full). index
// is the exact LED (REACTIVE_SIMPLE) or 0xFF for a remote key with no local LED.
static void hit_push(uint8_t index, uint16_t ux, uint16_t uy) {
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
    s_hits.index[slot] = index;
    s_hits.x[slot]     = ux;
    s_hits.y[slot]     = uy;
    s_hits.tick[slot]  = 0;
}

void rgb_matrix_record_hit(uint8_t row, uint8_t col) {
    // Find LED index for this key position, store it in UNIFIED coordinates
    // (local X + the split offset) so it shares one space with remote hits.
    for (uint16_t i = 0; i < RGB_LED_COUNT; i++) {
        if (g_led_config[i].row == row && g_led_config[i].col == col) {
            hit_push((uint8_t)i, (uint16_t)(g_led_config[i].x + s_x_offset),
                     g_led_config[i].y);
            break;
        }
    }
}

void rgb_matrix_record_hit_xy(uint16_t ux, uint16_t uy) {
    hit_push(0xFF, ux, uy);   // remote key — no local LED to light exactly
}

static void hits_tick(void) {
    uint8_t w = 0;
    for (uint8_t r = 0; r < s_hits.count; r++) {
        s_hits.tick[r]++;
        if (s_hits.tick[r] <= s_cfg.fade) {
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
// Base effects (non-reactive)
// ---------------------------------------------------------------------------

static hsv_t effect_solid(hsv_t base, uint32_t frame, uint16_t led) {
    (void)frame; (void)led;
    return base;
}

static hsv_t effect_breathing(hsv_t base, uint32_t frame, uint16_t led) {
    (void)led;
    uint8_t phase = (uint8_t)(frame & 0xFF);
    uint8_t breath = (phase < 128) ? (phase * 2) : ((255 - phase) * 2);
    base.v = (uint8_t)((uint16_t)base.v * breath / 255);
    return base;
}

static hsv_t effect_rainbow(hsv_t base, uint32_t frame, uint16_t led) {
    // Hue based on physical X position + time (unified across the split via s_x_offset)
    base.h = (uint8_t)((g_led_config[led].x + s_x_offset + frame) & 0xFF);
    return base;
}

static hsv_t effect_rainbow_swirl(hsv_t base, uint32_t frame, uint16_t led) {
    // Hue rotates with both X position and time, opposite direction
    base.h = (uint8_t)((g_led_config[led].x + s_x_offset - frame * 2) & 0xFF);
    return base;
}

static hsv_t effect_rain(hsv_t base, uint32_t frame, uint16_t led) {
    // Random drops — pseudo-random per-LED brightness
    uint16_t seed = (frame + led * 37 + (g_led_config[led].x + s_x_offset) * 7) & 0xFF;
    uint8_t brightness = (seed < 32) ? 255 : (seed < 64) ? 128 : 16;
    base.v = (uint8_t)((uint16_t)base.v * brightness / 255);
    return base;
}

// ---------------------------------------------------------------------------
// Reactive effects
// ---------------------------------------------------------------------------

static hsv_t effect_reactive_simple(hsv_t base, uint32_t frame, uint16_t led) {
    (void)frame;
    // Reactive effects start from a dark background — only key hits light up.
    base.v = 0;
    for (uint8_t h = 0; h < s_hits.count; h++) {
        if (s_hits.index[h] == led) {
            uint8_t fade = (uint8_t)(255 - (uint16_t)s_hits.tick[h] * 255 / s_cfg.fade);
            if (fade > base.v) base.v = fade;
        }
    }
    return base;
}

static hsv_t effect_reactive_splash(hsv_t base, uint32_t frame, uint16_t led) {
    (void)frame;
    uint16_t lx = (uint16_t)(g_led_config[led].x + s_x_offset);   // unified across the split
    uint16_t ly = g_led_config[led].y;

    // Dark background. Each hit emits an expanding HSV ring; hue shifts with
    // ring radius so the wave is visibly coloured (rainbow band moving outward)
    // instead of just a brightness pulse.
    base.v = 0;
    for (uint8_t h = 0; h < s_hits.count; h++) {
        int16_t dx = (int16_t)lx - (int16_t)s_hits.x[h];
        int16_t dy = (int16_t)ly - (int16_t)s_hits.y[h];
        uint16_t dist = (uint16_t)(abs(dx) + abs(dy));
        uint16_t wave = (uint16_t)s_hits.tick[h] * s_cfg.wave_speed;

        // Annulus: hit if we're inside the wave and not yet past its trailing edge.
        if (dist <= wave && dist + s_cfg.wave_width > wave) {
            uint8_t intensity = (uint8_t)(255 - (uint16_t)s_hits.tick[h] * 255 / s_cfg.fade);
            if (intensity > base.v) {
                base.v = intensity;
                base.h = (uint8_t)(s_hits.x[h] + (uint16_t)s_hits.tick[h] * s_cfg.hue_shift);
            }
        }
    }
    return base;
}

static hsv_t effect_reactive_multinexus(hsv_t base, uint32_t frame, uint16_t led) {
    (void)frame;
    uint16_t lx = (uint16_t)(g_led_config[led].x + s_x_offset);   // unified across the split
    uint16_t ly = g_led_config[led].y;
    uint16_t accum = 0;
    uint16_t hue_accum = 0;
    uint8_t  hue_n     = 0;

    base.v = 0;  // dark background; brightness/hue come only from active hits
    for (uint8_t h = 0; h < s_hits.count; h++) {
        int16_t dx = (int16_t)lx - (int16_t)s_hits.x[h];
        int16_t dy = (int16_t)ly - (int16_t)s_hits.y[h];
        uint16_t dist = (uint16_t)(abs(dx) + abs(dy));
        uint16_t wave = (uint16_t)s_hits.tick[h] * s_cfg.wave_speed;

        if (dist <= wave && dist + s_cfg.wave_width > wave) {
            accum     += (255 - (uint16_t)s_hits.tick[h] * 255 / s_cfg.fade);
            hue_accum += (uint16_t)(s_hits.x[h] + (uint16_t)s_hits.tick[h] * s_cfg.hue_shift);
            hue_n++;
        }
    }

    if (hue_n) {
        if (accum > 255) accum = 255;
        base.v = (uint8_t)accum;
        base.h = (uint8_t)(hue_accum / hue_n);
    }
    return base;
}

// ---------------------------------------------------------------------------
// Effect dispatch table
// ---------------------------------------------------------------------------

typedef hsv_t (*mx_effect_fn_t)(hsv_t base, uint32_t frame, uint16_t led);

static const mx_effect_fn_t s_effects[RGB_MX_COUNT] = {
    [RGB_MX_SOLID]              = effect_solid,
    [RGB_MX_BREATHING]          = effect_breathing,
    [RGB_MX_RAINBOW]            = effect_rainbow,
    [RGB_MX_RAINBOW_SWIRL]      = effect_rainbow_swirl,
    [RGB_MX_REACTIVE_SIMPLE]    = effect_reactive_simple,
    [RGB_MX_REACTIVE_SPLASH]    = effect_reactive_splash,
    [RGB_MX_REACTIVE_MULTINEXUS]= effect_reactive_multinexus,
    [RGB_MX_RAIN]               = effect_rain,
};

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

// Load the persisted RGB config from eeconfig into the live state. Also used by
// the CMK Config Protocol (Phase 15b) to apply a config written over Raw HID
// without rebooting.
void rgb_matrix_load_eeconfig(void) {
    eeconfig_rgb_t ec = eeconfig_read_rgb();
    // Validate BEFORE publishing to s_cfg: this runs in the USB task (config
    // protocol) while rgb_matrix_tick (keyscan task) may index s_effects[] with
    // s_cfg.mode at any time — an out-of-range mode must never be visible, even
    // transiently, or the tick calls a garbage function pointer.
    uint8_t mode = ec.mode;
    if (mode >= RGB_MX_COUNT) mode = RGB_MX_SOLID;

    s_cfg.enable     = ec.enable;
    s_cfg.mode       = mode;
    s_cfg.hue        = ec.hue;
    s_cfg.sat        = ec.sat;
    s_cfg.val        = ec.val;
    s_cfg.speed      = ec.speed;
    s_cfg.wave_speed = ec.wave_speed;
    s_cfg.wave_width = ec.wave_width;
    s_cfg.fade       = ec.fade;
    s_cfg.hue_shift  = ec.hue_shift;
    s_dirty = false;

    if (!s_cfg.enable) {
        rgb_driver_clear();
    }
}

void rgb_matrix_init(void) {
    rgb_matrix_load_eeconfig();

    s_last_frame  = 0;
    s_frame_count = 0;
    memset(&s_hits, 0, sizeof(s_hits));
}

// ---------------------------------------------------------------------------
// Tick — called at 1kHz from action_tick()
// ---------------------------------------------------------------------------

void rgb_matrix_tick(uint32_t now_ticks) {
    bool overlay = false;
#ifdef FN_OVERLAY_ENABLE
    overlay = fn_overlay_active();
#endif

    // Matrix dark when the RGB engine is off OR we are in low-power suspend —
    // EXCEPT while the FN dashboard is held, so the status indicators stay usable.
    // In low-power suspend we additionally cut the 5V LED boost rail (VLED_EN).
    if ((!s_cfg.enable || s_suspended) && !overlay) {
        if (!s_blanked) { rgb_driver_clear(); s_blanked = true; }
        if (s_suspended && s_vled_powered) { rgb_driver_power(false); s_vled_powered = false; }
        return;
    }

    // Coming out of dark with the rail cut (suspend exit, or FN held during sleep):
    // re-power VLED and skip one frame so the 5V rail settles before WS2812 data.
    if (!s_vled_powered) {
        rgb_driver_power(true);
        s_vled_powered = true;
        s_last_frame   = now_ticks;
        return;
    }
    s_blanked = false;

    // Frame rate limiting
    uint32_t interval = RGB_FRAME_INTERVAL_MS;

    // R5: while the FN dashboard is up, render at the FIXED base interval. The
    // speed-scaled interval (66 ms at speed=0) under-samples the 150 ms blink
    // half-period → visibly irregular status blinking (aliasing).
    if (!overlay) {
        // Velocikey: scale interval by typing speed
        if (velocikey_is_enabled()) {
            uint8_t spd = velocikey_get_speed();
            interval = interval * 256 / (spd + 256);
            if (interval < 1) interval = 1;
        }

        // Speed config scaling
        interval = interval * 256 / (s_cfg.speed + 128);
        if (interval < 1) interval = 1;
    }

    if ((now_ticks - s_last_frame) < interval) return;
    s_last_frame = now_ticks;
    s_frame_count++;

    // Critical battery: blank the whole matrix to save power. The FN overlay is
    // exempt — while FN is held the dashboard still renders below, so the user
    // can read the battery LED. rgb_driver_clear() also flushes.
    if (pm_battery_is_critical() && !overlay) {
        rgb_driver_clear();
        s_blanked = true;
        return;
    }

    // Base animation + indicator layers run only when the RGB engine is enabled.
    // When it is off we only reach here with the FN dashboard held (`overlay`),
    // which writes every LED below (status keys lit, the rest OV_BLACK).
    if (s_cfg.enable) {
        // Advance hit timers
        hits_tick();

        // Render frame
        hsv_t base = { .h = s_cfg.hue, .s = s_cfg.sat, .v = s_cfg.val };
        mx_effect_fn_t fn = s_effects[s_cfg.mode];

        for (uint16_t i = 0; i < RGB_LED_COUNT; i++) {
            hsv_t led_hsv = fn(base, s_frame_count, i);
            rgb_t led_rgb = hsv_to_rgb(led_hsv);
            rgb_driver_set_led(i, led_rgb);
        }

        // LED Indicator overlay: Caps Lock → red on indicator LEDs
        uint8_t leds = led_indicators_get_state();
        if (leds) {
            for (uint16_t i = 0; i < RGB_LED_COUNT; i++) {
                if (g_led_config[i].flags & LED_FLAG_INDICATOR) {
                    if (leds & 0x02) {  // Caps Lock (bit 1)
                        rgb_driver_set_led(i, (rgb_t){255, 0, 0});
                    } else if (leds & 0x01) {  // Num Lock (bit 0)
                        rgb_driver_set_led(i, (rgb_t){0, 0, 255});
                    } else if (leds & 0x04) {  // Scroll Lock (bit 2)
                        rgb_driver_set_led(i, (rgb_t){0, 255, 0});
                    }
                }
            }
        }
    }

#ifdef FN_OVERLAY_ENABLE
    // FN status overlay: while FN is held, replace the whole frame with the
    // status dashboard (keys without a status indicator → off).
    if (fn_overlay_active()) {
        for (uint16_t i = 0; i < RGB_LED_COUNT; i++) {
            rgb_driver_set_led(i, fn_overlay_color(g_led_config[i].row,
                                                   g_led_config[i].col, now_ticks));
        }
    }
#endif

    rgb_driver_flush();
    // Persist (s_dirty) is flushed by rgb_matrix_persist_poll() in the CM task
    // (R7) — an eeprom_write from this 1 kHz keyscan tick blocked the scan for
    // ~10 ms and burned keyscan stack on the 258-byte EEPROM tx buffer.
}

void rgb_matrix_persist_poll(void) {
    if (s_dirty) {
        rgb_matrix_persist();
    }
}

// ---------------------------------------------------------------------------
// Toggle & mode
// ---------------------------------------------------------------------------

void rgb_matrix_toggle(void) {
    s_cfg.enable = !s_cfg.enable;
    if (!s_cfg.enable) rgb_driver_clear();
    s_dirty = true;
}

void rgb_matrix_step(void) {
    s_cfg.mode = (s_cfg.mode + 1) % RGB_MX_COUNT;
    s_dirty = true;
}

void rgb_matrix_step_reverse(void) {
    s_cfg.mode = (s_cfg.mode == 0) ? (RGB_MX_COUNT - 1) : (s_cfg.mode - 1);
    s_dirty = true;
}

// ---------------------------------------------------------------------------
// HSV control
// ---------------------------------------------------------------------------

void rgb_matrix_increase_hue(void) {
    s_cfg.hue += RGB_HUE_STEP;
    s_dirty = true;
}

void rgb_matrix_decrease_hue(void) {
    s_cfg.hue -= RGB_HUE_STEP;
    s_dirty = true;
}

void rgb_matrix_increase_sat(void) {
    uint16_t sat = s_cfg.sat + RGB_SAT_STEP;
    s_cfg.sat = (sat > 255) ? 255 : (uint8_t)sat;
    s_dirty = true;
}

void rgb_matrix_decrease_sat(void) {
    s_cfg.sat = (s_cfg.sat < RGB_SAT_STEP) ? 0 : (s_cfg.sat - RGB_SAT_STEP);
    s_dirty = true;
}

void rgb_matrix_increase_val(void) {
    uint16_t val = s_cfg.val + RGB_VAL_STEP;
    s_cfg.val = (val > RGB_BRIGHTNESS_MAX) ? RGB_BRIGHTNESS_MAX : (uint8_t)val;
    s_dirty = true;
}

void rgb_matrix_decrease_val(void) {
    s_cfg.val = (s_cfg.val < RGB_VAL_STEP) ? 0 : (s_cfg.val - RGB_VAL_STEP);
    s_dirty = true;
}

// ---------------------------------------------------------------------------
// Speed control
// ---------------------------------------------------------------------------

void rgb_matrix_increase_speed(void) {
    uint16_t spd = s_cfg.speed + RGB_SPD_STEP;
    s_cfg.speed = (spd > 255) ? 255 : (uint8_t)spd;
    s_dirty = true;
}

void rgb_matrix_decrease_speed(void) {
    s_cfg.speed = (s_cfg.speed < RGB_SPD_STEP) ? 0 : (s_cfg.speed - RGB_SPD_STEP);
    s_dirty = true;
}

// ---------------------------------------------------------------------------
// 8-step rings (one keycode cycles through preset values)
// ---------------------------------------------------------------------------

// Pick the next preset >= current; wrap to step[0] after the largest. This
// makes the ring stable even if the persisted value doesn't match a step
// exactly (the user may have used the legacy ± keycodes before).
static uint8_t ring_next(const uint8_t *steps, uint8_t n, uint8_t cur) {
    for (uint8_t i = 0; i < n; i++) {
        if (steps[i] > cur) return steps[i];
    }
    return steps[0];
}

void rgb_matrix_cycle_val(void) {
    static const uint8_t steps[] = { 16, 32, 64, 96, 128, 160, 192, 255 };
    s_cfg.val = ring_next(steps, 8, s_cfg.val);
    s_dirty = true;
}

void rgb_matrix_cycle_hue(void) {
    static const uint8_t steps[] = { 0, 32, 64, 96, 128, 160, 192, 224 };
    s_cfg.hue = ring_next(steps, 8, s_cfg.hue);
    s_dirty = true;
}

void rgb_matrix_cycle_sat(void) {
    static const uint8_t steps[] = { 0, 36, 72, 108, 144, 180, 216, 255 };
    s_cfg.sat = ring_next(steps, 8, s_cfg.sat);
    s_dirty = true;
}

void rgb_matrix_cycle_speed(void) {
    static const uint8_t steps[] = { 32, 64, 96, 128, 160, 192, 224, 255 };
    s_cfg.speed = ring_next(steps, 8, s_cfg.speed);
    s_dirty = true;
}

void rgb_matrix_cycle_wave_speed(void) {
    static const uint8_t steps[] = { 2, 3, 4, 6, 8, 12, 16, 24 };
    s_cfg.wave_speed = ring_next(steps, 8, s_cfg.wave_speed);
    s_dirty = true;
}

void rgb_matrix_cycle_wave_width(void) {
    static const uint8_t steps[] = { 8, 14, 20, 28, 40, 56, 80, 120 };
    s_cfg.wave_width = ring_next(steps, 8, s_cfg.wave_width);
    s_dirty = true;
}

void rgb_matrix_cycle_fade(void) {
    static const uint8_t steps[] = { 12, 20, 30, 40, 60, 90, 130, 200 };
    s_cfg.fade = ring_next(steps, 8, s_cfg.fade);
    s_dirty = true;
}

void rgb_matrix_cycle_hue_shift(void) {
    static const uint8_t steps[] = { 0, 1, 2, 4, 6, 10, 16, 24 };
    s_cfg.hue_shift = ring_next(steps, 8, s_cfg.hue_shift);
    s_dirty = true;
}

// ---------------------------------------------------------------------------
// Getters
// ---------------------------------------------------------------------------

bool rgb_matrix_is_enabled(void) {
    return s_cfg.enable;
}

uint8_t rgb_matrix_get_mode(void) {
    return s_cfg.mode;
}

// ---------------------------------------------------------------------------
// Cross-half RGB sync (Phase 24)
// ---------------------------------------------------------------------------

void rgb_matrix_build_sync_state(uint8_t *frame) {
    frame[0]  = CMK_RGB_SYNC_TAG_STATE;
    frame[1]  = s_cfg.enable;
    frame[2]  = s_cfg.mode;
    frame[3]  = s_cfg.hue;
    frame[4]  = s_cfg.sat;
    frame[5]  = s_cfg.val;
    frame[6]  = s_cfg.speed;
    frame[7]  = s_cfg.wave_speed;
    frame[8]  = s_cfg.wave_width;
    frame[9]  = s_cfg.fade;
    frame[10] = s_cfg.hue_shift;
    uint16_t phase = (uint16_t)s_frame_count;   // low 16 bits — effects use it mod 256
    frame[11] = (uint8_t)(phase & 0xFF);
    frame[12] = (uint8_t)(phase >> 8);
}

void rgb_matrix_apply_sync_state(const uint8_t *frame, uint8_t len) {
    if (len < CMK_RGB_SYNC_STATE_LEN || frame[0] != CMK_RGB_SYNC_TAG_STATE) return;
    // Validate mode BEFORE publishing (the keyscan tick indexes s_effects[] with it).
    uint8_t mode = frame[2];
    if (mode >= RGB_MX_COUNT) mode = RGB_MX_SOLID;
    s_cfg.enable     = frame[1];
    s_cfg.mode       = mode;
    s_cfg.hue        = frame[3];
    s_cfg.sat        = frame[4];
    s_cfg.val        = frame[5];
    s_cfg.speed      = frame[6];
    s_cfg.wave_speed = frame[7];
    s_cfg.wave_width = frame[8];
    s_cfg.fade       = frame[9];
    s_cfg.hue_shift  = frame[10];
    // Phase-align to the master so animations stay in step. A 32-bit write is
    // atomic on RP2350; effects only use the low bits, so the jump is seamless.
    s_frame_count = (uint32_t)(frame[11] | ((uint16_t)frame[12] << 8));
    // Follower state — deliberately NOT marked dirty: the slave never persists the
    // master's config; rgb_matrix_load_eeconfig() restores its own on disengage.
    // Bug #13: the FN dashboard bypasses enable (see rgb_matrix_tick). Clearing
    // here while it is up blanked it on every 250 ms resync → visible blinking.
    // On FN release the tick's dark branch clears the strip (s_blanked is false).
    bool overlay = false;
#ifdef FN_OVERLAY_ENABLE
    overlay = fn_overlay_active();
#endif
    if (!s_cfg.enable && !overlay) rgb_driver_clear();
}

void rgb_matrix_set_split_engaged(bool engaged) {
    s_x_offset = (engaged && board_side_get() == CMK_SIDE_RIGHT)
                     ? RGB_MATRIX_SPLIT_X_OFFSET : 0;
}

#endif // RGB_MATRIX_ENABLE
