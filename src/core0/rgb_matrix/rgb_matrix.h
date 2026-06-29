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
// rgb_matrix.h — Per-key RGB animation engine (Phase 13b)
//
// Each LED maps to a key matrix position + physical coordinate.
// Supports positional and reactive (keypress) effects.
// Mutually exclusive with RGBLIGHT_ENABLE and LED_MATRIX_ENABLE.
//

#ifndef CMK_RGB_MATRIX_H
#define CMK_RGB_MATRIX_H

#include "config/cmk_config.h"

#ifdef RGB_MATRIX_ENABLE

#include "rgb_driver.h"
#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// LED flags (bitmask)
// ---------------------------------------------------------------------------

#define LED_FLAG_NONE       0x00
#define LED_FLAG_KEYLIGHT   0x01   // LED under a key
#define LED_FLAG_UNDERGLOW  0x02   // underglow LED (not associated with a key)
#define LED_FLAG_INDICATOR  0x04   // indicator LED (Caps Lock, etc.)
#define LED_FLAG_ALL        0xFF

// ---------------------------------------------------------------------------
// LED config entry — one per LED in the chain
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t row;    // matrix row (0xFF = no key)
    uint8_t col;    // matrix col (0xFF = no key)
    uint8_t x;      // physical X (0-224)
    uint8_t y;      // physical Y (0-64)
    uint8_t flags;  // LED_FLAG_*
} led_config_entry_t;

// Per-side LED map; the pointer is set at boot by keymap_init() to the layout
// matching the detected board side (see core/keymap.c). Indexing the pointer is
// identical to indexing the old array, so consumers are unchanged.
extern const led_config_entry_t *g_led_config;

// ---------------------------------------------------------------------------
// Reactive hit tracking
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  count;
    uint8_t  index[RGB_MATRIX_HIT_MAX];  // LED index (0xFF = remote key, no local LED)
    uint16_t x[RGB_MATRIX_HIT_MAX];      // UNIFIED physical X (spans both halves)
    uint16_t y[RGB_MATRIX_HIT_MAX];
    uint16_t tick[RGB_MATRIX_HIT_MAX];   // frames since hit
} rgb_matrix_hit_t;

// ---------------------------------------------------------------------------
// Effect modes
// ---------------------------------------------------------------------------

typedef enum {
    RGB_MX_SOLID = 0,
    RGB_MX_BREATHING,
    RGB_MX_RAINBOW,
    RGB_MX_RAINBOW_SWIRL,
    RGB_MX_REACTIVE_SIMPLE,     // pressed key lights up, fades
    RGB_MX_REACTIVE_SPLASH,     // pressed key radiates outward
    RGB_MX_REACTIVE_MULTINEXUS, // multiple splash overlapping
    RGB_MX_RAIN,                // random drops falling
    RGB_MX_COUNT
} rgb_matrix_effect_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

void    rgb_matrix_init(void);
void    rgb_matrix_tick(uint32_t now_ticks);

// R7: flush a pending eeconfig persist (s_dirty). Called from the CM task —
// EEPROM writes must not run in the 1 kHz keyscan tick (latency + stack).
void    rgb_matrix_persist_poll(void);

// Reload live state from the persisted eeconfig RGB block (CMK Config Protocol).
void    rgb_matrix_load_eeconfig(void);

// Called from action_exec() on keypress for reactive effects
void    rgb_matrix_record_hit(uint8_t row, uint8_t col);

// Phase 24b — register a reactive hit at an arbitrary UNIFIED position (a key on
// the OTHER half, which has no local LED). Splash/multinexus radiate from it onto
// the local LEDs. index is marked 0xFF (no exact LED to light for REACTIVE_SIMPLE).
void    rgb_matrix_record_hit_xy(uint16_t ux, uint16_t uy);

// Low-power: suspend rendering and cut the 5V LED rail (VLED_EN) while idle.
// The FN dashboard overlay still renders when held. Called by the low_power module.
void    rgb_matrix_set_suspended(bool suspended);

// Toggle & mode (shared interface with rgblight for process_rgb.c)
void    rgb_matrix_toggle(void);
void    rgb_matrix_step(void);
void    rgb_matrix_step_reverse(void);

// HSV control
void    rgb_matrix_increase_hue(void);
void    rgb_matrix_decrease_hue(void);
void    rgb_matrix_increase_sat(void);
void    rgb_matrix_decrease_sat(void);
void    rgb_matrix_increase_val(void);
void    rgb_matrix_decrease_val(void);

// Speed control
void    rgb_matrix_increase_speed(void);
void    rgb_matrix_decrease_speed(void);

// 8-step rings (one keycode cycles through preset values)
void    rgb_matrix_cycle_val(void);
void    rgb_matrix_cycle_hue(void);
void    rgb_matrix_cycle_sat(void);
void    rgb_matrix_cycle_speed(void);
void    rgb_matrix_cycle_wave_speed(void);
void    rgb_matrix_cycle_wave_width(void);
void    rgb_matrix_cycle_fade(void);
void    rgb_matrix_cycle_hue_shift(void);

// Getters
bool    rgb_matrix_is_enabled(void);
uint8_t rgb_matrix_get_mode(void);

// ---------------------------------------------------------------------------
// Cross-half RGB sync (Phase 24) — MASTER pushes effect state to the SLAVE so
// both halves render one animation in phase. State only (no per-LED frames).
// ---------------------------------------------------------------------------

// MASTER: serialize the live effect state into a CMK_RGB_SYNC_STATE_LEN frame
// (tag + config + 16-bit animation phase). Called from the CM task.
void    rgb_matrix_build_sync_state(uint8_t *frame);

// SLAVE: apply a received state frame to the live config WITHOUT persisting
// (follower state — restored from eeconfig on disengage). Aligns the animation
// phase to the master's. Called from the CM task.
void    rgb_matrix_apply_sync_state(const uint8_t *frame, uint8_t len);

// Engage/disengage the unified-X offset for positional effects (RIGHT half only).
// Called on split role transitions; pass false to fall back to local coordinates.
void    rgb_matrix_set_split_engaged(bool engaged);

#endif // RGB_MATRIX_ENABLE

#endif // CMK_RGB_MATRIX_H
