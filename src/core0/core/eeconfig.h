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
// eeconfig.h — Persistent settings via EEPROM (Phase 11)
//
// EEPROM region: 0x0100-0x011F (32 bytes) on M24M01-R.
// Pattern: RAM cache + flush to EEPROM on update.
//
// Call eeconfig_init() from main() after bonding_store_init().
//

#ifndef CMK_EECONFIG_H
#define CMK_EECONFIG_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Init — call from main() before vTaskStartScheduler()
// ---------------------------------------------------------------------------

// Read eeconfig from EEPROM into RAM cache.
// If magic doesn't match → factory reset that region.
void eeconfig_init(void);

// Reset eeconfig region to defaults and flush to EEPROM.
void eeconfig_factory_reset(void);

// Returns true if eeconfig was loaded successfully (valid magic).
bool eeconfig_is_enabled(void);

// ---------------------------------------------------------------------------
// Default layer
// ---------------------------------------------------------------------------

uint8_t  eeconfig_read_default_layer(void);
void     eeconfig_update_default_layer(uint8_t layer);

// ---------------------------------------------------------------------------
// Keymap flags (magic swap settings — 16-bit bitmask)
//   bit 0: swap_ctl_caps
//   bit 1: swap_esc_caps
//   bit 2: caps_as_ctl
//   bit 3: swap_lalt_lgui
//   bit 4: swap_ralt_rgui
//   bit 5: gui_disabled
// ---------------------------------------------------------------------------

uint16_t eeconfig_read_keymap_flags(void);
void     eeconfig_update_keymap_flags(uint16_t flags);

// ---------------------------------------------------------------------------
// Auto Shift enabled state
// ---------------------------------------------------------------------------

uint8_t  eeconfig_read_auto_shift(void);
void     eeconfig_update_auto_shift(uint8_t on);

// ---------------------------------------------------------------------------
// Unicode mode (Phase 12)
// ---------------------------------------------------------------------------

uint8_t  eeconfig_read_unicode_mode(void);
void     eeconfig_update_unicode_mode(uint8_t mode);

// ---------------------------------------------------------------------------
// HID output destination (Phase 14 ext) — 0=USB, 1=BLE (see output_target_t)
// ---------------------------------------------------------------------------

uint8_t  eeconfig_read_output_target(void);
void     eeconfig_update_output_target(uint8_t target);

// ---------------------------------------------------------------------------
// Split config mode (Phase 20b) — 0 = two independent halves, 1 = unified
// split view (config protocol exposes 4×12 when the peer is joined over I2C1)
// ---------------------------------------------------------------------------

uint8_t  eeconfig_read_split_mode(void);
void     eeconfig_update_split_mode(uint8_t on);

// ---------------------------------------------------------------------------
// Low Power Mode idle timeouts (v8, bug #2) — SECONDS, 0 = tier disabled.
// Runtime source for low_power.c (cached at init; low_power_refresh_config()
// re-reads them so a SET_SETTING applies without a reboot).
// ---------------------------------------------------------------------------

uint16_t eeconfig_read_lp_light_s(void);
void     eeconfig_update_lp_light_s(uint16_t seconds);

uint16_t eeconfig_read_lp_deep_s(void);
void     eeconfig_update_lp_deep_s(uint16_t seconds);

// ---------------------------------------------------------------------------
// Tapping term (bug #8) — ms, 0 = factory default (TAPPING_TERM). Consumato
// via tapping_term_get() (action_tapping.c, che applica default+clamp).
// Ricavato dall'ex blob `keyboard` (stesso offset, nessun version bump).
// ---------------------------------------------------------------------------

uint16_t eeconfig_read_tapping_term(void);
void     eeconfig_update_tapping_term(uint16_t ms);

// ---------------------------------------------------------------------------
// User blob (32-bit — for custom storage)
// ---------------------------------------------------------------------------

uint32_t eeconfig_read_user(void);
void     eeconfig_update_user(uint32_t val);

// ---------------------------------------------------------------------------
// RGB config (Phase 13) — 10 bytes packed
// ---------------------------------------------------------------------------

typedef struct __attribute__((packed)) {
    uint8_t enable;          // 0/1
    uint8_t mode;            // rgb_effect_t
    uint8_t hue;             // 0-255
    uint8_t sat;             // 0-255
    uint8_t val;             // 0-255 (brightness)
    uint8_t speed;           // 0-255 (animation frame rate scaling)

    // Reactive-effect parameters (REACTIVE_SIMPLE/SPLASH/MULTINEXUS)
    uint8_t wave_speed;      // wave-front propagation (multiplier per tick)
    uint8_t wave_width;      // ring width in pixels
    uint8_t fade;            // frames until hit fully fades
    uint8_t hue_shift;       // hue change per tick along the wave
} eeconfig_rgb_t;

eeconfig_rgb_t eeconfig_read_rgb(void);
void           eeconfig_update_rgb(const eeconfig_rgb_t *cfg);

// ---------------------------------------------------------------------------
// Whole-record blob access (32 bytes) — used by the inter-half config transfer
// (Phase 20). export copies the live cache out; import validates the incoming
// record's magic + version, replaces the cache and flushes once. import
// returns false (and changes nothing) on a magic/version mismatch.
// ---------------------------------------------------------------------------

void eeconfig_export(uint8_t *out32);
bool eeconfig_import(const uint8_t *in32);

#endif // CMK_EECONFIG_H
