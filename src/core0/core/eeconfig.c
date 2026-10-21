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
// eeconfig.c — Persistent settings via EEPROM (Phase 11)
//
// EEPROM region: 0x0100-0x011F (32 bytes) on M24M01-R.
// Pattern identical to bonding_store.c: RAM cache + flush on write.
//
// Thread safety: writes are rare and short (single task context in practice).
//

#include "eeconfig.h"
#include "eeprom.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Record layout
// ---------------------------------------------------------------------------

#define EECONFIG_MAGIC_VAL  0xEC01u
#define EECONFIG_VERSION    8
#define EECONFIG_ADDR       0x0100u

typedef struct __attribute__((packed)) {
    uint16_t magic;           // EECONFIG_MAGIC_VAL
    uint8_t  version;         // EECONFIG_VERSION
    uint8_t  default_layer;
    uint16_t keymap_flags;    // magic swap bitmask
    uint8_t  auto_shift_on;
    uint8_t  unicode_mode;
    // Bug #8: tapping term runtime (ms; 0 = factory default TAPPING_TERM),
    // ricavato dall'ex blob `keyboard` (mai scritto → 0 su tutti i record v8:
    // stesso layout/offset, niente version bump).
    uint16_t tapping_term;
    uint8_t  _reserved2[2];   // ex "keyboard" blob, byte alti
    uint32_t user;            // user-specific blob
    // Phase 13: RGB base config (6 bytes)
    uint8_t  rgb_enable;
    uint8_t  rgb_mode;
    uint8_t  rgb_hue;
    uint8_t  rgb_sat;
    uint8_t  rgb_val;
    uint8_t  rgb_speed;
    // Phase 13: reactive-effect tunables (4 bytes)
    uint8_t  rgb_wave_speed;
    uint8_t  rgb_wave_width;
    uint8_t  rgb_fade;
    uint8_t  rgb_hue_shift;
    // Phase 14 ext: HID output destination selector
    uint8_t  output_target;   // output_target_t: 0=USB, 1=BLE (factory default = USB)
    // Phase 20b: split config mode (0=two independent halves, 1=unified split view)
    uint8_t  split_mode;      // factory default = 0 (off)
    // v8 (bug #2): Low Power Mode idle timeouts in SECONDS, 0 = tier disabled.
    // Runtime source for low_power.c; factory defaults from LP_*_IDLE_MS.
    // The DEEP value is also the window power_veto()/power_request() use, so it
    // doubles as the relayed-HID hold that keeps the radio up (bug #1).
    uint16_t lp_light_s;
    uint16_t lp_deep_s;       // (total = 32 bytes)
} eeconfig_t;

_Static_assert(sizeof(eeconfig_t) == 32, "eeconfig_t must be 32 bytes");

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static eeconfig_t s_cache;
static bool       s_eeprom_ok;   // true if EEPROM was detected by bonding_store

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// We don't call eeprom_init() here — bonding_store_init() already did.
// Instead we check if the EEPROM is reachable by attempting a read and
// verifying magic. If bonding_store didn't find EEPROM, we still work
// in RAM-only mode (defaults applied, not persisted).

static void cache_defaults(void) {
    memset(&s_cache, 0, sizeof(s_cache));
    s_cache.magic     = EECONFIG_MAGIC_VAL;
    s_cache.version   = EECONFIG_VERSION;
    // RGB base defaults (Phase 13)
    s_cache.rgb_enable     = RGB_DEFAULT_ENABLE;
    s_cache.rgb_mode       = RGB_DEFAULT_MODE;
    s_cache.rgb_hue        = RGB_DEFAULT_HUE;
    s_cache.rgb_sat        = RGB_DEFAULT_SAT;
    s_cache.rgb_val        = RGB_DEFAULT_VAL;
    s_cache.rgb_speed      = RGB_DEFAULT_SPEED;
    // Reactive-effect defaults
    s_cache.rgb_wave_speed = RGB_DEFAULT_WAVE_SPEED;
    s_cache.rgb_wave_width = RGB_DEFAULT_WAVE_WIDTH;
    s_cache.rgb_fade       = RGB_DEFAULT_FADE;
    s_cache.rgb_hue_shift  = RGB_DEFAULT_HUE_SHIFT;
    // Low Power timeouts (v8) — seconds
    s_cache.lp_light_s     = (uint16_t)(LP_LIGHT_IDLE_MS / 1000u);
    s_cache.lp_deep_s      = (uint16_t)(LP_DEEP_IDLE_MS / 1000u);
}

static void store_flush(void) {
    if (!s_eeprom_ok) return;
    eeprom_write(EECONFIG_ADDR, (const uint8_t *)&s_cache, sizeof(s_cache));
    DBG_MEM("[EECONFIG] flushed to EEPROM\n");
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void eeconfig_init(void) {
    // Try reading from EEPROM. R4: retried + explicit error path — before, a bus
    // glitch returned garbage that looked like a magic mismatch and the settings
    // were FACTORY-RESET (re-written over the still-valid record on EEPROM).
    eeconfig_t tmp;
    if (!eeprom_read_retry(EECONFIG_ADDR, (uint8_t *)&tmp, sizeof(tmp), 3)) {
        printf("[EECONFIG] ERRORE lettura EEPROM — default in RAM, persist disabilitato\n");
        cache_defaults();
        s_eeprom_ok = false;   // skip flushes: never overwrite a possibly-valid record
        return;
    }

    if (tmp.magic == EECONFIG_MAGIC_VAL && tmp.version == EECONFIG_VERSION) {
        // Valid record found — current version
        s_cache = tmp;
        s_eeprom_ok = true;
        DBG_MEM("[EECONFIG] loaded from EEPROM (default_layer=%d, keymap_flags=0x%04x)\n",
                s_cache.default_layer, s_cache.keymap_flags);
    } else if (tmp.magic == EECONFIG_MAGIC_VAL && tmp.version >= 1 && tmp.version < EECONFIG_VERSION) {
        // Migration any-older-version → current: re-apply RGB defaults from
        // cmk_config.h, keep default_layer and keymap_flags from the
        // existing record.
        s_cache = tmp;
        s_cache.version        = EECONFIG_VERSION;
        s_cache.rgb_enable     = RGB_DEFAULT_ENABLE;
        s_cache.rgb_mode       = RGB_DEFAULT_MODE;
        s_cache.rgb_hue        = RGB_DEFAULT_HUE;
        s_cache.rgb_sat        = RGB_DEFAULT_SAT;
        s_cache.rgb_val        = RGB_DEFAULT_VAL;
        s_cache.rgb_speed      = RGB_DEFAULT_SPEED;
        s_cache.rgb_wave_speed = RGB_DEFAULT_WAVE_SPEED;
        s_cache.rgb_wave_width = RGB_DEFAULT_WAVE_WIDTH;
        s_cache.rgb_fade       = RGB_DEFAULT_FADE;
        s_cache.rgb_hue_shift  = RGB_DEFAULT_HUE_SHIFT;
        // Phase 14 ext: default output destination = USB (0)
        s_cache.output_target  = 0;
        // v8 (bug #2): these bytes were `_reserved` and read back as 0, which
        // here means "tier disabled" — seed them with the factory defaults
        // instead, or a migrated keyboard would silently lose LIGHT and DEEP.
        s_cache.lp_light_s     = (uint16_t)(LP_LIGHT_IDLE_MS / 1000u);
        s_cache.lp_deep_s      = (uint16_t)(LP_DEEP_IDLE_MS / 1000u);
        s_eeprom_ok = true;
        store_flush();
        DBG_MEM("[EECONFIG] migrated v%u → v%u (re-applied RGB defaults)\n",
                tmp.version, EECONFIG_VERSION);
    } else {
        // Invalid or empty — factory reset
        DBG_MEM("[EECONFIG] magic mismatch (got 0x%04x) — factory reset\n", tmp.magic);
        cache_defaults();
        // Try writing defaults — if EEPROM is absent, eeprom_write is a no-op
        // (bonding_store_init already probed). We set s_eeprom_ok optimistically
        // and rely on eeprom_write being safe when absent.
        s_eeprom_ok = true;
        store_flush();
    }
}

void eeconfig_factory_reset(void) {
    cache_defaults();
    store_flush();
    DBG_MEM("[EECONFIG] factory reset\n");
}

bool eeconfig_is_enabled(void) {
    return s_cache.magic == EECONFIG_MAGIC_VAL;
}

// ---------------------------------------------------------------------------
// Default layer
// ---------------------------------------------------------------------------

uint8_t eeconfig_read_default_layer(void) {
    return s_cache.default_layer;
}

void eeconfig_update_default_layer(uint8_t layer) {
    if (s_cache.default_layer == layer) return;
    s_cache.default_layer = layer;
    store_flush();
}

// ---------------------------------------------------------------------------
// Keymap flags
// ---------------------------------------------------------------------------

uint16_t eeconfig_read_keymap_flags(void) {
    return s_cache.keymap_flags;
}

void eeconfig_update_keymap_flags(uint16_t flags) {
    if (s_cache.keymap_flags == flags) return;
    s_cache.keymap_flags = flags;
    store_flush();
}

// ---------------------------------------------------------------------------
// Auto Shift
// ---------------------------------------------------------------------------

uint8_t eeconfig_read_auto_shift(void) {
    return s_cache.auto_shift_on;
}

void eeconfig_update_auto_shift(uint8_t on) {
    if (s_cache.auto_shift_on == on) return;
    s_cache.auto_shift_on = on;
    store_flush();
}

// ---------------------------------------------------------------------------
// Unicode mode
// ---------------------------------------------------------------------------

uint8_t eeconfig_read_unicode_mode(void) {
    return s_cache.unicode_mode;
}

void eeconfig_update_unicode_mode(uint8_t mode) {
    if (s_cache.unicode_mode == mode) return;
    s_cache.unicode_mode = mode;
    store_flush();
}

// ---------------------------------------------------------------------------
// Tapping term (bug #8) — ms, 0 = factory default (TAPPING_TERM)
// ---------------------------------------------------------------------------

uint16_t eeconfig_read_tapping_term(void) {
    return s_cache.tapping_term;
}

void eeconfig_update_tapping_term(uint16_t ms) {
    if (s_cache.tapping_term == ms) return;
    s_cache.tapping_term = ms;
    store_flush();
}

// ---------------------------------------------------------------------------
// User blob
// ---------------------------------------------------------------------------

uint32_t eeconfig_read_user(void) {
    return s_cache.user;
}

void eeconfig_update_user(uint32_t val) {
    if (s_cache.user == val) return;
    s_cache.user = val;
    store_flush();
}

// ---------------------------------------------------------------------------
// HID output destination (Phase 14 ext)
// ---------------------------------------------------------------------------

uint8_t eeconfig_read_output_target(void) {
    return s_cache.output_target;
}

void eeconfig_update_output_target(uint8_t target) {
    if (s_cache.output_target == target) return;
    s_cache.output_target = target;
    store_flush();
}

// ---------------------------------------------------------------------------
// Split config mode (Phase 20b)
// ---------------------------------------------------------------------------

uint8_t eeconfig_read_split_mode(void) {
    return s_cache.split_mode;
}

void eeconfig_update_split_mode(uint8_t on) {
    on = on ? 1 : 0;
    if (s_cache.split_mode == on) return;
    s_cache.split_mode = on;
    store_flush();
}

// ---------------------------------------------------------------------------
// Low Power timeouts (v8, bug #2) — seconds, 0 = tier disabled
// ---------------------------------------------------------------------------

uint16_t eeconfig_read_lp_light_s(void) {
    return s_cache.lp_light_s;
}

void eeconfig_update_lp_light_s(uint16_t seconds) {
    if (s_cache.lp_light_s == seconds) return;
    s_cache.lp_light_s = seconds;
    store_flush();
}

uint16_t eeconfig_read_lp_deep_s(void) {
    return s_cache.lp_deep_s;
}

void eeconfig_update_lp_deep_s(uint16_t seconds) {
    if (s_cache.lp_deep_s == seconds) return;
    s_cache.lp_deep_s = seconds;
    store_flush();
}

// ---------------------------------------------------------------------------
// RGB config (Phase 13)
// ---------------------------------------------------------------------------

eeconfig_rgb_t eeconfig_read_rgb(void) {
    return (eeconfig_rgb_t){
        .enable     = s_cache.rgb_enable,
        .mode       = s_cache.rgb_mode,
        .hue        = s_cache.rgb_hue,
        .sat        = s_cache.rgb_sat,
        .val        = s_cache.rgb_val,
        .speed      = s_cache.rgb_speed,
        .wave_speed = s_cache.rgb_wave_speed,
        .wave_width = s_cache.rgb_wave_width,
        .fade       = s_cache.rgb_fade,
        .hue_shift  = s_cache.rgb_hue_shift,
    };
}

void eeconfig_update_rgb(const eeconfig_rgb_t *cfg) {
    if (s_cache.rgb_enable     == cfg->enable     &&
        s_cache.rgb_mode       == cfg->mode       &&
        s_cache.rgb_hue        == cfg->hue        &&
        s_cache.rgb_sat        == cfg->sat        &&
        s_cache.rgb_val        == cfg->val        &&
        s_cache.rgb_speed      == cfg->speed      &&
        s_cache.rgb_wave_speed == cfg->wave_speed &&
        s_cache.rgb_wave_width == cfg->wave_width &&
        s_cache.rgb_fade       == cfg->fade       &&
        s_cache.rgb_hue_shift  == cfg->hue_shift) return;
    s_cache.rgb_enable     = cfg->enable;
    s_cache.rgb_mode       = cfg->mode;
    s_cache.rgb_hue        = cfg->hue;
    s_cache.rgb_sat        = cfg->sat;
    s_cache.rgb_val        = cfg->val;
    s_cache.rgb_speed      = cfg->speed;
    s_cache.rgb_wave_speed = cfg->wave_speed;
    s_cache.rgb_wave_width = cfg->wave_width;
    s_cache.rgb_fade       = cfg->fade;
    s_cache.rgb_hue_shift  = cfg->hue_shift;
    store_flush();
}

// ---------------------------------------------------------------------------
// Whole-record blob access (Phase 20 — inter-half config transfer)
// ---------------------------------------------------------------------------

void eeconfig_export(uint8_t *out32) {
    memcpy(out32, &s_cache, sizeof(s_cache));
}

bool eeconfig_import(const uint8_t *in32) {
    eeconfig_t tmp;
    memcpy(&tmp, in32, sizeof(tmp));
    if (tmp.magic != EECONFIG_MAGIC_VAL || tmp.version != EECONFIG_VERSION) {
        return false;
    }
    s_cache = tmp;
    store_flush();
    return true;
}
