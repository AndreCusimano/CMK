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
// keycode.h — 16-bit keycode definitions (Phase 7)
//
// Encoding ranges:
//   Basic HID    0x0000-0x00FF   Direct USB HID usage codes
//   Mods+Key     0x0100-0x1FFF   Modifier(s) applied to a basic keycode
//   Mod-Tap      0x2000-0x3FFF   Hold = modifier, Tap = basic keycode
//   Layer-Tap    0x4000-0x4FFF   Hold = MO(layer), Tap = basic keycode
//   Layer key    0x5100-0x55FF   MO / TG / TO / DF / OSL
//   Consumer     0x7100-0x71FF   Media/volume/brightness — Consumer Page report (ID 4)
//   Macro        0x7700-0x770F   MACRO(n) — EEPROM-stored macros (QMK QK_MACRO base, Phase 15e)
//   CMK custom   0x7C00-0x7FFF   Firmware-specific (BLE slots, reset, RGB, etc.)
//

#ifndef CMK_KEYCODE_H
#define CMK_KEYCODE_H

#include <stdint.h>
#include <Scan-Codes.h>

// ---------------------------------------------------------------------------
// Special keycodes
// ---------------------------------------------------------------------------

#define KC_NO       0x0000   // No key — blocked position
#define KC_TRNS     0x0001   // Transparent — fall through to lower layer

// ---------------------------------------------------------------------------
// Basic HID keycodes — aliases for Scan-Codes.h KEY_xxx defines
// ---------------------------------------------------------------------------
// The Scan-Codes.h header defines KEY_A (0x04), KEY_Q (0x14), KEY_SPACE (0x2C),
// KEY_LEFTCTRL (0xE0), etc. — all standard USB HID Usage Table values.
// These fit in the 0x0000-0x00FF basic range and can be used directly in keymaps.
//
// Convenience aliases (QMK-style KC_xxx):

#define KC_A    KEY_A
#define KC_B    KEY_B
#define KC_C    KEY_C
#define KC_D    KEY_D
#define KC_E    KEY_E
#define KC_F    KEY_F
#define KC_G    KEY_G
#define KC_H    KEY_H
#define KC_I    KEY_I
#define KC_J    KEY_J
#define KC_K    KEY_K
#define KC_L    KEY_L
#define KC_M    KEY_M
#define KC_N    KEY_N
#define KC_O    KEY_O
#define KC_P    KEY_P
#define KC_Q    KEY_Q
#define KC_R    KEY_R
#define KC_S    KEY_S
#define KC_T    KEY_T
#define KC_U    KEY_U
#define KC_V    KEY_V
#define KC_W    KEY_W
#define KC_X    KEY_X
#define KC_Y    KEY_Y
#define KC_Z    KEY_Z

#define KC_1    KEY_1
#define KC_2    KEY_2
#define KC_3    KEY_3
#define KC_4    KEY_4
#define KC_5    KEY_5
#define KC_6    KEY_6
#define KC_7    KEY_7
#define KC_8    KEY_8
#define KC_9    KEY_9
#define KC_0    KEY_0

#define KC_ENT   KEY_ENTER
#define KC_ESC   KEY_ESC
#define KC_BSPC  KEY_BACKSPACE
#define KC_TAB   KEY_TAB
#define KC_SPC   KEY_SPACE
#define KC_MINS  KEY_MINUS
#define KC_EQL   KEY_EQUAL
#define KC_LBRC  KEY_LEFTBRACE
#define KC_RBRC  KEY_RIGHTBRACE
#define KC_BSLS  KEY_BACKSLASH
#define KC_NUHS  KEY_HASHTILDE    // Non-US #  (\ on some ISO layouts)
#define KC_NUBS  KEY_102ND        // Non-US \  (ISO key left of Z: < > on IT/UK)
#define KC_SCLN  KEY_SEMICOLON
#define KC_QUOT  KEY_APOSTROPHE
#define KC_GRV   KEY_GRAVE
#define KC_COMM  KEY_COMMA
#define KC_DOT   KEY_DOT
#define KC_SLSH  KEY_SLASH
#define KC_CAPS  KEY_CAPSLOCK

#define KC_F1   KEY_F1
#define KC_F2   KEY_F2
#define KC_F3   KEY_F3
#define KC_F4   KEY_F4
#define KC_F5   KEY_F5
#define KC_F6   KEY_F6
#define KC_F7   KEY_F7
#define KC_F8   KEY_F8
#define KC_F9   KEY_F9
#define KC_F10  KEY_F10
#define KC_F11  KEY_F11
#define KC_F12  KEY_F12

#define KC_INS   KEY_INSERT
#define KC_HOME  KEY_HOME
#define KC_PGUP  KEY_PAGEUP
#define KC_DEL   KEY_DELETE
#define KC_END   KEY_END
#define KC_PGDN  KEY_PAGEDOWN
#define KC_RGHT  KEY_RIGHT
#define KC_LEFT  KEY_LEFT
#define KC_DOWN  KEY_DOWN
#define KC_UP    KEY_UP

// Modifier keys (also basic HID keycodes 0xE0-0xE7)
#define KC_LCTL  KEY_LEFTCTRL
#define KC_LSFT  KEY_LEFTSHIFT
#define KC_LALT  KEY_LEFTALT
#define KC_LGUI  KEY_LEFTMETA
#define KC_RCTL  KEY_RIGHTCTRL
#define KC_RSFT  KEY_RIGHTSHIFT
#define KC_RALT  KEY_RIGHTALT
#define KC_RGUI  KEY_RIGHTMETA

// Numpad keys (for Unicode Windows mode)
#define KC_KP_PLUS  KEY_KPPLUS
#define KC_KP_0     KEY_KP0
#define KC_KP_1     KEY_KP1
#define KC_KP_2     KEY_KP2
#define KC_KP_3     KEY_KP3
#define KC_KP_4     KEY_KP4
#define KC_KP_5     KEY_KP5
#define KC_KP_6     KEY_KP6
#define KC_KP_7     KEY_KP7
#define KC_KP_8     KEY_KP8
#define KC_KP_9     KEY_KP9

// ---------------------------------------------------------------------------
// 5-bit modifier encoding (for Mod-Tap / Mods+Key packed keycodes)
// ---------------------------------------------------------------------------
// Bits [3:0] = CTRL/SHIFT/ALT/GUI, Bit [4] = Right flag
// These are NOT the HID report byte — use MOD5_TO_HID() to convert.

#define MOD_LCTL  0x01
#define MOD_LSFT  0x02
#define MOD_LALT  0x04
#define MOD_LGUI  0x08
#define MOD_RCTL  0x11
#define MOD_RSFT  0x12
#define MOD_RALT  0x14
#define MOD_RGUI  0x18

// Convert 5-bit packed mod to 8-bit HID modifier byte
static inline uint8_t mod5_to_hid(uint8_t mod5) {
    uint8_t base = mod5 & 0x0F;
    return (mod5 & 0x10) ? (base << 4) : base;
}

// Convert HID modifier keycode (0xE0-0xE7) to 8-bit HID modifier bit
#define MOD_BIT(kc)  (1u << ((kc) - 0xE0))

// ---------------------------------------------------------------------------
// Mods+Key — basic keycode with modifier(s) applied
// ---------------------------------------------------------------------------
// Encoding: (mod5 << 8) | kc   — mod5 is never 0, so the result is always in the
// 0x0100-0x1FFF range. Do NOT also OR a 0x0100 marker: bit 8 is MOD_LCTL's own
// bit, so an extra 0x0100 would inject a spurious Ctrl into every non-Ctrl mod
// (e.g. LSFT → Ctrl+Shift). MODKEY_MOD() = (kc>>8)&0x1F expects a clean mod5.
// Range: 0x0100-0x1FFF
#define LCTL(kc)  ((MOD_LCTL << 8) | (kc))
#define LSFT(kc)  ((MOD_LSFT << 8) | (kc))
#define LALT(kc)  ((MOD_LALT << 8) | (kc))
#define LGUI(kc)  ((MOD_LGUI << 8) | (kc))
#define RCTL(kc)  ((MOD_RCTL << 8) | (kc))
#define RSFT(kc)  ((MOD_RSFT << 8) | (kc))
#define RALT(kc)  ((MOD_RALT << 8) | (kc))
#define RGUI(kc)  ((MOD_RGUI << 8) | (kc))

// Shorthand: S(kc) = LSFT(kc), e.g. S(KC_1) = '!'
#define S(kc)  LSFT(kc)

// ---------------------------------------------------------------------------
// Mod-Tap — Hold = modifier, Tap = basic keycode
// ---------------------------------------------------------------------------
// Encoding: 0x2000 | (mod5 << 8) | kc
// Range: 0x2000-0x3FFF

#define MT(mod, kc)  (0x2000 | ((mod) << 8) | (kc))

#define LCTL_T(kc)  MT(MOD_LCTL, kc)
#define LSFT_T(kc)  MT(MOD_LSFT, kc)
#define LALT_T(kc)  MT(MOD_LALT, kc)
#define LGUI_T(kc)  MT(MOD_LGUI, kc)
#define RCTL_T(kc)  MT(MOD_RCTL, kc)
#define RSFT_T(kc)  MT(MOD_RSFT, kc)
#define RALT_T(kc)  MT(MOD_RALT, kc)
#define RGUI_T(kc)  MT(MOD_RGUI, kc)

// ---------------------------------------------------------------------------
// Layer-Tap — Hold = MO(layer), Tap = basic keycode
// ---------------------------------------------------------------------------
// Encoding: 0x4000 | (layer << 8) | kc
// Range: 0x4000-0x4FFF  (layer 0-15)

#define LT(layer, kc)  (0x4000 | ((layer) << 8) | (kc))

// ---------------------------------------------------------------------------
// Layer keys — pure layer operations (no keycode output)
// ---------------------------------------------------------------------------
// Each operation type gets its own 0x100 subrange.
// Range: 0x5100-0x55FF

#define MO(layer)   (0x5100 | (layer))   // Momentary: ON while held
#define TG(layer)   (0x5200 | (layer))   // Toggle: flip on/off
#define TO(layer)   (0x5300 | (layer))   // Turn ON (and OFF all others except default)
#define DF(layer)   (0x5400 | (layer))   // Set default layer
#define OSL(layer)  (0x5500 | (layer))   // One-shot: active for next keypress only

// FN alias: momentary entry into the firmware-owned Settings layer.
// Place anywhere in the keymap — no longer hardcoded by row/col.
#define KC_FN       MO(LAYER_SETTINGS)

// ---------------------------------------------------------------------------
// Tap Dance — multi-tap counting (Phase 9)
// ---------------------------------------------------------------------------
// Encoding: 0x5700 | index (0-31)
// Range: 0x5700-0x571F

#define TD(n)  (0x5700 | (n))

// ---------------------------------------------------------------------------
// Mouse Keys — keyboard-emulated mouse (Phase 9)
// ---------------------------------------------------------------------------
// Range: 0x6000-0x60FF

// Movement
#define KC_MS_U   0x6000   // cursor up
#define KC_MS_D   0x6001   // cursor down
#define KC_MS_L   0x6002   // cursor left
#define KC_MS_R   0x6003   // cursor right

// Buttons
#define KC_BTN1   0x6010   // left click
#define KC_BTN2   0x6011   // right click
#define KC_BTN3   0x6012   // middle click
#define KC_BTN4   0x6013   // back
#define KC_BTN5   0x6014   // forward

// Wheel
#define KC_WH_U   0x6020   // scroll up
#define KC_WH_D   0x6021   // scroll down
#define KC_WH_L   0x6022   // scroll left (pan)
#define KC_WH_R   0x6023   // scroll right (pan)

// Acceleration presets
#define KC_ACL0   0x6030   // slow
#define KC_ACL1   0x6031   // medium (default)
#define KC_ACL2   0x6032   // fast

// ---------------------------------------------------------------------------
// CMK custom keycodes — firmware-specific actions
// ---------------------------------------------------------------------------
// Range: 0x7C00-0x7FFF

#define CMK_TD0      0x7C00   // DS slot 0: short=directed ADV, long=clear+general ADV
#define CMK_TD1      0x7C01   // DS slot 1
#define CMK_TD2      0x7C02   // DS slot 2
#define CMK_TU       0x7C03   // US: short=reconnect/scan upstream, long=clear+general scan

#define CMK_EE_CLR   0x7C10   // Long press 3s → clear all BLE bonds + SM keys
#define CMK_RESET    0x7C11   // Press → watchdog_reboot()

// HID output destination selector (Phase 14 ext) — see cmk/output_select.h.
// Persisted in eeconfig (factory default = USB). CMK_TD0/1/2 also set BLE.
#define CMK_OUT_USB  0x7C12   // Route HID output to USB
#define CMK_OUT_BLE  0x7C13   // Route HID output to BLE (active DS slot)

#define CMK_RGB_TOG  0x7C20   // RGB toggle on/off
#define CMK_RGB_INC  0x7C21   // RGB brightness up (alias: CMK_RGB_VAI)
#define CMK_RGB_DEC  0x7C22   // RGB brightness down (alias: CMK_RGB_VAD)
#define CMK_RGB_MOD  0x7C23   // RGB next effect mode
#define CMK_RGB_RMOD 0x7C24   // RGB previous effect mode
#define CMK_RGB_HUI  0x7C25   // RGB hue up
#define CMK_RGB_HUD  0x7C26   // RGB hue down
#define CMK_RGB_SAI  0x7C27   // RGB saturation up
#define CMK_RGB_SAD  0x7C28   // RGB saturation down
#define CMK_RGB_SPI  0x7C29   // RGB speed up
#define CMK_RGB_SPD  0x7C2A   // RGB speed down
#define CMK_RGB_VAI  CMK_RGB_INC  // alias
#define CMK_RGB_VAD  CMK_RGB_DEC  // alias

// Backlight (Phase 13d)
#define CMK_BL_TOGG  0x7C2B   // Backlight toggle
#define CMK_BL_STEP  0x7C2C   // Backlight cycle levels
#define CMK_BL_UP    0x7C2D   // Backlight brightness up
#define CMK_BL_DOWN  0x7C2E   // Backlight brightness down

// RGB ring keycodes — each press cycles through 8 preset values for one
// parameter (instead of needing a +/- pair). All persisted in eeconfig.
#define CMK_RGB_VAL_R   0x7C2F   // brightness (val) ring
#define CMK_RGB_HUE_R   0x7C30   // hue ring
#define CMK_RGB_SAT_R   0x7C31   // saturation ring
#define CMK_RGB_SPD_R   0x7C32   // animation frame-rate ring
#define CMK_RGB_WSPD_R  0x7C33   // reactive wave front speed ring
#define CMK_RGB_WWID_R  0x7C34   // reactive wave-ring width ring
#define CMK_RGB_FADE_R  0x7C35   // reactive fade duration ring
#define CMK_RGB_HSFT_R  0x7C36   // reactive hue-shift-with-distance ring

// Phase 9 custom keycodes
#define CMK_CAPS_WORD  0x7C40   // Caps Word toggle
#define CMK_REP        0x7C41   // Repeat last keycode
#define CMK_AREP       0x7C42   // Alternate repeat (user-defined)

// Phase 10 custom keycodes
#define CMK_LEAD             0x7C50   // Leader Key toggle
#define CMK_AUTO_SHIFT_TOG   0x7C51   // Auto Shift toggle

#define CMK_GESC             0x7C52   // Grave Escape (ESC or ` based on mods)

#define SC_LSPO              0x7C53   // Space Cadet: Left Shift / (
#define SC_RSPC              0x7C54   // Space Cadet: Right Shift / )
#define SC_LCPO              0x7C55   // Space Cadet: Left Ctrl / (
#define SC_RCPC              0x7C56   // Space Cadet: Right Ctrl / )

#define DM_REC1              0x7C58   // Dynamic Macro: start recording macro 1
#define DM_REC2              0x7C59   // Dynamic Macro: start recording macro 2
#define DM_PLY1              0x7C5A   // Dynamic Macro: play macro 1
#define DM_PLY2              0x7C5B   // Dynamic Macro: play macro 2
#define DM_RSTP              0x7C5C   // Dynamic Macro: stop recording

#define MG_SWAP_CTL_CAPS     0x7C60   // Magic: swap Ctrl ↔ Caps Lock
#define MG_SWAP_ESC_CAPS     0x7C61   // Magic: swap ESC ↔ Caps Lock
#define MG_CAPS_AS_CTL       0x7C62   // Magic: Caps Lock → Left Ctrl
#define MG_SWAP_LALT_LGUI    0x7C63   // Magic: swap Left Alt ↔ Left GUI
#define MG_SWAP_RALT_RGUI    0x7C64   // Magic: swap Right Alt ↔ Right GUI
#define MG_GUI_OFF           0x7C65   // Magic: disable GUI keys
#define MG_GUI_ON            0x7C66   // Magic: enable GUI keys

// Phase 11 custom keycodes
#define CMK_LAYER_LOCK       0x7C67   // Layer Lock: toggle lock on highest active layer

#define SH_ON                0x7C68   // Swap Hands: ON
#define SH_OFF               0x7C69   // Swap Hands: OFF
#define SH_TOGG              0x7C6A   // Swap Hands: toggle
#define SH_MON               0x7C6B   // Swap Hands: momentary ON
#define SH_MOFF              0x7C6C   // Swap Hands: momentary OFF
#define SH_OS                0x7C6D   // Swap Hands: one-shot

#define CMK_KEY_LOCK         0x7C6E   // Key Lock: lock/unlock next key

#define CMK_SECURE_LOCK      0x7C70   // Secure: lock keyboard
#define CMK_SECURE_UNLOCK    0x7C71   // Secure: unlock keyboard
#define CMK_SECURE_TOGGLE    0x7C72   // Secure: toggle lock

#define CMK_VELOCIKEY_TOG    0x7C73   // Velocikey: toggle

// Phase 20 — Inter-half I2C config transfer
#define CMK_CFG_PUSH         0x7C74   // push this half's config to the peer (become I2C master)
#define CMK_CFG_RECV         0x7C75   // arm to receive a config push (become I2C slave)
#define CMK_SPLIT_TOGG       0x7C76   // toggle this half's split_mode (same as SET_SETTING 0x05)

// Phase 12 custom keycodes — Unicode mode switching
#define CMK_UC_WIN           0x7C80   // Unicode mode: Windows (Alt+numpad hex)
#define CMK_UC_MAC           0x7C81   // Unicode mode: macOS (Option+hex)
#define CMK_UC_LINX          0x7C82   // Unicode mode: Linux (Ctrl+Shift+U)
#define CMK_UC_WINC          0x7C83   // Unicode mode: WinCompose
#define CMK_UC_NEXT          0x7C84   // Unicode mode: cycle next
#define CMK_UC_PREV          0x7C85   // Unicode mode: cycle prev
#define CMK_UCIS             0x7C86   // UCIS: start abbreviation input

// Programmable Buttons (32 HID buttons)
// Range: 0x7000-0x701F
#define PB_1    0x7000
#define PB_2    0x7001
#define PB_3    0x7002
#define PB_4    0x7003
#define PB_5    0x7004
#define PB_6    0x7005
#define PB_7    0x7006
#define PB_8    0x7007
#define PB_9    0x7008
#define PB_10   0x7009
#define PB_11   0x700A
#define PB_12   0x700B
#define PB_13   0x700C
#define PB_14   0x700D
#define PB_15   0x700E
#define PB_16   0x700F
#define PB_17   0x7010
#define PB_18   0x7011
#define PB_19   0x7012
#define PB_20   0x7013
#define PB_21   0x7014
#define PB_22   0x7015
#define PB_23   0x7016
#define PB_24   0x7017
#define PB_25   0x7018
#define PB_26   0x7019
#define PB_27   0x701A
#define PB_28   0x701B
#define PB_29   0x701C
#define PB_30   0x701D
#define PB_31   0x701E
#define PB_32   0x701F

// ---------------------------------------------------------------------------
// Consumer Page keycodes (bug #6) — media/volume/brightness/screen controls.
// The low byte is an index into consumer_usage_map[] (process_consumer.c),
// which yields the 16-bit Consumer usage sent in the Report ID 4 field.
// ---------------------------------------------------------------------------

#define KC_MUTE   0x7100   // Consumer 0x00E2 — Mute
#define KC_VOLU   0x7101   // Consumer 0x00E9 — Volume Increment
#define KC_VOLD   0x7102   // Consumer 0x00EA — Volume Decrement
#define KC_MPLY   0x7103   // Consumer 0x00CD — Play/Pause
#define KC_MNXT   0x7104   // Consumer 0x00B5 — Scan Next Track
#define KC_MPRV   0x7105   // Consumer 0x00B6 — Scan Previous Track
#define KC_MSTP   0x7106   // Consumer 0x00B7 — Stop
#define KC_BRIU   0x7107   // Consumer 0x006F — Display Brightness Increment
#define KC_BRID   0x7108   // Consumer 0x0070 — Display Brightness Decrement
#define KC_MCTL   0x7109   // Consumer 0x029F — AC Desktop Show All Windows (Mission Control)
#define KC_LPAD   0x710A   // Consumer 0x02A0 — AC Soft Key Left (Launchpad)
#define CONSUMER_KC_COUNT  11

// ---------------------------------------------------------------------------
// Macro storage keycodes (Phase 15e) — MACRO(n) plays EEPROM-stored macro n.
// QMK-compatible base (QK_MACRO = 0x7700), n = 0..MACRO_COUNT-1.
// ---------------------------------------------------------------------------

#define MACRO(n)   (0x7700 | ((n) & 0x0F))

// ---------------------------------------------------------------------------
// Unicode codepoint encoding macros
// ---------------------------------------------------------------------------
// UC(c):  direct codepoint (BMP 0x0000-0x7FFF) → 0x8000 | c
// UM(i):  unicode map index (0-255) → 0x6100 | i
// UP(i,j): unicode map pair (unshifted=i, shifted=j) → 0x6200 | (i<<4) | j

#define UC(c)      (0x8000 | ((c) & 0x7FFF))
#define UM(i)      (0x6100 | ((i) & 0xFF))
#define UP(i, j)   (0x6200 | (((i) & 0xF) << 4) | ((j) & 0xF))

// ---------------------------------------------------------------------------
// Classification macros
// ---------------------------------------------------------------------------

#define IS_BASIC(kc)      ((kc) <= 0x00FF)
#define IS_MOD_KEY(kc)    ((kc) >= 0x0100 && (kc) <= 0x1FFF)
#define IS_MOD_TAP(kc)    ((kc) >= 0x2000 && (kc) <= 0x3FFF)
#define IS_LAYER_TAP(kc)  ((kc) >= 0x4000 && (kc) <= 0x4FFF)
#define IS_LAYER_KEY(kc)  ((kc) >= 0x5100 && (kc) <= 0x55FF)
#define IS_TAP_DANCE(kc)  ((kc) >= 0x5700 && (kc) <= 0x571F)
#define IS_MOUSEKEY(kc)   ((kc) >= 0x6000 && (kc) <= 0x60FF)
#define IS_CMK_KEY(kc)    ((kc) >= 0x7C00 && (kc) <= 0x7FFF)
#define IS_SPACE_CADET(kc) ((kc) >= SC_LSPO && (kc) <= SC_RCPC)
#define IS_DYN_MACRO(kc)  ((kc) >= DM_REC1 && (kc) <= DM_RSTP)
#define IS_MAGIC(kc)      ((kc) >= MG_SWAP_CTL_CAPS && (kc) <= MG_GUI_ON)
#define IS_SWAP_HANDS(kc) ((kc) >= SH_ON && (kc) <= SH_OS)
#define IS_SECURE(kc)     ((kc) >= CMK_SECURE_LOCK && (kc) <= CMK_SECURE_TOGGLE)
#define IS_RGB(kc)        (((kc) >= CMK_RGB_TOG  && (kc) <= CMK_RGB_SPD) || \
                           ((kc) >= CMK_RGB_VAL_R && (kc) <= CMK_RGB_HSFT_R))
#define IS_BACKLIGHT(kc)  ((kc) >= CMK_BL_TOGG && (kc) <= CMK_BL_DOWN)
#define IS_UNICODE(kc)      ((kc) >= 0x8000)
#define IS_UNICODEMAP(kc)   ((kc) >= 0x6100 && (kc) <= 0x61FF)
#define IS_UNICODEMAP_PAIR(kc) ((kc) >= 0x6200 && (kc) <= 0x62FF)
#define IS_PROG_BUTTON(kc)  ((kc) >= PB_1 && (kc) <= PB_32)
#define IS_UC_MODE(kc)      ((kc) >= CMK_UC_WIN && (kc) <= CMK_UC_PREV)
#define IS_MACRO_KC(kc)     (((kc) & 0xFFF0) == 0x7700)
#define IS_CONSUMER(kc)     (((kc) & 0xFF00) == 0x7100)

// Sub-classification for layer keys
#define IS_MO(kc)   ((kc) >= 0x5100 && (kc) <= 0x51FF)
#define IS_TG(kc)   ((kc) >= 0x5200 && (kc) <= 0x52FF)
#define IS_TO(kc)   ((kc) >= 0x5300 && (kc) <= 0x53FF)
#define IS_DF(kc)   ((kc) >= 0x5400 && (kc) <= 0x54FF)
#define IS_OSL(kc)  ((kc) >= 0x5500 && (kc) <= 0x55FF)

// Tap Dance extraction
#define TD_INDEX(kc)  ((kc) & 0x1F)

// Macro extraction (Phase 15e)
#define MACRO_INDEX(kc)  ((kc) & 0x0F)

// Consumer extraction (bug #6) — index into consumer_usage_map[]
#define CONSUMER_INDEX(kc)  ((kc) & 0xFF)

// Mouse Keys sub-classification
#define IS_MOUSEKEY_MOVE(kc)   ((kc) >= 0x6000 && (kc) <= 0x600F)
#define IS_MOUSEKEY_BUTTON(kc) ((kc) >= 0x6010 && (kc) <= 0x601F)
#define IS_MOUSEKEY_WHEEL(kc)  ((kc) >= 0x6020 && (kc) <= 0x602F)
#define IS_MOUSEKEY_ACCEL(kc)  ((kc) >= 0x6030 && (kc) <= 0x603F)

// HID modifier keycode check (0xE0-0xE7)
#define IS_HID_MOD(kc)  ((kc) >= 0xE0 && (kc) <= 0xE7)

// ---------------------------------------------------------------------------
// Extraction macros
// ---------------------------------------------------------------------------

// Mods+Key: extract 5-bit mod and basic keycode
#define MODKEY_MOD(kc)   (((kc) >> 8) & 0x1F)
#define MODKEY_KC(kc)    ((kc) & 0xFF)

// Mod-Tap: extract 5-bit mod and basic keycode
#define MT_MOD(kc)       (((kc) >> 8) & 0x1F)
#define MT_KC(kc)        ((kc) & 0xFF)

// Layer-Tap: extract layer and basic keycode
#define LT_LAYER(kc)    (((kc) >> 8) & 0x0F)
#define LT_KC(kc)       ((kc) & 0xFF)

// Layer key: extract layer number
#define LAYER_VAL(kc)   ((kc) & 0xFF)

#endif // CMK_KEYCODE_H
