/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
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
// cmk_config.h — Firmware-level configuration.
//
// Hardware pinout / peripheral selection / I2C addresses live in board.h.
// This file only contains firmware parameters that do not depend on the
// specific board: matrix size is sourced from board.h, but timings, feature
// flags, task priorities, and the action-system limits are here.
//

#ifndef CMK_CONFIG_H
#define CMK_CONFIG_H

// Pull in board-specific hardware definitions first. board.h provides:
//   - MATRIX_ROWS, MATRIX_COLS, MATRIX_ROW_PINS, MATRIX_COL_PINS, DIODE_DIRECTION
//   - PIN_I2C_SDA/SCL, I2C_ADDR_* device addresses
//   - PIN_UART_TX/RX
//   - PIN_RGB, PIN_VLED_EN, RGB_LED_COUNT
//   - PIN_BC_*, PIN_USB_*, ADC channels
//   - CYW43_DEFAULT_PIN_WL_* overrides
#include "board.h"

// Firmware version (semver) — single source of truth. See version.h.
#include "version.h"

// ---------------------------------------------------------------------------
// Debounce (algorithm sym_defer_g — identical to QMK default)
// ---------------------------------------------------------------------------

#define DEBOUNCE          5    // ms — debounce time

// ---------------------------------------------------------------------------
// Layer system (Phase 7)
// ---------------------------------------------------------------------------

#define NUM_LAYERS          6     // 0=Base, 1-3=User, 4=US Safe, 5=Settings
#define LAYER_US_SAFE       4     // Protected US ANSI layout (read-only)
#define LAYER_SETTINGS      5     // Protected firmware settings layer (read-only)

// ---------------------------------------------------------------------------
// Tapping (Phase 7 — Mod-Tap / Layer-Tap)
// ---------------------------------------------------------------------------

#define TAPPING_TERM        200   // ms — hold threshold for MT() and LT()
#define TAPPING_BUFFER_SIZE   8   // max buffered events during tap/hold resolution

// tap_code (non-blocking tap emitter): press and release are emitted this many ms
// apart, on separate keyscan ticks. Must be ≥ one BLE connection interval (~15 ms)
// so the press is transmitted before the release overwrites the last-wins IPC slot;
// it is also the press duration AND the gap between queued taps → caps the tap rate
// at ~1000/(2*DELAY) ≈ 25/s. Lower = snappier (USB-safe) but risks dropped BLE taps.
#define TAP_CODE_DELAY_MS    20
#define TAP_CODE_QUEUE_SIZE  16   // pending taps buffered during fast bursts

// ---------------------------------------------------------------------------
// Status LEDs — board-dependent
// ---------------------------------------------------------------------------
// On the current prototype board there are no discrete status LEDs.
// Power/BLE/connection/relay state is shown as an RGB overlay while FN is
// held (handled by led_indicators + rgb_matrix). Code paths that drive
// PIN_LED_POWER / PIN_LED_BLE / PIN_LED_CONN / PIN_LED_RELAY must guard
// the GPIO access with #ifdef.

// ---------------------------------------------------------------------------
// Device names — one firmware, two halves. board_side_name() picks one at boot
// from the detected side (see src/core0/cmk/board_side.{h,c}). Applied to the
// USB product string and the BLE advertised / GAP device name.
// ---------------------------------------------------------------------------

#define CMK_NAME_LEFT   "CMK-Keeeyboard-Split-L"
#define CMK_NAME_RIGHT  "CMK-Keeeyboard-Split-R"

// ---------------------------------------------------------------------------
// HID Report IDs (keyboard + mouse composite)
// ---------------------------------------------------------------------------

#define HID_REPORT_ID_KEYBOARD  1
#define HID_REPORT_ID_MOUSE     2

// ---------------------------------------------------------------------------
// BLE timeout (BTstack timer in Core 1)
// ---------------------------------------------------------------------------

#define ADV_TIMEOUT_MS        10000   // stop advertising if no downstream connects in 10 s
#define SCAN_TIMEOUT_MS       10000   // stop scanning if no upstream found in 10 s
#define CONNECT_TIMEOUT_MS    10000   // abort gap_connect() if upstream doesn't respond in 10 s

// ---------------------------------------------------------------------------
// Keyscan task
// ---------------------------------------------------------------------------

#define KS_SCAN_INTERVAL_MS   1    // scan period (1ms = 1kHz)
#define KS_LONG_PRESS_MS   3000    // FN combo long-press threshold
#define KS_TASK_STACK      1280    // FreeRTOS stack (words) — increased for Phase 9
#define KS_TASK_PRIORITY      3   // R7: ABOVE CM/USB/PM (2) — the 1 kHz scan must
                                  // never wait out another task's time slice

// ---------------------------------------------------------------------------
// Low Power Mode (automatic idle sleep) — two tiers, see low_power.c
// ---------------------------------------------------------------------------
//
// ACTIVE → LIGHT (after LP_LIGHT_IDLE_MS of no key activity): cut the LEDs +
// 5V boost rail and throttle the keyscan to LP_LIGHT_SCAN_MS; BLE stays
// connected (instant wake). LIGHT → DEEP (after LP_DEEP_IDLE_MS): power the BLE
// radio off and sleep until a column GPIO edge (any key). The __wfi idle hook
// (vApplicationIdleHook) is the always-on baseline below all tiers.
#define LOW_POWER_ENABLE
#define LP_LIGHT_IDLE_MS    60000u    // 60 s  → LEDs off + keyscan throttle
#define LP_DEEP_IDLE_MS    300000u    // 5 min → radio off + GPIO-IRQ deep sleep
#define LP_LIGHT_SCAN_MS       10u    // throttled scan period in LIGHT/DEEP (1 kHz → 100 Hz)
// ih_link (I2C1 config transfer) poll period in low-power tiers: at the normal
// 1 ms it would wake the core every tick and defeat tickless idle.
#define IH_LP_LIGHT_MS         25u    // LIGHT: slower config-transfer polling
#define IH_LP_DEEP_MS         100u    // DEEP: link quiesced, just check for wake
#define LP_WAKE_PULSE_US       80u    // cross-half wake: SDA LOW pulse width (item 3)
#define USB_IDLE_POLL_MS       10u    // USB tud_task poll when cable unplugged (battery)

// ---------------------------------------------------------------------------
// Phase 9 — Key Feature Configuration
// ---------------------------------------------------------------------------

// Combo (simultaneous key press → action)
#define COMBO_TERM          50    // ms — max window for simultaneous key detection
#define COMBO_COUNT          8    // max combo definitions
#define COMBO_MAX_KEYS       4    // max keys per combo

// Tap Dance (multi-tap counting)
#define TAP_DANCE_COUNT      4    // max tap dance definitions

// Caps Word (auto-shift while typing)
#define CAPS_WORD_IDLE_TIMEOUT  5000  // ms — auto-deactivate after idle

// Key Override (modifier-based remap)
#define KEY_OVERRIDE_COUNT   8    // max override definitions

// Mouse Keys (keyboard-emulated mouse)
#define MK_INTERVAL         10    // ms between mouse reports while keys held
#define MK_MOVE_DELTA        5    // pixels per report (cursor movement)
#define MK_WHEEL_DELTA       1    // clicks per report (scroll)

// Send String (character sequence output)
#define SEND_STRING_DELAY   10    // ms between characters (non-blocking)

// ---------------------------------------------------------------------------
// Phase 10 — Key Feature Configuration
// ---------------------------------------------------------------------------

// Auto Shift (hold brief → auto-shift)
#define AUTO_SHIFT_TIMEOUT  175   // ms — hold threshold for auto-shift
#define AS_BUFFER_SIZE        4   // max buffered events during auto-shift resolution

// Leader Key (sequence → action)
#define LEADER_TIMEOUT      300   // ms — timeout for leader sequence
#define LEADER_SEQ_MAX        5   // max keys in a leader sequence

// Dynamic Macro (record/play key sequences)
#define DYNAMIC_MACRO_SIZE  128   // max events total (shared buffer, 512B RAM)
#define DYNAMIC_MACRO_DELAY  10   // ms between events during playback

// Space Cadet (Shift tap → parenthesis)
#define SPACE_CADET_TERM    200   // ms — tap threshold for Space Cadet

// ---------------------------------------------------------------------------
// Phase 11 — Key Feature Configuration
// ---------------------------------------------------------------------------

// Bootmagic (boot-time factory reset via held key)
#define BOOTMAGIC_ROW        0
#define BOOTMAGIC_COL        0    // matrix 4x6: top-left key (ROW_0 x COL_0)

// Tri-Layer (layer LOWER+UPPER active → auto layer ADJUST)
// Disabilitato: layer 1/2/3 sono usati come momentary indipendenti per Qwerty/Colemak,
// non come schema LOWER+UPPER → ADJUST. Valori = NUM_LAYERS → bit fuori range,
// `tri_layer_update()` non muta mai lo stato. Per riattivare: rimettere layer reali.
#define TRI_LAYER_LOWER      NUM_LAYERS
#define TRI_LAYER_UPPER      NUM_LAYERS
#define TRI_LAYER_ADJUST     NUM_LAYERS

// Secure (keyboard lock — unlock sequence)
#define SECURE_UNLOCK_SEQ_MAX  4  // max keys in unlock sequence

// ---------------------------------------------------------------------------
// Phase 12 — Key Feature Configuration
// ---------------------------------------------------------------------------

// Unicode input
#define UNICODE_ENABLE             // Unicode keycodes active; advertised in feature_flags
#define UNICODE_SEND_DELAY    10   // ms between steps in unicode send sequence
#define UC_SEQ_MAX            16   // max steps in one unicode send sequence

// UCIS (Unicode Character Input System)
#define UCIS_MAX_INPUT         8   // max chars in abbreviation

// Programmable Button HID
#define HID_REPORT_ID_PROG_BUTTON  3

// ---------------------------------------------------------------------------
// Phase 13 — RGB/LED Subsystem Selection
// ---------------------------------------------------------------------------
// Choose ONE of the first three (or none). BACKLIGHT is independent.
//   RGBLIGHT_ENABLE      — strip underglow (linear, no per-key mapping)
//   RGB_MATRIX_ENABLE    — per-key RGB (positional + reactive effects)
//   LED_MATRIX_ENABLE    — per-key monochrome (brightness only)
//   BACKLIGHT_ENABLE     — single-channel PWM backlight (independent)

// #define RGBLIGHT_ENABLE
#define RGB_MATRIX_ENABLE
// #define LED_MATRIX_ENABLE
// #define BACKLIGHT_ENABLE

// Phase 19 — FN status overlay: while FN is held, the RGB matrix shows a status
// dashboard (BLE DS/US, USB, battery, active DF) instead of the normal effect.
// Requires RGB_MATRIX_ENABLE. See src/core0/cmk/fn_overlay.{h,c}.
#define FN_OVERLAY_ENABLE

// R5 — Shared status-blink half-periods. Used by BOTH the GP3 status LED
// (led_manager) and the FN dashboard (fn_overlay), phase-locked to the absolute
// millisecond tick so every indicator blinks in unison.
#define CMK_BLINK_FAST_HALF_MS  150      // general ADV / scanning (~3.3 Hz)
#define CMK_BLINK_SLOW_HALF_MS  400      // directed ADV / connecting (~1.25 Hz)

// ---------------------------------------------------------------------------
// Phase 13 — Shared RGB/LED parameters (PIN_RGB / RGB_LED_COUNT in board.h)
// ---------------------------------------------------------------------------

#define RGB_FRAME_INTERVAL_MS   33       // ~30fps frame rate
#define RGB_HUE_STEP            8        // hue increment per keypress
#define RGB_SAT_STEP            16       // saturation increment per keypress
#define RGB_VAL_STEP            16       // brightness increment per keypress
#define RGB_SPD_STEP            16       // speed increment per keypress
#define RGB_BRIGHTNESS_MAX      255
#define RGB_DEFAULT_ENABLE      1
#define RGB_DEFAULT_MODE        5        // RGB_MX_REACTIVE_SPLASH (HSV wave on keypress)
#define RGB_DEFAULT_HUE         0
#define RGB_DEFAULT_SAT         255      // full saturation → vivid colours
#define RGB_DEFAULT_VAL         128      // half brightness (full was painful)
#define RGB_DEFAULT_SPEED       128

// Reactive-effect defaults (middle of the 8-step rings)
#define RGB_DEFAULT_WAVE_SPEED  6        // wave-front propagation (px/tick)
#define RGB_DEFAULT_WAVE_WIDTH  20       // ring width (px)
#define RGB_DEFAULT_FADE        40       // frames until hit fades (~1.3s @30fps)
#define RGB_DEFAULT_HUE_SHIFT   4        // hue change per tick along the wave

// ---------------------------------------------------------------------------
// Phase 13b — RGB Matrix Configuration
// ---------------------------------------------------------------------------

#define RGB_MATRIX_HIT_MAX          8    // max concurrent reactive hits tracked
#define RGB_MATRIX_REACTIVE_FADE   40    // frames until hit fades (~1.3s at 30fps)

// --- Cross-half RGB sync (Phase 24) ------------------------------------------
// MASTER→SLAVE effect-state sync over the rgb_sync GATT channel so both halves
// render one animation in phase. Active ONLY in split MASTER/SLAVE role.
#define CMK_SPLIT_RGB_SYNC           1    // master pushes RGB state to the slave
#define CMK_SPLIT_RGB_SYNC_MS        250  // state + phase resync period (~4 Hz)
// Unified physical-X offset applied to the RIGHT half while the split is engaged,
// so positional effects (rainbow/swirl/rain) form a continuous gradient across
// both halves. Left half = 0; right half starts past the left's 224 px width.
#define RGB_MATRIX_SPLIT_X_OFFSET    270

// ---------------------------------------------------------------------------
// Phase 13d — Backlight Configuration
// ---------------------------------------------------------------------------
// Not used on the current prototype board (no PWM backlight FET wired).
// If a future board defines PIN_BACKLIGHT in board.h and enables
// BACKLIGHT_ENABLE, the levels below are used by backlight.c.

#define BACKLIGHT_LEVELS         5       // 0=off, 1-5 brightness steps
#define BACKLIGHT_DEFAULT_LEVEL  3       // default brightness on first boot

// ---------------------------------------------------------------------------
// Phase 14 — USB HID (TinyUSB)
// ---------------------------------------------------------------------------
// HID output destination is chosen explicitly (Phase 14 ext): CMK_OUT_USB / the
// BLE slot keys (CMK_TD0/1/2) select USB or BLE, persisted in eeconfig. See
// cmk/output_select.c. Factory default = USB.

#define USB_ENABLE                          // enable TinyUSB HID composite
// #define OS_DETECTION_ENABLE              // detect host OS from USB descriptors
#define RAW_HID_ENABLE                      // 32-byte bidirectional WebHID channel (Interface 1)

#define USB_VID             0x1209          // pid.codes open-source VID
#define USB_PID             0xCB01          // CMK project PID (CB = custom board)
#define USB_MANUFACTURER    "CMK"
// USB product string is per-side at runtime — see board_side_name() and
// tud_descriptor_string_cb() in src/usb/usb_descriptors.c (index 2).

#define USB_TASK_STACK      1024            // FreeRTOS stack (words, 4KB) — TinyUSB init + descriptor callbacks
// Same priority as keyscan/CM (2). The WS2812 flush is now DMA-driven (immune to
// preemption), so the USB task no longer needs to be starved to avoid LED
// flicker. NOT max priority: that hung boot during Core 1 CYW43/BTstack bringup.
#define USB_TASK_PRIORITY   2

#define RAW_HID_BUFFER_SIZE 32              // bytes per raw HID packet
// (R7: il ring buffer raw HID in ingresso è stato rimosso — la consegna è solo
// via callback sincrona; RAW_HID_QUEUE_DEPTH non esiste più.)

#define OS_DETECTION_DEBOUNCE_MS  500       // ms before locking detected OS

// ---------------------------------------------------------------------------
// Phase 15 — Dynamic Keymap + CMK Config Protocol (WebHID configurator)
// ---------------------------------------------------------------------------
// Runtime-editable keymap persisted on EEPROM, configured over Raw HID (USB).
// See src/core0/core/dynamic_keymap.{h,c}, src/usb/cmk_config_protocol.{h,c},
// and CMK_Config_Protocol.md. Requires RAW_HID_ENABLE.

#define DYNAMIC_KEYMAP_ENABLE               // keymap stored in / loaded from EEPROM at runtime

#define DYNAMIC_KEYMAP_ADDR     0x2000      // EEPROM region base (reserved, ~8KB)
#define DYNAMIC_KEYMAP_MAGIC    0x4B44u     // 'DK' — region header magic
#define DYNAMIC_KEYMAP_VERSION  2           // v2 (Fase 18): unified 4×12 keymap (1152 B)

#define CMK_PROTOCOL_VERSION    0x0004      // CMK Config Protocol spec version implemented
                                            // v2: RGB commands gained a per-half `target`
                                            // v3: uniform `target` byte at req[1] on EVERY command
                                            //     (args shift to req[2]); centralised per-side routing
                                            // byte (0=LEFT/1=RIGHT/2=BOTH). See §6.
                                            // v4 (additive, framing unchanged): GET_DIAG (0x04) +
                                            //     ST_OK_LOCAL_ONLY (0x08) for BOTH writes with
                                            //     peer not joined (was a silent ST_OK in v3).

// Magic token required by CMD_BOOTLOADER (0x0F) to actually reboot into the ROM
// BOOTSEL. The configurator must send these 4 bytes (big-endian) at req[2..5];
// any other value is rejected with ST_BAD_PARAMS and does NOT reboot. Guards
// against accidental/hostile reboots from a WebHID page. "CMKB" = 0x434D4B42.
#define CMK_BOOTSEL_MAGIC       0x434D4B42u

// ---------------------------------------------------------------------------
// Key simulation timing (Core 1 BLE test — IPC_SEND_TEST_KEY)
// ---------------------------------------------------------------------------

#define KEY_TEST_INTERVAL_MS   3000   // new random key every 3 s
#define KEY_TEST_PRESS_MS        50   // key-down duration before key-up

// ---------------------------------------------------------------------------
// Phase 17 — Power Management (BQ27441 fuel gauge + BQ24296 charger)
// ---------------------------------------------------------------------------
// Battery: single-cell LiPo, 2500 mAh nominal. The BQ27441-G1A defaults to a
// 1340 mAh profile, so its Design Capacity / Design Energy are reprogrammed
// once (only when they differ) by pm_fuel_gauge_configure() at boot. Design
// Energy = capacity x 3.7 V for the -G1A part.

#define PM_BATTERY_DESIGN_CAPACITY_MAH   2500   // BQ27441 State subclass 82, offset 10
#define PM_BATTERY_DESIGN_ENERGY_MWH     9250   // = 2500 x 3.7 (offset 12)

// Low-battery threshold with hysteresis (SoC %). Below CRITICAL the RGB matrix
// blanks to save power (FN overlay still works); it recovers above RECOVER.
// Tunable once the SoC readings are validated against the real cell.
#define PM_BATTERY_CRITICAL_PCT          8
#define PM_BATTERY_CRITICAL_RECOVER_PCT  15

// ---------------------------------------------------------------------------
// Phase 20 — Inter-half I2C config transfer (magnetic connector)
// ---------------------------------------------------------------------------
// Push the dynamic keymap + eeconfig from one half to the other over the I2C1
// link (board.h: PIN_I2C1_*, CMK_I2C1_ADDR, I2C1_BUS_HZ) without a PC. Arm the
// receiver with CMK_CFG_RECV (I2C slave) then push from the source with
// CMK_CFG_PUSH (I2C master). See src/core0/cmk/inter_half_sync.{h,c} and
// src/core0/drivers/i2c1_link.{h,c}. Comment out to compile the link out
// (the keycodes then resolve to no-op stubs).

#define INTER_HALF_LINK_ENABLE

#define IH_CHUNK_BYTES       16      // payload bytes per DATA frame
#define IH_ARM_TIMEOUT_MS    30000   // receiver disarms if no push within this window
#define IH_TXN_TIMEOUT_US    50000   // per-transaction timeout (hot-plug safe; ~ms typical)
#define IH_MAX_RETRY         3       // retransmits of a CRC-failed frame before aborting
#define IH_APPLY_REBOOT_MS   600     // delay before the receiver reboots to fully apply config

// Phase 20b — config proxy: master poll spacing while waiting for the peer's
// config-serve tick to produce a response (peer is a separate MCU).
#define IH_PROXY_POLL_US     1500    // wait between response polls (~1 peer tick)
#define IH_PROXY_MAX_POLLS   24      // give up after this many polls (~36 ms worst case)

// A single proxy transaction (write request + poll response) is retried this
// many times before the peer is declared absent (s_peer_joined = false). A
// brief disconnect / bus glitch must not drop the link on the first error —
// mirrors the CRC-retry robustness of the push path (IH_MAX_RETRY). Between
// failed attempts the I2C1 peripheral is re-initialised (unwedge a stuck bus).
#define IH_PROXY_TXN_ATTEMPTS    3     // full (write+poll) attempts before giving up
#define IH_PROXY_RETRY_BACKOFF_US 500  // settle time between a failed attempt and the next

// Config-serve slave self-heal: if no I2C transaction completes within this
// window, re-arm the slave peripheral (recovers a DW_apb_i2c slave left stuck
// by a mid-transaction connector disconnect — otherwise it stops ACKing until
// reboot). Long enough not to fire during an active config session.
#define IH_SLAVE_HEAL_MS     1500

// R6 — inter-half link runs in its OWN FreeRTOS task (no longer in the keyscan
// tick: the slave-leg EEPROM writes froze the scan for tens of ms per chunk).
// Priority BELOW keyscan/CM so config work never delays typing. Stack sized for
// cmk_config_protocol_handle_peer → dk paths (1.2 KB buffers) + EEPROM txbuf.
#define IH_TASK_STACK            2048   // words
#define IH_TASK_PRIORITY         1
// Role hysteresis: usb_hid_is_active() flaps on USB suspend/resume; require the
// desired slave/idle role to be stable before switching (no mid-txn teardowns).
#define IH_ROLE_HYST_MS          500
// A push that accepted FR_START but never completes (connector detached) resets
// after this window, re-enabling the self-heal it would otherwise suppress.
#define IH_PUSH_ACTIVE_TIMEOUT_MS 2000

#endif // CMK_CONFIG_H
