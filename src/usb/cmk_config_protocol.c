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
// cmk_config_protocol.c — see cmk_config_protocol.h and CMK_Protocol.md.
//

#include "config/cmk_config.h"
#include "low_power.h"
#include "pico/time.h"

#ifdef RAW_HID_ENABLE

#include "cmk_config_protocol.h"
#include "raw_hid.h"
#include "board_side.h"
#include "eeconfig.h"
#include "low_power.h"     // low_power_refresh_config() — LP timeouts via SET_SETTING
#include "action_tapping.h" // tapping_term_get() — GET_SETTINGS d[11..12] (bug #8)
#include "process_secure.h"
#include "inter_half_sync.h"
#include "diag.h"
#include "i2c1_link.h"
#include "connection_manager.h"
#include "power_manager.h"   // pm_get_battery_status()/pm_is_charging() — GET_BATTERY
#ifdef MACRO_STORAGE_ENABLE
#include "macro_storage.h"   // Phase 15e: macro buffer commands 0x41/0x43/0x44/0x4F
#include "process_macro.h"   // macro_play_request_abort() before buffer mutations
#endif
#include "ipc/ipc_core.h"
#ifdef USB_ENABLE
#include "usb/usb_hid.h"
#endif

#ifdef DYNAMIC_KEYMAP_ENABLE
#include "dynamic_keymap.h"
#endif

#if defined(RGB_MATRIX_ENABLE)
#include "rgb_matrix.h"
#define CMK_HAS_RGB 1
#elif defined(LED_MATRIX_ENABLE)
#include "led_matrix.h"
#define CMK_HAS_RGB 1
#elif defined(RGBLIGHT_ENABLE)
#include "rgblight.h"
#define CMK_HAS_RGB 1
#endif

#include "pico/bootrom.h"

#include <string.h>

// Buffer-command size limits (protocol v3). SET fills req[5..31] (27 bytes: the
// target byte at req[1] shifted args by +1); GET echoes offset+size in resp[2..4]
// then bytes from resp[5..31] (27 bytes).
#define CFG_BUF_MAX_SET  27
#define CFG_BUF_MAX_GET  27

// Firmware version (semver) comes from the single source of truth in version.h
// (pulled in via config/cmk_config.h). Reported by GET_FW_VERSION (0x03) and in
// the GET_KEYBOARD_INFO version bytes. Independent of CMK_PROTOCOL_VERSION.

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static uint8_t feature_flags(void) {
    uint8_t f = 0;
#ifdef USB_ENABLE
    f |= 0x01;
#endif
#ifdef CMK_HAS_RGB
    f |= 0x02;
#endif
#ifdef RAW_HID_ENABLE
    f |= 0x04;
#endif
#ifdef DYNAMIC_KEYMAP_ENABLE
    f |= 0x08;
#endif
#ifdef MACRO_STORAGE_ENABLE
    f |= 0x10;   // bit4: macro storage (Phase 15e — v5; was the never-set DYNAMIC_MACRO)
#endif
#ifdef BACKLIGHT_ENABLE
    f |= 0x20;
#endif
#ifdef UNICODE_ENABLE
    f |= 0x40;
#endif
    return f;
}

static uint16_t keymap_total_bytes(void) {
#ifdef DYNAMIC_KEYMAP_ENABLE
    return dynamic_keymap_size();
#else
    return (uint16_t)(NUM_LAYERS * MATRIX_ROWS * MATRIX_COLS * 2);
#endif
}

#ifdef CMK_HAS_RGB
static void rgb_reload(void) {
#if defined(RGB_MATRIX_ENABLE)
    rgb_matrix_load_eeconfig();
#elif defined(LED_MATRIX_ENABLE)
    led_matrix_init();
#elif defined(RGBLIGHT_ENABLE)
    rgblight_init();
#endif
}

// --- Per-half RGB targeting (protocol v2) -----------------------------------
// RGB config is stored and rendered independently per half. The handlers below
// route a command at the local half, the peer half (via I2C1 proxy), or both.

// Local apply primitives — factored so the BOTH path reuses them without
// duplicating the EEPROM/reload logic. Each returns a status code.
static uint8_t rgb_local_get(uint8_t out[10]) {
    eeconfig_rgb_t ec = eeconfig_read_rgb();
    memcpy(out, &ec, sizeof(ec));
    return ST_OK;
}

static uint8_t rgb_local_set_config(const uint8_t in[10]) {
    eeconfig_rgb_t ec;
    memcpy(&ec, in, sizeof(ec));
    eeconfig_update_rgb(&ec);
    rgb_reload();
    return ST_OK;
}

static uint8_t rgb_local_set_field(uint8_t field, uint8_t value) {
    if (field >= sizeof(eeconfig_rgb_t)) return ST_OUT_OF_RANGE;
    eeconfig_rgb_t ec = eeconfig_read_rgb();
    ((uint8_t *)&ec)[field] = value;
    eeconfig_update_rgb(&ec);
    rgb_reload();
    return ST_OK;
}
#endif

// Writes (keymap/RGB/settings/eeprom) are rejected while Secure lock is active.
static bool cmd_is_write(uint8_t cmd) {
    switch (cmd) {
        case CMD_SET_KEYMAP_KEYCODE:
        case CMD_SET_KEYMAP_BUFFER:
        case CMD_PUSH_KEYMAP_TO_PEER:
        case CMD_RESET_KEYMAP:
        case CMD_SET_RGB_CONFIG:
        case CMD_SET_RGB_FIELD:
        case CMD_SET_SETTING:
        case CMD_SET_MACRO_BUFFER:
        case CMD_RESET_MACROS:
        case CMD_EEPROM_RESET:
            return true;
        default:
            return false;
    }
}

// ---------------------------------------------------------------------------
// Per-side routing (protocol v3). The USB-origin wrapper classifies each command
// and routes it to the local half, the peer half (I2C1 proxy), or both. The
// LOCAL handler (cfg_dispatch) ignores req[1] and always operates on this half.
// ---------------------------------------------------------------------------

// Target-irrelevant commands: always run on the half that holds the USB cable.
static bool cmd_is_system(uint8_t cmd) {
    switch (cmd) {
        case CMD_GET_PROTOCOL_VERSION:
        case CMD_GET_KEYBOARD_INFO:
        case CMD_GET_FW_VERSION:
        case CMD_GET_DIAG:
        case CMD_COMMIT:
        case CMD_BOOTLOADER:
        case CMD_PUSH_KEYMAP_TO_PEER:   // itself orchestrates the peer copy
            return true;
        default:
            return false;
    }
}

// Read commands return data from exactly one side (BOTH is invalid for them).
static bool cmd_is_read(uint8_t cmd) {
    switch (cmd) {
        case CMD_GET_KEYMAP_KEYCODE:
        case CMD_GET_KEYMAP_BUFFER:
        case CMD_GET_RGB_CONFIG:
        case CMD_GET_SETTINGS:
        case CMD_GET_BATTERY:      // per-side: each half answers with ITS fuel gauge
        case CMD_GET_MACRO_INFO:   // per-side (macros stored per-half like the keymap)
        case CMD_GET_MACRO_BUFFER:
            return true;
        default:
            return false;
    }
}

static uint8_t cfg_local_side(void) { return (uint8_t)board_side_get(); }
static uint8_t cfg_peer_side(void)  {
    return (board_side_get() == CMK_SIDE_LEFT) ? CFG_TGT_RIGHT : CFG_TGT_LEFT;
}
static bool need_local(uint8_t target) {
    return target == CFG_TGT_BOTH || target == cfg_local_side();
}
static bool need_peer(uint8_t target) {
    return target == CFG_TGT_BOTH || target == cfg_peer_side();
}

// ---------------------------------------------------------------------------
// Command handler — fills resp[0..31] from req[0..31].
// ---------------------------------------------------------------------------

// Armed by a CMD_BOOTLOADER carrying the correct magic token; consumed in
// raw_hid_receive_cb() to reboot into BOOTSEL only after the ack has been sent.
static volatile bool s_bootsel_armed = false;

static void cfg_dispatch(const uint8_t *req, uint8_t *resp) {
    uint8_t cmd = req[0];
    memset(resp, 0, RAW_HID_BUFFER_SIZE);
    resp[0] = cmd;          // echo command id
    resp[1] = ST_OK;        // default status
    uint8_t *d = &resp[2];  // response data region (30 bytes)

    // Secure lock: block behaviour-changing writes; reads stay allowed.
    if (cmd_is_write(cmd) && secure_is_locked()) {
        resp[1] = ST_LOCKED;
        return;
    }

    switch (cmd) {

    case CMD_GET_PROTOCOL_VERSION:
        d[0] = (uint8_t)(CMK_PROTOCOL_VERSION >> 8);
        d[1] = (uint8_t)(CMK_PROTOCOL_VERSION & 0xFF);
        break;

    case CMD_GET_FW_VERSION:
        // Firmware semver from version.h (single source of truth).
        d[0] = CMK_FW_VERSION_MAJOR;
        d[1] = CMK_FW_VERSION_MINOR;
        d[2] = CMK_FW_VERSION_PATCH;
        break;

    case CMD_GET_DIAG: {
        // R1 telemetry. req[2] = page (7 counters per page, uint32 BE each).
        // Response: d[0]=page, d[1]=count on this page, d[2..] = count×4 bytes.
        uint8_t page = req[2];
        if (page == 0xFD || page == 0xFC) {
            uint32_t words[7]={1,IPC_TRACE_CAPACITY,ipc_trace_end(),0,0,0,0};
            if (page==0xFC) {
                uint32_t seq=((uint32_t)req[3]<<24)|((uint32_t)req[4]<<16)|
                             ((uint32_t)req[5]<<8)|req[6];
                if (!ipc_trace_get(seq,words)) { resp[1]=ST_OUT_OF_RANGE; break; }
            }
            d[0]=page; d[1]=7;
            for(unsigned i=0;i<7;i++) for(unsigned b=0;b<4;b++)
                d[2+4*i+b]=(uint8_t)(words[i]>>(24-8*b));
            break;
        }
        if (page == 0xFE) {
            // Additive diagnostic page: no persistent format/version changes.
            power_state_t p; ipc_power_get(&p);
            uint64_t now=time_us_64();
            d[0]=0xFE; d[1]=26; d[2]=(uint8_t)low_power_state(); d[3]=p.phase;
            d[4]=power_veto(&p,now,LP_DEEP_IDLE_MS);
            if (p.phase==POWER_REQUESTED && p.radio.peer_connected && p.radio.peer_ack!=p.token) d[4]|=POWER_VETO_PEER;
            d[5]=(p.local.held?1:0)|(p.local.usb?2:0)|(p.radio.peer_connected?4:0)|
                 (p.radio.peer_ready?8:0)|(p.radio.peer_held?16:0)|(p.radio.held?32:0)|
                 (p.radio.pending?64:0)|(p.local.ready?128:0);
            d[6]=(uint8_t)(p.token>>8); d[7]=(uint8_t)p.token;
            uint32_t age[5]={power_age_ms(now,power_last_key(&p)),
                power_age_ms(now,p.radio.hid_us),power_age_ms(now,p.radio.peer_key_us),
                power_age_ms(now,p.radio.peer_hid_us),power_age_ms(now,p.radio.peer_sample_us)};
            for (unsigned i=0;i<5;++i) for (unsigned b=0;b<4;++b)
                d[8+4*i+b]=(uint8_t)(age[i]>>(24-8*b));
            break;
        }
        if (page == 0xFF) {
            // Live-state snapshot (debug): not counters — raw machine state.
            // d[2]=ih state, d[3]=loop-exit liveness, d[4]=slave-persistent,
            // d[5]=want_config_serve, d[6]=tick-entry counter, d[7]=heal phase,
            // d[8]=i2c1 role, d[9]=CM DS-USB state, d[10]=usb_hid_is_active.
            d[0] = 0xFF;
            d[1] = 21;          // payload bytes that follow (d[2]..d[22])
            inter_half_sync_debug_state(&d[2]);
            d[8] = (uint8_t)i2c1_link_role();
            d[9] = (uint8_t)connection_manager_ds_usb_state();
#ifdef USB_ENABLE
            d[10] = usb_hid_is_active() ? 1 : 0;
#else
            d[10] = 0xFF;
#endif
            // d[11..15] = per-task loop counters (CM, LED, PM, USB, KS) —
            // frozen byte across two reads = that task is not being scheduled.
            for (int t = 0; t < DIAG_TASK_COUNT; t++)
                d[11 + t] = g_diag_task_alive[t];
            // d[16..20] = Core 1 BLE status as published every 5 ms (R3):
            // periph state, central state, active slot, split role, valid.
            // split role: 0=STANDALONE 1=MASTER 2=SLAVE 3=MISMATCH (strict matching).
            // Lets the host tell a TRUE long-CONNECTING (gap_connect pending,
            // peer silent) from a CM/Core1 desync without UART access.
            {
                ipc_ble_status_t bst;
                ipc_get_ble_status(&bst);
                d[16] = bst.periph;
                d[17] = bst.central;
                d[18] = bst.active_slot;
                d[19] = bst.split_role;
                d[20] = bst.valid ? 1 : 0;
            }
            // d[21..22] = CM-side states (cm_upstream_state_t / cm_ds_ble_state_t)
            d[21] = (uint8_t)connection_manager_upstream_state();
            d[22] = (uint8_t)connection_manager_ds_ble_state();
            break;
        }
        uint32_t snap[DIAG_COUNTER_COUNT];
        diag_snapshot(snap);
        uint32_t first = (uint32_t)page * 7u;
        if (first >= DIAG_COUNTER_COUNT) { resp[1] = ST_OUT_OF_RANGE; break; }
        uint32_t n = DIAG_COUNTER_COUNT - first;
        if (n > 7u) n = 7u;
        d[0] = page;
        d[1] = (uint8_t)n;
        for (uint32_t i = 0; i < n; i++) {
            uint32_t v = snap[first + i];
            d[2 + i * 4 + 0] = (uint8_t)(v >> 24);
            d[2 + i * 4 + 1] = (uint8_t)(v >> 16);
            d[2 + i * 4 + 2] = (uint8_t)(v >> 8);
            d[2 + i * 4 + 3] = (uint8_t)(v);
        }
        break;
    }

    case CMD_GET_BATTERY: {
        // v5 (bug #3). Per-side read: each half answers with its OWN fuel gauge
        // (a peer-targeted request rides the I2C1 proxy). d[3]/d[4] are the
        // BLE view of the half that holds the link (0xFF elsewhere/unknown).
        pm_battery_status_t bat;
        pm_get_battery_status(&bat);
        uint8_t peer_batt, mouse_batt;
        connection_manager_get_ble_batt(&peer_batt, &mouse_batt);
        d[0] = bat.valid ? 1 : 0;
        d[1] = bat.soc_pct;
        d[2] = pm_is_charging() ? 1 : 0;
        d[3] = peer_batt;    // sibling half via CMK-Split peer_batt
        d[4] = mouse_batt;   // upstream mouse via BAS client
        break;
    }

    case CMD_GET_KEYBOARD_INFO: {
        // Phase 18: every half stores the COMPLETE unified 4×12 keymap, so the
        // addressable board is always 12 logical cols at offset 0 (0–5 left,
        // 6–11 right), regardless of whether the peer is joined. `side` reports
        // which physical half is on USB; `split_status` still exposes join state.
        bool joined = inter_half_sync_refresh_peer();
        uint16_t ksz = keymap_total_bytes();   // full 4×12 size (1152)
        d[0] = MATRIX_ROWS;
        d[1] = (uint8_t)(2 * MATRIX_COLS);     // always 12 addressable cols
        d[2] = NUM_LAYERS;
        d[3] = (uint8_t)board_side_get();      // 0=Left, 1=Right (the USB half)
        d[4] = CMK_FW_VERSION_MAJOR;
        d[5] = CMK_FW_VERSION_MINOR;
        d[6] = feature_flags();
        d[7] = (uint8_t)(ksz >> 8);
        d[8] = (uint8_t)(ksz & 0xFF);
        // split_status: bit0 = split_mode enabled, bit1 = peer joined over I2C1.
        d[9]  = (uint8_t)((eeconfig_read_split_mode() ? 0x01 : 0x00) | (joined ? 0x02 : 0x00));
        d[10] = 0;   // col_offset: full 4×12 addressable on every half
        break;
    }

    case CMD_EEPROM_RESET:
        eeconfig_factory_reset();
#ifdef DYNAMIC_KEYMAP_ENABLE
        dynamic_keymap_reset(board_side_get());
#endif
#ifdef MACRO_STORAGE_ENABLE
        macro_play_request_abort();
        macro_storage_reset();   // v5: factory reset also clears the macro region
#endif
        break;

    case CMD_COMMIT:
        // Every SET already flushes — nothing pending. OK.
        break;

    case CMD_BOOTLOADER: {
        // Require the magic token at req[2..5] (big-endian) to guard against
        // accidental/hostile reboots from a WebHID page. On match we only ARM
        // the reset; raw_hid_receive_cb performs it AFTER queuing this response.
        uint32_t token = ((uint32_t)req[2] << 24) | ((uint32_t)req[3] << 16) |
                         ((uint32_t)req[4] <<  8) |  (uint32_t)req[5];
        if (token != CMK_BOOTSEL_MAGIC) { resp[1] = ST_BAD_PARAMS; break; }
        s_bootsel_armed = true;
        break;
    }

    case CMD_GET_KEYMAP_KEYCODE: {
#ifdef DYNAMIC_KEYMAP_ENABLE
        // v3: req[2..4] = layer, row, col (GLOBAL logical 0..11). Read is local;
        // routing to the correct side is done by the wrapper before we get here.
        uint8_t layer = req[2], row = req[3], col = req[4];
        if (layer >= NUM_LAYERS || row >= MATRIX_ROWS || col >= 2 * MATRIX_COLS) {
            resp[1] = ST_OUT_OF_RANGE; break;
        }
        uint16_t kc = dynamic_keymap_get_keycode(layer, row, col);
        d[0] = (uint8_t)(kc & 0xFF); d[1] = (uint8_t)(kc >> 8);   // little-endian
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;
    }

    case CMD_SET_KEYMAP_KEYCODE: {
#ifdef DYNAMIC_KEYMAP_ENABLE
        // v3: req[2..6] = layer, row, col, kc_lo, kc_hi. LOCAL apply; the wrapper
        // already routed to this half (and/or the peer) per the target byte.
        uint8_t  layer = req[2], row = req[3], col = req[4];   // global logical col 0..11
        uint16_t kc    = (uint16_t)req[5] | ((uint16_t)req[6] << 8);  // LE
        if (layer >= NUM_LAYERS || row >= MATRIX_ROWS || col >= 2 * MATRIX_COLS) {
            resp[1] = ST_OUT_OF_RANGE; break;
        }
        if (!dynamic_keymap_set_keycode(layer, row, col, kc)) resp[1] = ST_OUT_OF_RANGE;
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;
    }

    case CMD_GET_KEYMAP_BUFFER: {
#ifdef DYNAMIC_KEYMAP_ENABLE
        uint16_t off  = ((uint16_t)req[2] << 8) | req[3];   // v3: big-endian @req[2..3]
        uint8_t  size = req[4];
        if (size > CFG_BUF_MAX_GET) { resp[1] = ST_BAD_PARAMS; break; }
        d[0] = (uint8_t)(off >> 8);
        d[1] = (uint8_t)(off & 0xFF);
        d[2] = size;
        if (!dynamic_keymap_get_buffer(off, size, &d[3])) {
            resp[1] = ST_OUT_OF_RANGE;
        }
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;
    }

    case CMD_SET_KEYMAP_BUFFER: {
#ifdef DYNAMIC_KEYMAP_ENABLE
        uint16_t off  = ((uint16_t)req[2] << 8) | req[3];   // v3: big-endian @req[2..3]
        uint8_t  size = req[4];                             // data @req[5..]
        if (size > CFG_BUF_MAX_SET) { resp[1] = ST_BAD_PARAMS; break; }
        if (!dynamic_keymap_set_buffer(off, size, &req[5])) {
            resp[1] = ST_OUT_OF_RANGE;
        }
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;
    }

    case CMD_GET_MACRO_INFO: {
#ifdef MACRO_STORAGE_ENABLE
        uint16_t msz = macro_storage_size();
        d[0] = macro_storage_count();
        d[1] = (uint8_t)(msz >> 8);      // big-endian, like the keymap buffers
        d[2] = (uint8_t)(msz & 0xFF);
        d[3] = MACRO_STORAGE_VERSION;
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;
    }

    case CMD_GET_MACRO_BUFFER: {
#ifdef MACRO_STORAGE_ENABLE
        uint16_t off  = ((uint16_t)req[2] << 8) | req[3];   // big-endian @req[2..3]
        uint8_t  size = req[4];
        if (size > CFG_BUF_MAX_GET) { resp[1] = ST_BAD_PARAMS; break; }
        d[0] = (uint8_t)(off >> 8);
        d[1] = (uint8_t)(off & 0xFF);
        d[2] = size;
        if (!macro_storage_get_buffer(off, size, &d[3])) {
            resp[1] = ST_OUT_OF_RANGE;
        }
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;
    }

    case CMD_SET_MACRO_BUFFER: {
#ifdef MACRO_STORAGE_ENABLE
        uint16_t off  = ((uint16_t)req[2] << 8) | req[3];   // big-endian @req[2..3]
        uint8_t  size = req[4];                             // data @req[5..]
        if (size > CFG_BUF_MAX_SET) { resp[1] = ST_BAD_PARAMS; break; }
        // Stop an active playback BEFORE mutating the buffer — the FSM walks
        // the RAM cache from the keyscan task (flag, served by keyscan).
        macro_play_request_abort();
        if (!macro_storage_set_buffer(off, size, &req[5])) {
            resp[1] = ST_OUT_OF_RANGE;
        }
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;
    }

    case CMD_RESET_MACROS: {
#ifdef MACRO_STORAGE_ENABLE
        macro_play_request_abort();
        macro_storage_reset();
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;
    }

    case CMD_PUSH_KEYMAP_TO_PEER: {
#ifdef DYNAMIC_KEYMAP_ENABLE
        // Force-copy the full local keymap to the peer over I2C1 (manual reconcile
        // / seed). Iterates the keymap in CFG_BUF_MAX_SET chunks, proxying a
        // SET_KEYMAP_BUFFER to the peer for each (the peer applies local-only).
        // Requires the peer joined; blocks the USB task for the transfer (~tens of
        // ms). On a chunk failure the peer may be left partially updated → retry.
        if (!inter_half_sync_peer_joined()) { resp[1] = ST_PEER_UNAVAIL; break; }
        uint16_t total = dynamic_keymap_size();
        uint8_t  chunk[CFG_BUF_MAX_SET];
        for (uint16_t off = 0; off < total; off += CFG_BUF_MAX_SET) {
            uint16_t left = (uint16_t)(total - off);
            uint8_t  n    = (left < CFG_BUF_MAX_SET) ? (uint8_t)left : CFG_BUF_MAX_SET;
            if (!dynamic_keymap_get_buffer(off, n, chunk)) { resp[1] = ST_OUT_OF_RANGE; break; }
            // v3 SET_KEYMAP_BUFFER frame: [cmd][target][off_hi][off_lo][size][data…].
            // The peer ignores the target (applies local), so any value is fine.
            uint8_t preq[32] = {0}, presp[32];
            preq[0] = CMD_SET_KEYMAP_BUFFER;
            preq[1] = cfg_peer_side();
            preq[2] = (uint8_t)(off >> 8);   // big-endian offset
            preq[3] = (uint8_t)(off & 0xFF);
            preq[4] = n;
            memcpy(&preq[5], chunk, n);
            if (!inter_half_sync_proxy_config(preq, presp)) { resp[1] = ST_PEER_UNAVAIL; break; }
            if (presp[1] != ST_OK) { resp[1] = presp[1]; break; }
        }
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;
    }

    case CMD_RESET_KEYMAP:
#ifdef DYNAMIC_KEYMAP_ENABLE
        // LOCAL reset; the wrapper handles routing to peer/both per the target.
        dynamic_keymap_reset(board_side_get());
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;

    case CMD_GET_RGB_CONFIG: {
#ifdef CMK_HAS_RGB
        // LOCAL read of this half's RGB config (resp[2..11] = struct). The wrapper
        // routes a peer-targeted GET via the I2C1 proxy before reaching here.
        resp[1] = rgb_local_get(d);
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;
    }

    case CMD_SET_RGB_CONFIG: {
#ifdef CMK_HAS_RGB
        // v3: req[2..11] = 10-byte struct. LOCAL apply (wrapper handles routing).
        resp[1] = rgb_local_set_config(&req[2]);
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;
    }

    case CMD_SET_RGB_FIELD: {
#ifdef CMK_HAS_RGB
        // v3: req[2] = field (0..9), req[3] = value. LOCAL apply.
        resp[1] = rgb_local_set_field(req[2], req[3]);
#else
        resp[1] = ST_NOT_SUPPORTED;
#endif
        break;
    }

    case CMD_GET_SETTINGS: {
        uint16_t flags = eeconfig_read_keymap_flags();
        d[0] = eeconfig_read_default_layer();
        d[1] = (uint8_t)(flags >> 8);     // big-endian
        d[2] = (uint8_t)(flags & 0xFF);
        d[3] = eeconfig_read_auto_shift();
        d[4] = eeconfig_read_unicode_mode();
        d[5] = eeconfig_read_output_target();
        d[6] = eeconfig_read_split_mode();
        // v5 (bug #2): LP timeouts in seconds, LE16 — same byte order as the
        // SET_SETTING value, so the UI reads back exactly what it wrote.
        uint16_t lp_light = eeconfig_read_lp_light_s();
        uint16_t lp_deep  = eeconfig_read_lp_deep_s();
        d[7]  = (uint8_t)(lp_light & 0xFF);
        d[8]  = (uint8_t)(lp_light >> 8);
        d[9]  = (uint8_t)(lp_deep & 0xFF);
        d[10] = (uint8_t)(lp_deep >> 8);
        // v5 (bug #8): EFFECTIVE tapping term ms, LE16 (never 0 — the getter
        // resolves 0 → factory default and clamps)
        uint16_t tt = tapping_term_get();
        d[11] = (uint8_t)(tt & 0xFF);
        d[12] = (uint8_t)(tt >> 8);
        break;
    }

    case CMD_SET_SETTING: {
        uint8_t  id    = req[2];                                      // v3: @req[2]
        uint16_t value = (uint16_t)req[3] | ((uint16_t)req[4] << 8);  // lo, hi @req[3..4]
        switch (id) {
            case 0x00: eeconfig_update_default_layer((uint8_t)value); break;
            case 0x01: eeconfig_update_keymap_flags(value);           break;
            case 0x02: eeconfig_update_auto_shift((uint8_t)value);    break;
            case 0x03: eeconfig_update_unicode_mode((uint8_t)value);  break;
            case 0x04: eeconfig_update_output_target((uint8_t)value); break;
            case 0x05: eeconfig_update_split_mode((uint8_t)value);    break;
            // v5 (bug #2): LP timeouts in seconds, 0 = tier disabled. Refresh so
            // the change takes effect without a reboot.
            case 0x06: eeconfig_update_lp_light_s(value);
                       low_power_refresh_config();                    break;
            case 0x07: eeconfig_update_lp_deep_s(value);
                       low_power_refresh_config();                    break;
            // v5 (bug #8): tapping term ms (LE16). 0 = back to the factory
            // default; silent firmware-side clamp (like the other ids).
            // Immediate effect: the getter reads eeconfig live.
            case 0x08:
                if (value != 0) {
                    if (value < TAPPING_TERM_MIN) value = TAPPING_TERM_MIN;
                    if (value > TAPPING_TERM_MAX) value = TAPPING_TERM_MAX;
                }
                eeconfig_update_tapping_term(value);
                break;
        }
        break;
    }

    default:
        resp[1] = ST_UNKNOWN_CMD;
        break;
    }
}

// Fill a bare status response (echo command, no data).
static void resp_status(uint8_t *resp, uint8_t cmd, uint8_t status) {
    memset(resp, 0, RAW_HID_BUFFER_SIZE);
    resp[0] = cmd;
    resp[1] = status;
}

// USB-side entry (protocol v3): parse req[1]=target and route to the local half,
// the peer half (I2C1 proxy), or both (peer-first-then-local). System commands
// ignore the target and run locally. The peer applies its leg LOCAL-ONLY (it is
// served via cmk_config_protocol_handle_peer → cfg_dispatch), so no loop.
void cmk_config_protocol_handle(const uint8_t *req, uint8_t *resp) {
    uint8_t cmd    = req[0];
    uint8_t target = req[1];

    // Target-irrelevant commands run on the USB half directly.
    if (cmd_is_system(cmd)) { cfg_dispatch(req, resp); return; }

    if (target > CFG_TGT_BOTH) { resp_status(resp, cmd, ST_BAD_PARAMS); return; }

    bool is_read = cmd_is_read(cmd);
    if (is_read && target == CFG_TGT_BOTH) {     // one response can't carry two sides
        resp_status(resp, cmd, ST_BAD_PARAMS); return;
    }

    // Local-leg secure-lock guard: reject a write that would touch this half BEFORE
    // proxying, so a locked local half can't leave the peer written (de-sync).
    if (!is_read && need_local(target) && secure_is_locked()) {
        resp_status(resp, cmd, ST_LOCKED); return;
    }

    // Peer leg first (peer-first-then-local for writes; the only side for a peer GET).
    bool degraded_local_only = false;
    if (need_peer(target)) {
        if (inter_half_sync_peer_joined()) {
            uint8_t presp[RAW_HID_BUFFER_SIZE];
            if (!inter_half_sync_proxy_config(req, presp)) {
                resp_status(resp, cmd, ST_PEER_UNAVAIL); return;
            }
            if (is_read) {                       // peer-targeted read → return its data
                memcpy(resp, presp, RAW_HID_BUFFER_SIZE); return;
            }
            if (presp[1] != ST_OK) {             // write: peer rejected → abort, no local
                memcpy(resp, presp, RAW_HID_BUFFER_SIZE); return;
            }
            // write peer OK → fall through to the local leg if BOTH
        } else if (target != CFG_TGT_BOTH) {     // peer explicitly targeted but absent
            resp_status(resp, cmd, ST_PEER_UNAVAIL); return;
        } else {
            // (BOTH && peer absent) → local-only, but SAY so (R6): a silent
            // ST_OK made the configurator believe both halves were written —
            // an invisible L/R de-sync.
            degraded_local_only = true;
        }
    }

    if (need_local(target)) {
        cfg_dispatch(req, resp);                 // local leg
        if (degraded_local_only && resp[1] == ST_OK) resp[1] = ST_OK_LOCAL_ONLY;
    } else {
        resp_status(resp, cmd, ST_OK);           // peer-only write already succeeded
    }
}

// Peer-side entry: a command proxied FROM the other half over I2C1. Runs the LOCAL
// handler directly (no routing) so a BOTH/peer-targeted request resolves to one
// local apply and never loops. Called by inter_half_sync as the I2C1 slave.
void cmk_config_protocol_handle_peer(const uint8_t *req, uint8_t *resp) {
    cfg_dispatch(req, resp);
}

// ---------------------------------------------------------------------------
// Raw HID glue — overrides the weak stub in raw_hid.c. Runs in the Core 0 USB
// task (synchronous within tud_task). One command in flight at a time.
// ---------------------------------------------------------------------------

void raw_hid_receive_cb(const uint8_t *data, uint16_t length) {
    if (length < 1) return;

    uint8_t resp[RAW_HID_BUFFER_SIZE];
    cmk_config_protocol_handle(data, resp);
    raw_hid_send(resp);

    // BOOTLOADER: reset into BOOTSEL only after the response has been queued,
    // and only if a valid magic token armed it (see cfg_dispatch). mask=0 keeps
    // BOTH the MSC and the PICOBOOT vendor interface up — PICOBOOT is what the
    // WebUSB-based "Update FW" flow flashes through.
    if (s_bootsel_armed) {
        s_bootsel_armed = false;
        reset_usb_boot(0, 0);
    }
}

#endif // RAW_HID_ENABLE
