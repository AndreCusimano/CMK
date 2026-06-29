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
// inter_half_sync.c — see inter_half_sync.h for the design rationale.
//

#include "inter_half_sync.h"

#include "config/cmk_config.h"
#include "config/board.h"
#include "config/debug_config.h"

#ifdef INTER_HALF_LINK_ENABLE

#include "i2c1_link.h"
#include "crc16.h"
#include "diag.h"
#include "eeconfig.h"
#include "process_secure.h"
#include "connection_manager.h"
#include "low_power.h"            // throttle the link in LIGHT/DEEP (tickless)

#include "FreeRTOS.h"
#include "task.h"

#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
#include "pico/i2c_slave.h"
#include "pico/time.h"

#include <string.h>

#ifdef RAW_HID_ENABLE
#include "cmk_config_protocol.h"
#endif

// ---------------------------------------------------------------------------
// Push transfer geometry — eeconfig only (the keymap is per-side; it is edited
// via the config proxy, not cloned). Blob = the 32-byte eeconfig record.
// ---------------------------------------------------------------------------

#define IH_BLOB_BYTES   32

// Push frame types (every frame ends with a CRC-16 of its own bytes).
#define FR_START   0x01   // [0x01][len_lo][len_hi][0] + crc16   (6 B)
#define FR_DATA    0x02   // [0x02][seq][n][payload n] + crc16
#define FR_END     0x03   // [0x03][blob_crc_lo][blob_crc_hi] + crc16 (5 B)
#define FR_CFG_REQ 0x10   // [0x10][req 32B] + crc16  (35 B) — config proxy

// Push status byte (1-byte read after each push frame).
#define ST_OK      0x00
#define ST_CRC     0x01   // frame CRC mismatch → master retries this frame
#define ST_LOCKED  0x02   // receiver in Secure lock → abort
#define ST_BUSY    0x03   // not armed / unexpected frame → abort
#define ST_SEQ     0x04   // out-of-order / overflow → abort

// Config-proxy response framing: [status][resp 32B][crc16] (35 B read).
#define CFG_READY  0xA5
#define CFG_BUSY   0x5A
#define IH_CFG_REQ_FRAME_LEN 35   // 1 + 32 + 2
#define IH_CFG_RESP_LEN      35   // 1 + 32 + 2

#define IH_FRAME_MAX (3 + IH_CHUNK_BYTES + 2)   // largest push frame we build
#define IH_RX_MAX    40                          // largest frame we receive (cfg req = 35)

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

typedef enum {
    S_IDLE = 0,
    S_SLAVE,         // slave: push-receive (armed, timed) or config-serve (persistent)
    S_MASTER_START,  // master push: send START
    S_MASTER_DATA,   // master push: stream DATA
    S_MASTER_END,    // master push: send END
    S_PROXY_BUSY,    // master proxy in progress (USB task owns I2C1 — tick stays clear)
} ih_state_t;

static volatile ih_state_t s_state = S_IDLE;
static ih_led_state_t      s_led   = IH_LED_OFF;

// Shared blob buffer (one role at a time): master fills it (export) before a
// push; slave stages the received eeconfig into it.
static uint8_t  s_buf[IH_BLOB_BYTES];

// Master push progress
static uint16_t s_off;
static uint8_t  s_seq;
static uint8_t  s_retry;

// Slave receive state
static uint8_t           s_rx[IH_RX_MAX];
static volatile uint16_t s_rx_len;
static volatile bool     s_rx_overflow;
static volatile bool     s_active;          // push START accepted, mid-transfer
static volatile bool     s_apply_pending;   // push END validated → tick applies
static uint16_t          s_staging_off;
static uint8_t           s_exp_seq;
static bool              s_slave_persistent; // true = config-serve, false = armed push
static uint32_t          s_arm_at;

// Slave TX cursor (REQUEST serves s_tx_buf[0..s_tx_len)).
static volatile uint8_t  s_tx_buf[IH_CFG_RESP_LEN];
static volatile uint16_t s_tx_len;
static volatile uint16_t s_tx_pos;

// Config-serve (slave) request/response
#ifdef RAW_HID_ENABLE
static uint8_t           s_cfg_req[32];
static volatile bool     s_cfg_req_pending;
static volatile bool     s_cfg_resp_ready;
// Self-heal: a slave transaction (any FINISH) bumps s_slave_evt from the ISR;
// the tick re-arms the peripheral if it stays unchanged past s_heal_deadline.
static volatile uint32_t s_slave_evt;
static uint32_t          s_slave_evt_seen;
static uint32_t          s_heal_deadline;
#endif

// Config-proxy (master) cached join state
static bool s_peer_joined;

// Deferred reboot after a successful push receive-apply.
static bool     s_reboot_pending;
static uint32_t s_reboot_at;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

#ifdef RAW_HID_ENABLE
// This half currently has a USB host (it is the configurator side).
static bool this_half_has_usb(void) {
    return connection_manager_ds_usb_state() == CM_DS_USB_CONNECTED;
}

// Should this half sit as a persistent config-serve slave on I2C1?
// Independent of split_mode (Phase 20b decision): a half without a USB host
// listens as I2C1 slave so the USB half (master) can query/configure it. The
// half with USB stays idle and becomes master on-demand.
static bool want_config_serve(void) {
    return !this_half_has_usb();
}
#endif

// Set the push status served on the next REQUEST. R6: the response buffer is
// ALWAYS a full IH_CFG_RESP_LEN frame (status + 0xFF padding) — a master that
// reads 35 bytes (config poll) against a 1-byte status used to empty the TX
// FIFO and the DW_apb_i2c then CLOCK-STRETCHED SCL until the self-heal (1.5 s),
// turning one corrupted frame into a wedged bus and ERR_PEER_UNAVAIL.
static void set_push_status(uint8_t st) {
    s_tx_buf[0] = st;
    for (uint16_t i = 1; i < IH_CFG_RESP_LEN; i++) s_tx_buf[i] = 0xFF;
    s_tx_len = IH_CFG_RESP_LEN;
}

// ---------------------------------------------------------------------------
// Slave ISR — interrupt context. Light only: byte buffering, small CRC, status
// staging. NO printf, NO EEPROM, NO blocking I2C, NO protocol handler here.
// ---------------------------------------------------------------------------

static void slave_process_frame(void) {
    // DIAG_INC from ISR is fine: plain ++ on a volatile uint32, single writer core.
    if (s_rx_overflow) { s_rx_overflow = false; DIAG_INC(ih_crc_fail); set_push_status(ST_CRC); return; }

    uint16_t len = s_rx_len;
    if (len < 3) { DIAG_INC(ih_crc_fail); set_push_status(ST_CRC); return; }

    uint16_t rx_crc = (uint16_t)s_rx[len - 2] | ((uint16_t)s_rx[len - 1] << 8);
    if (crc16_ccitt(s_rx, len - 2) != rx_crc) { DIAG_INC(ih_crc_fail); set_push_status(ST_CRC); return; }

    uint8_t type = s_rx[0];

#ifdef RAW_HID_ENABLE
    if (type == FR_CFG_REQ) {
        if (len != IH_CFG_REQ_FRAME_LEN) { set_push_status(ST_CRC); return; }
        memcpy(s_cfg_req, &s_rx[1], 32);
        s_cfg_req_pending = true;
        s_cfg_resp_ready  = false;
        // Prime a BUSY response; the tick fills the READY one.
        s_tx_buf[0] = CFG_BUSY;
        for (uint16_t i = 1; i < IH_CFG_RESP_LEN; i++) s_tx_buf[i] = 0;
        s_tx_len = IH_CFG_RESP_LEN;
        return;
    }
#endif

    // --- Push frames ---
    // R6: while a completed transfer waits for the task-context apply, s_buf
    // must not be re-staged by a new push — the apply would import a torn blob.
    if (s_apply_pending) { set_push_status(ST_BUSY); return; }
    if (type == FR_START) {
        if (secure_is_locked()) { set_push_status(ST_LOCKED); s_active = false; return; }
        uint16_t total = (uint16_t)s_rx[1] | ((uint16_t)s_rx[2] << 8);
        if (total != IH_BLOB_BYTES) { set_push_status(ST_SEQ); s_active = false; return; }
        s_staging_off   = 0;
        s_exp_seq       = 0;
        s_active        = true;
        s_apply_pending = false;
        set_push_status(ST_OK);
        return;
    }
    if (!s_active) { set_push_status(ST_BUSY); return; }
    if (type == FR_DATA) {
        uint8_t seq = s_rx[1];
        uint8_t n   = s_rx[2];
        if ((uint16_t)(3 + n) != len - 2) { set_push_status(ST_CRC); return; }
        if (seq != s_exp_seq) { set_push_status(ST_SEQ); s_active = false; return; }
        if (s_staging_off + n > IH_BLOB_BYTES) { set_push_status(ST_SEQ); s_active = false; return; }
        memcpy(s_buf + s_staging_off, &s_rx[3], n);
        s_staging_off = (uint16_t)(s_staging_off + n);
        s_exp_seq++;
        set_push_status(ST_OK);
        return;
    }
    if (type == FR_END) {
        uint16_t bcrc = (uint16_t)s_rx[1] | ((uint16_t)s_rx[2] << 8);
        if (s_staging_off != IH_BLOB_BYTES) { set_push_status(ST_SEQ); s_active = false; return; }
        if (crc16_ccitt(s_buf, IH_BLOB_BYTES) != bcrc) { set_push_status(ST_CRC); s_active = false; return; }
        s_apply_pending = true;
        s_active        = false;
        set_push_status(ST_OK);
        return;
    }
    set_push_status(ST_CRC);
}

static void slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE:
        while (i2c_get_read_available(i2c)) {
            uint8_t b = i2c_read_byte_raw(i2c);
            if (s_rx_len < sizeof(s_rx)) {
                s_rx[s_rx_len++] = b;
            } else {
                s_rx_overflow = true;
            }
        }
        break;
    case I2C_SLAVE_REQUEST:
        // Serve the response from the cursor; past the end feed 0xFF padding.
        // R6: NEVER leave the TX FIFO empty on a pending read — an empty FIFO
        // makes the DW_apb_i2c clock-stretch SCL indefinitely (bus wedge).
        while (i2c_get_write_available(i2c)) {
            if (s_tx_pos < s_tx_len) i2c_write_byte_raw(i2c, s_tx_buf[s_tx_pos++]);
            else                     i2c_write_byte_raw(i2c, 0xFF);
        }
        break;
    case I2C_SLAVE_FINISH:
        s_tx_pos = 0;
#ifdef RAW_HID_ENABLE
        s_slave_evt++;          // mark slave activity (for the self-heal timer)
#endif
        if (s_rx_len > 0) {
            slave_process_frame();
            s_rx_len = 0;
        }
        break;
    }
}

// ---------------------------------------------------------------------------
// Master push helpers (task context)
// ---------------------------------------------------------------------------

static int master_txn(const uint8_t *frame, size_t flen, uint8_t *status) {
    int w = i2c1_link_write(CMK_I2C1_ADDR, frame, flen, IH_TXN_TIMEOUT_US);
    if (w < 0 || (size_t)w != flen) return -1;
    uint8_t st = 0;
    int r = i2c1_link_read(CMK_I2C1_ADDR, &st, 1, IH_TXN_TIMEOUT_US);
    if (r != 1) return -1;
    *status = st;
    return 0;
}

static void master_fail(void) {
    i2c1_link_idle();
    s_state = S_IDLE;
    s_led   = IH_LED_ERR;
    DBG_IH("[IH] push aborted\n");
}

static void receive_apply(uint32_t now) {
    bool ok = false;
    if (!secure_is_locked()) {
        ok = eeconfig_import(s_buf);   // 32-byte eeconfig record
    }
    s_apply_pending = false;
    i2c1_link_idle();
    s_state           = S_IDLE;
    s_slave_persistent = false;

    if (ok) {
        s_led            = IH_LED_OK;
        s_reboot_pending = true;
        s_reboot_at      = now + pdMS_TO_TICKS(IH_APPLY_REBOOT_MS);
        DBG_IH("[IH] eeconfig received → apply ok, reboot in %d ms\n", IH_APPLY_REBOOT_MS);
    } else {
        s_led = IH_LED_ERR;
        DBG_IH("[IH] eeconfig received → apply REJECTED (locked/version)\n");
    }
}

// ---------------------------------------------------------------------------
// Config proxy (master, called synchronously from the USB task)
// ---------------------------------------------------------------------------

#ifdef RAW_HID_ENABLE
// One full proxy transaction: write the request frame, then poll for the peer's
// response. Returns true only on a complete success (CFG_READY + valid CRC);
// any bus error / wrong length / bad CRC returns false so the caller can retry.
static bool proxy_txn_once(const uint8_t *f, uint8_t resp[32]) {
    if (i2c1_link_write(CMK_I2C1_ADDR, f, IH_CFG_REQ_FRAME_LEN, IH_TXN_TIMEOUT_US) != IH_CFG_REQ_FRAME_LEN) {
        return false;                                   // request not delivered
    }
    for (int i = 0; i < IH_PROXY_MAX_POLLS; i++) {
        // R6: yield instead of busy-waiting — this runs in the USB task; a
        // busy_wait starved everything at the same priority for the whole poll.
        vTaskDelay(pdMS_TO_TICKS(2));     // let the peer's ih task process the request
        uint8_t r[IH_CFG_RESP_LEN];
        int got = i2c1_link_read(CMK_I2C1_ADDR, r, IH_CFG_RESP_LEN, IH_TXN_TIMEOUT_US);
        if (got != IH_CFG_RESP_LEN) return false;       // bus error / peer gone
        if (r[0] == CFG_READY) {
            uint16_t rc = (uint16_t)r[33] | ((uint16_t)r[34] << 8);
            if (crc16_ccitt(r, 33) != rc) return false; // corrupted response
            // R6: command echo check — resp[0] echoes the request command id;
            // a stale response (from a previous request that we gave up on)
            // must not be taken for this one.
            if (r[1] != f[1]) return false;
            memcpy(resp, &r[1], 32);
            return true;
        }
        // CFG_BUSY → keep polling
    }
    return false;                                       // peer never produced a response
}

bool inter_half_sync_proxy_config(const uint8_t req[32], uint8_t resp[32]) {
    // R6: atomic test-and-set — the USB task (proxy) and the ih task (push/serve
    // entries) race on s_state; two owners of the I2C1 peripheral corrupt it.
    taskENTER_CRITICAL();
    if (s_state != S_IDLE) { taskEXIT_CRITICAL(); return false; }
    s_state = S_PROXY_BUSY;
    taskEXIT_CRITICAL();
    i2c1_link_enter_master();

    uint8_t f[IH_CFG_REQ_FRAME_LEN];
    f[0] = FR_CFG_REQ;
    memcpy(&f[1], req, 32);
    uint16_t c = crc16_ccitt(f, 33);
    f[33] = (uint8_t)(c & 0xFF);
    f[34] = (uint8_t)(c >> 8);

    // Retry the whole transaction: a brief disconnect / bus glitch must not drop
    // the link on the first error. Re-init the peripheral between attempts AND
    // run the GPIO bus recovery (R6): re-init alone can't release an SDA line
    // held low by a peer wedged mid-bit. Proxied commands are register-style →
    // idempotent, so re-sending the request is safe.
    bool ok = false;
    for (int attempt = 0; attempt < IH_PROXY_TXN_ATTEMPTS && !ok; attempt++) {
        if (attempt > 0) {
            DIAG_INC(ih_proxy_retry);
            i2c1_link_idle();
            i2c1_link_bus_recover();
            vTaskDelay(pdMS_TO_TICKS(1));
            i2c1_link_enter_master();
            DBG_IH("[IH] proxy retry %d/%d\n", attempt, IH_PROXY_TXN_ATTEMPTS - 1);
        }
        ok = proxy_txn_once(f, resp);
    }

    i2c1_link_idle();
    s_state = S_IDLE;
    if (!ok) {                        // only after ALL attempts failed
        DIAG_INC(ih_proxy_fail);
        s_peer_joined = false;
    }
    return ok;
}

bool inter_half_sync_refresh_peer(void) {
    uint8_t req[32];
    memset(req, 0, sizeof(req));
    req[0] = CMD_GET_PROTOCOL_VERSION;   // lightweight ping
    uint8_t resp[32];
    s_peer_joined = inter_half_sync_proxy_config(req, resp);
    return s_peer_joined;
}

bool inter_half_sync_peer_joined(void) {
    return s_peer_joined;
}
#else  // no RAW_HID → no config proxy
bool inter_half_sync_proxy_config(const uint8_t req[32], uint8_t resp[32]) { (void)req; (void)resp; return false; }
bool inter_half_sync_refresh_peer(void) { return false; }
bool inter_half_sync_peer_joined(void)  { return false; }
#endif

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void inter_half_sync_init(void) {
    i2c1_link_init();
    s_state          = S_IDLE;
    s_led            = IH_LED_OFF;
    s_apply_pending  = false;
    s_reboot_pending = false;
    s_peer_joined    = false;
    set_push_status(ST_BUSY);
    s_tx_pos = 0;
}

// R6: atomically claim the link for a user-initiated push/arm. In wireless use
// BOTH halves sit in persistent config-serve (no USB anywhere) and the old
// `s_state != S_IDLE → return` made CMK_CFG_PUSH/RECV silent no-ops — the
// "clone config without a PC" feature was simply dead. A quiescent config-serve
// slave can now be taken over.
static bool ih_claim_for_user_op(void) {
    taskENTER_CRITICAL();
    bool from_serve = (s_state == S_SLAVE && s_slave_persistent &&
                       !s_active && !s_apply_pending);
    bool ok = from_serve || (s_state == S_IDLE);
    if (ok) s_state = S_PROXY_BUSY;   // temporary claim (no I2C yet)
    taskEXIT_CRITICAL();
    if (!ok) return false;
    if (from_serve) {
        i2c1_link_idle();             // release the slave peripheral
        s_slave_persistent = false;
    }
    return true;
}

void inter_half_sync_request_push(void) {
    if (!ih_claim_for_user_op()) return;

    eeconfig_export(s_buf);   // 32-byte eeconfig record

    i2c1_link_enter_master();
    s_off   = 0;
    s_seq   = 0;
    s_retry = 0;
    s_state = S_MASTER_START;
    s_led   = IH_LED_ACTIVE;
    DBG_IH("[IH] push requested (%d bytes eeconfig)\n", IH_BLOB_BYTES);
}

void inter_half_sync_arm_receive(void) {
    if (!ih_claim_for_user_op()) return;

    s_rx_len        = 0;
    s_rx_overflow   = false;
    s_active        = false;
    s_apply_pending = false;
    s_staging_off   = 0;
    s_exp_seq       = 0;
    set_push_status(ST_BUSY);
    s_tx_pos = 0;

    i2c1_link_enter_slave(CMK_I2C1_ADDR, slave_handler);
    s_slave_persistent = false;          // explicit one-shot push receive (timed)
    s_arm_at           = xTaskGetTickCount();
    s_state            = S_SLAVE;
    s_led              = IH_LED_ARMED;
    DBG_IH("[IH] armed for push receive (addr 0x%02x)\n", CMK_I2C1_ADDR);
}

ih_led_state_t inter_half_sync_led_state(void) {
    return s_led;
}

#ifdef RAW_HID_ENABLE
static void enter_config_serve(void) {
    // R6: atomic claim vs the proxy in the USB task.
    taskENTER_CRITICAL();
    if (s_state != S_IDLE) { taskEXIT_CRITICAL(); return; }
    s_state = S_PROXY_BUSY;   // temporary claim (released to S_SLAVE below)
    taskEXIT_CRITICAL();

    s_rx_len        = 0;
    s_rx_overflow   = false;
    s_active        = false;
    s_apply_pending = false;
    s_staging_off   = 0;
    s_exp_seq       = 0;
    s_cfg_req_pending = false;
    s_cfg_resp_ready  = false;
    set_push_status(ST_BUSY);
    s_tx_pos = 0;

    i2c1_link_enter_slave(CMK_I2C1_ADDR, slave_handler);
    s_slave_persistent = true;           // persistent config-serve
    s_state            = S_SLAVE;
    s_led              = IH_LED_ARMED;
    s_slave_evt_seen   = s_slave_evt;    // arm the self-heal timer (deadline set on next tick)
    s_heal_deadline    = 0;
    DBG_IH("[IH] config-serve slave (no USB)\n");
}
#endif

// ---------------------------------------------------------------------------
// Tick — non-blocking state machine (keyscan task).
// ---------------------------------------------------------------------------

// Debug breadcrumbs (GET_DIAG page 0xFF): tick-entry counter + last phase
// reached inside the heal sequence (1=pre-idle, 2=pre-enter-slave, 3=done).
static volatile uint8_t s_ih_tick_enter;
static volatile uint8_t s_ih_heal_phase;

void inter_half_sync_tick(uint32_t now) {
    s_ih_tick_enter++;
#ifdef RAW_HID_ENABLE
    // R6: role hysteresis — usb_hid_is_active() flaps on USB suspend/resume;
    // switching slave↔idle on every flap can truncate a peer transaction in
    // flight and opens role races. Require the desired role to be stable.
    static bool     s_want_last   = false;
    static uint32_t s_want_since  = 0;
    bool want = want_config_serve();
    if (want != s_want_last) { s_want_last = want; s_want_since = now; }
    bool want_stable = (uint32_t)(now - s_want_since) >= pdMS_TO_TICKS(IH_ROLE_HYST_MS);

    // Auto-enter config-serve when this half has no USB (stable).
    if (s_state == S_IDLE && want && want_stable) {
        enter_config_serve();
    }
#endif

    switch (s_state) {
    case S_IDLE:
    case S_PROXY_BUSY:
        break;

    case S_SLAVE:
        if (s_apply_pending) { receive_apply(now); break; }
#ifdef RAW_HID_ENABLE
        if (s_cfg_req_pending) {
            // R6: snapshot the request with the ISR masked — a master retry can
            // rewrite s_cfg_req mid-handling (torn request).
            uint8_t req_copy[32];
            irq_set_enabled(I2C1_IRQ, false);
            memcpy(req_copy, s_cfg_req, 32);
            irq_set_enabled(I2C1_IRQ, true);

            uint8_t resp[32];
            // Peer-side handler: applies keymap writes LOCAL-ONLY so the shared-
            // keymap mirror (proxy-first on the USB half) cannot loop back here.
            cmk_config_protocol_handle_peer(req_copy, resp);    // task context — EEPROM ok
            uint8_t out[IH_CFG_RESP_LEN];
            out[0] = CFG_READY;
            memcpy(&out[1], resp, 32);
            uint16_t c = crc16_ccitt(out, 33);
            out[33] = (uint8_t)(c & 0xFF);
            out[34] = (uint8_t)(c >> 8);
            // Publish atomically vs the I2C1 ISR.
            irq_set_enabled(I2C1_IRQ, false);
            memcpy((void *)s_tx_buf, out, IH_CFG_RESP_LEN);
            s_tx_len         = IH_CFG_RESP_LEN;
            s_cfg_resp_ready = true;
            s_cfg_req_pending = false;
            irq_set_enabled(I2C1_IRQ, true);
        }
        if (s_slave_persistent) {
            if (!want && want_stable) {     // USB appeared (stable) → release the bus
                i2c1_link_idle();
                s_state = S_IDLE;
                s_led   = IH_LED_OFF;
                break;
            }

            // R6: a push interrupted mid-transfer (connector detach after a
            // valid FR_START) left s_active=true FOREVER in config-serve mode —
            // and s_active suppresses the self-heal below, resurrecting the
            // "slave dead until reboot" wedge. Time the active window out.
            static uint32_t s_active_since = 0;
            if (s_active) {
                if (s_active_since == 0) s_active_since = now;
                else if ((uint32_t)(now - s_active_since) >= pdMS_TO_TICKS(IH_PUSH_ACTIVE_TIMEOUT_MS)) {
                    s_active        = false;
                    s_active_since  = 0;
                    s_staging_off   = 0;
                    s_exp_seq       = 0;
                    set_push_status(ST_BUSY);
                    DBG_IH("[IH] push receive timed out mid-transfer → reset\n");
                }
            } else {
                s_active_since = 0;
            }
            // Self-heal: re-arm the slave peripheral if no transaction has
            // completed within IH_SLAVE_HEAL_MS. Recovers a slave left stuck by
            // a mid-transaction connector disconnect (it stops ACKing otherwise,
            // hence "not recognized until reboot"). Skipped while a push is in
            // flight so it never interrupts an active transfer.
            if (s_slave_evt != s_slave_evt_seen) {
                s_slave_evt_seen = s_slave_evt;
                s_heal_deadline  = now + pdMS_TO_TICKS(IH_SLAVE_HEAL_MS);
            } else if (!s_active && !s_apply_pending &&
                       (int32_t)(now - s_heal_deadline) >= 0) {
                DIAG_INC(ih_heal);
                s_ih_heal_phase = 1;
                i2c1_link_idle();
                s_ih_heal_phase = 2;
                i2c1_link_enter_slave(CMK_I2C1_ADDR, slave_handler);
                s_ih_heal_phase = 3;
                s_rx_len = 0; s_rx_overflow = false; s_tx_pos = 0;
                s_cfg_req_pending = false; s_cfg_resp_ready = false;
                set_push_status(ST_BUSY);
                s_slave_evt_seen = s_slave_evt;
                s_heal_deadline  = now + pdMS_TO_TICKS(IH_SLAVE_HEAL_MS);
                DBG_IH("[IH] slave re-armed (heal: idle/disconnect)\n");
            }
            break;
        }
#endif
        // Armed (one-shot push receive): disarm on timeout — but never tear
        // down a push that is mid-transfer (R6: it raced the arm window edge).
        if (!s_slave_persistent && !s_active &&
            (uint32_t)(now - s_arm_at) >= pdMS_TO_TICKS(IH_ARM_TIMEOUT_MS)) {
            i2c1_link_idle();
            s_state = S_IDLE;
            s_led   = IH_LED_OFF;
            DBG_IH("[IH] arm window expired → idle\n");
        }
        break;

    case S_MASTER_START: {
        uint8_t f[6];
        f[0] = FR_START;
        f[1] = (uint8_t)(IH_BLOB_BYTES & 0xFF);
        f[2] = (uint8_t)(IH_BLOB_BYTES >> 8);
        f[3] = 0;
        uint16_t c = crc16_ccitt(f, 4);
        f[4] = (uint8_t)(c & 0xFF);
        f[5] = (uint8_t)(c >> 8);
        uint8_t st;
        if (master_txn(f, sizeof(f), &st) < 0) { master_fail(); break; }
        if (st == ST_OK) { s_off = 0; s_seq = 0; s_retry = 0; s_state = S_MASTER_DATA; }
        else { DBG_IH("[IH] START rejected (0x%02x)\n", st); master_fail(); }
        break;
    }

    case S_MASTER_DATA: {
        if (s_off >= IH_BLOB_BYTES) { s_state = S_MASTER_END; s_retry = 0; break; }
        uint8_t n = (uint8_t)((IH_BLOB_BYTES - s_off) < IH_CHUNK_BYTES
                                  ? (IH_BLOB_BYTES - s_off) : IH_CHUNK_BYTES);
        uint8_t f[IH_FRAME_MAX];
        f[0] = FR_DATA;
        f[1] = s_seq;
        f[2] = n;
        memcpy(&f[3], s_buf + s_off, n);
        uint16_t c = crc16_ccitt(f, (size_t)(3 + n));
        f[3 + n]     = (uint8_t)(c & 0xFF);
        f[3 + n + 1] = (uint8_t)(c >> 8);
        uint8_t st;
        if (master_txn(f, (size_t)(3 + n + 2), &st) < 0) { master_fail(); break; }
        if (st == ST_OK) { s_off = (uint16_t)(s_off + n); s_seq++; s_retry = 0; }
        else if (st == ST_CRC && s_retry < IH_MAX_RETRY) { s_retry++; }
        else { DBG_IH("[IH] DATA seq %d rejected (0x%02x)\n", s_seq, st); master_fail(); }
        break;
    }

    case S_MASTER_END: {
        uint8_t f[5];
        f[0] = FR_END;
        uint16_t bc = crc16_ccitt(s_buf, IH_BLOB_BYTES);
        f[1] = (uint8_t)(bc & 0xFF);
        f[2] = (uint8_t)(bc >> 8);
        uint16_t c = crc16_ccitt(f, 3);
        f[3] = (uint8_t)(c & 0xFF);
        f[4] = (uint8_t)(c >> 8);
        uint8_t st;
        if (master_txn(f, sizeof(f), &st) < 0) { master_fail(); break; }
        if (st == ST_OK) {
            i2c1_link_idle();
            s_state = S_IDLE;
            s_led   = IH_LED_OK;
            DBG_IH("[IH] push complete\n");
        } else if (st == ST_CRC && s_retry < IH_MAX_RETRY) { s_retry++; }
        else { DBG_IH("[IH] END rejected (0x%02x)\n", st); master_fail(); }
        break;
    }
    }

    // Deferred reboot after a successful receive-apply.
    if (s_reboot_pending && (int32_t)(now - s_reboot_at) >= 0) {
        watchdog_reboot(0, 0, 0);
    }
}

// ---------------------------------------------------------------------------
// Dedicated task (R6). The tick used to run inside the keyscan task: the slave
// leg executes cmk_config_protocol_handle_peer → EEPROM flushes (tens of ms)
// which FROZE the 1 kHz matrix scan for every proxied SET chunk (lost presses /
// delayed releases on the slave half during configuration). Priority 1 — below
// keyscan and CM — so link work only runs in otherwise-idle time.
// ---------------------------------------------------------------------------

// Liveness counter for the debug snapshot — advances only while ih_task runs.
static volatile uint32_t s_ih_loops;

static void ih_task(void *arg) {
    (void)arg;
    for (;;) {
        // Low-power throttle: at 1 ms this task would wake Core 0 every tick and
        // defeat tickless idle. The I2C1 link only carries config transfer (the
        // split's keystrokes go over BLE), so it can poll far slower when idle.
        // In DEEP the link is quiesced entirely (no bus activity → no spurious
        // edges on the cross-half wake line); it resumes on wake (state→ACTIVE).
        lp_state_t lp = low_power_state();
        if (lp != LP_DEEP) {
            inter_half_sync_tick((uint32_t)xTaskGetTickCount());
        }
        s_ih_loops++;
        uint32_t delay_ms = (lp == LP_ACTIVE) ? 1u
                          : (lp == LP_LIGHT)  ? IH_LP_LIGHT_MS
                                              : IH_LP_DEEP_MS;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void inter_half_sync_debug_state(uint8_t out[6]) {
    out[0] = (uint8_t)s_state;
    out[1] = (uint8_t)s_ih_loops;       // loop-exit counter (after tick)
    out[2] = s_slave_persistent ? 1 : 0;
#ifdef RAW_HID_ENABLE
    out[3] = want_config_serve() ? 1 : 0;
#else
    out[3] = 0xFF;
#endif
    out[4] = s_ih_tick_enter;           // tick-entry counter (before tick body)
    out[5] = s_ih_heal_phase;           // last heal phase reached (0/1/2/3)
}

void inter_half_sync_create_task(void) {
    xTaskCreate(ih_task, "ih_link", IH_TASK_STACK, NULL, IH_TASK_PRIORITY, NULL);
}

#else  // !INTER_HALF_LINK_ENABLE — no-op stubs

void           inter_half_sync_init(void)               {}
void           inter_half_sync_tick(uint32_t now)        { (void)now; }
void           inter_half_sync_create_task(void)         {}
void           inter_half_sync_request_push(void)        {}
void           inter_half_sync_arm_receive(void)         {}
ih_led_state_t inter_half_sync_led_state(void)           { return IH_LED_OFF; }
bool           inter_half_sync_peer_joined(void)         { return false; }
bool           inter_half_sync_refresh_peer(void)        { return false; }
bool           inter_half_sync_proxy_config(const uint8_t req[32], uint8_t resp[32]) { (void)req; (void)resp; return false; }
void           inter_half_sync_debug_state(uint8_t out[6]) { for (int i = 0; i < 6; i++) out[i] = 0xFF; }

#endif // INTER_HALF_LINK_ENABLE
