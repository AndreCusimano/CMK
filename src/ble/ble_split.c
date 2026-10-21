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
// ble_split.c — CMK-Split keyboard transport, Phase 18 (Core 1 only, R7 split)
//
// SLAVE side: matrix_event notify queue + snapshot + split_mode served over the
// local GATT service (att_split_packet_handler, split_tx_enqueue, ATT hooks).
// MASTER side: CMK-Split service discovery on the upstream peer
// (split_gatt_client_handler, split_start_discovery) + periodic maintenance
// timer (split_snap_timer_handler: snapshot resync, probe/re-eval retry).
//
// Strict split-mode matching: the split engages ONLY when split_mode is ON on
// BOTH halves; sibling relay/merge happens ONLY when it is OFF on both. Mixed
// states put the central in SPLIT_MISMATCH: the halves do not cooperate (the
// sibling's HID reports are dropped in ble_relay) until the states agree.
// Mode changes hot-switch on the live link (subscribe/unsubscribe + the
// split_mode characteristic NOTIFY) — the inter-half link never disconnects.
//

#include "ble_internal.h"
#include "ble_power.h"
#include "pico/time.h"
#include "ipc/ipc_core.h"
#include "diag.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

// NOTE: hid_keyboard.h (generated from the .gatt) DEFINES profile_data, so it can
// be included by ONE translation unit only (ble_manager.c). The CMK-Split ATT
// handles this module needs are passed in by ble_split_init() instead.
#include "btstack.h"
#include "board_side.h"
#include "cmk_split_uuids.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Split keyboard (Phase 18) — runtime role + CMK-Split state
// ---------------------------------------------------------------------------
typedef enum { SPLIT_STANDALONE = 0, SPLIT_MASTER, SPLIT_SLAVE, SPLIT_MISMATCH } split_role_t;

static split_role_t     s_split_role          = SPLIT_STANDALONE;
// Split runtime enable (eeconfig split_mode), pushed from Core 0 (IPC_SET_SPLIT_MODE).
// Strict matching: arbitrated against the peer's split_mode characteristic (central)
// and gates matrix_event subscription acceptance (peripheral). Defaults off until
// Core 0 syncs the persisted value at boot → never engages split unbidden.
static bool             s_split_enabled       = false;
// SLAVE: the downstream (Peripheral) connection that subscribed our matrix_event.
static hci_con_handle_t s_split_master_handle = HCI_CON_HANDLE_INVALID;
// Dedicated bidirectional power-control lane. No dependency on RGB options.
static uint16_t s_power_value, s_power_cccd;
static hci_con_handle_t s_power_sub=HCI_CON_HANDLE_INVALID;
static gatt_client_characteristic_t s_power_char;
static gatt_client_notification_t s_power_listener;
static bool s_power_client_ready, s_power_notify_pending, s_power_listening;
static uint64_t s_power_tx_at;
static uint32_t s_power_remote_matrix;

// Local 24-bit pressed bitmap (served on a matrix_snapshot READ; updated in C).
static uint8_t          s_split_snapshot[3]   = {0, 0, 0};
// Peripheral: the downstream subscribed to our split_mode NOTIFY (hot-switch:
// it learns our toggle live). Independent of the SLAVE role — the central
// subscribes it in EVERY mode combination, including relay and mismatch.
static hci_con_handle_t s_mode_sub_handle     = HCI_CON_HANDLE_INVALID;
static bool             s_mode_notify_pending = false;

// MASTER discovery (Central): CMK-Split service/characteristic handles on the
// upstream peer, and a small step machine driven off GATT_EVENT_QUERY_COMPLETE.
static gatt_client_service_t        s_split_svc;
static gatt_client_characteristic_t s_split_evt_char;      // matrix_event (NOTIFY)
static gatt_client_characteristic_t s_split_snap_char;     // matrix_snapshot (READ)
static gatt_client_characteristic_t s_split_side_char;     // side_id (READ)
static gatt_client_characteristic_t s_split_mode_char;     // split_mode (READ+NOTIFY)
static gatt_client_characteristic_t s_split_rgb_char;      // rgb_sync (WRITE_NR, Phase 24)
static gatt_client_characteristic_t s_split_batt_char;     // peer_batt (READ, bug #3)
static gatt_client_notification_t   s_split_notification;  // matrix_event subscription
static gatt_client_notification_t   s_split_mode_notification; // split_mode subscription
static uint8_t  s_split_peer_side = 0;                    // slave's board side (read via side_id)
static bool     s_split_discovering = false;
static bool     s_split_svc_found  = false;
// Central-side peer knowledge (strict matching). probed = the probe reached a
// definitive answer (CMK sibling or not); transient failures leave it false so
// the maintenance timer retries the discovery.
static bool     s_split_peer_probed     = false;
static bool     s_split_peer_is_cmk_f   = false;
static bool     s_split_peer_mode_valid = false;
static uint8_t  s_split_peer_mode       = 0;
// A re-evaluation was requested while the gatt_client was busy → retried by the
// maintenance timer (R4 pattern: a sync reject must never strand the state).
static bool     s_pending_reeval        = false;
typedef enum { SD_IDLE = 0, SD_SVC, SD_CHARS, SD_READ_SIDE, SD_READ_MODE, SD_SUB_MODE,
               SD_SUBSCRIBE, SD_UNSUBSCRIBE, SD_READ_SNAP, SD_SUB_POWER,
               SD_READ_BATT } split_disc_step_t;
static split_disc_step_t s_split_disc_step = SD_IDLE;
// Role to land on when the in-flight matrix_event unsubscribe completes.
static split_role_t s_split_unsub_target = SPLIT_STANDALONE;

// SLAVE → MASTER matrix_event notify queue (drained on ATT can-send-now).
#define SPLIT_TX_RING 32
static uint8_t          s_split_tx_ring[SPLIT_TX_RING];
static volatile uint8_t s_split_tx_head = 0;   // producer (IPC handler)
static volatile uint8_t s_split_tx_tail = 0;   // consumer (att can-send-now)

// MASTER → SLAVE rgb_sync TX (Phase 24): a small frame ring (STATE + reactive HIT
// frames coexist). Written WITHOUT response and retried on can-write so HID reports
// always keep ACL-buffer priority. Drops oldest on full (cosmetic loss self-heals).
#define SPLIT_RGB_TX_RING 16
static uint8_t  s_rgb_ring[SPLIT_RGB_TX_RING][CMK_RGB_SYNC_FRAME_MAX];
static uint8_t  s_rgb_ring_len[SPLIT_RGB_TX_RING];
static uint8_t  s_rgb_ring_head = 0;
static uint8_t  s_rgb_ring_tail = 0;

// Split snapshot resync timer (R3).
#define SPLIT_SNAP_RESYNC_MS 500
// peer_batt poll (bug #3): SoC moves ~1 %/minutes, a read a minute is plenty.
// A failed read retries sooner. Armed to "now" by the probe → first read at
// the first idle maintenance tick.
#define SPLIT_BATT_POLL_MS       60000
#define SPLIT_BATT_RETRY_MS      5000
static uint32_t s_batt_next_read_ms = 0;
static btstack_timer_source_t s_split_snap_timer;
static void split_snap_timer_handler(btstack_timer_source_t *ts);

// Local CMK-Split ATT handles (from the generated GATT DB) — wired at init by
// ble_manager.c, the only TU allowed to include hid_keyboard.h (profile_data).
static uint16_t s_att_evt_value_handle  = 0;  // matrix_event VALUE
static uint16_t s_att_evt_cccd_handle   = 0;  // matrix_event CLIENT_CONFIGURATION
static uint16_t s_att_snap_value_handle = 0;  // matrix_snapshot VALUE
static uint16_t s_att_side_value_handle = 0;  // side_id VALUE
static uint16_t s_att_mode_value_handle = 0;  // split_mode VALUE
static uint16_t s_att_mode_cccd_handle  = 0;  // split_mode CLIENT_CONFIGURATION
static uint16_t s_att_rgb_value_handle  = 0;  // rgb_sync VALUE (Phase 24, slave RX)
static uint16_t s_att_batt_value_handle = 0;  // peer_batt VALUE (bug #3)

static void split_gatt_client_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz);
static void att_split_packet_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz);
static void split_reevaluate(void);

static void split_set_role(split_role_t role, uint8_t peer_side) {
    s_split_role = role;
    switch (role) {
        case SPLIT_MASTER:
            ipc_push_to_core0(IPC_ENCODE(IPC_SPLIT_ROLE_MASTER, peer_side));
            C1_DBG_STATUS("[SPLIT] role=MASTER peer_side=%u\n", peer_side);
            break;
        case SPLIT_SLAVE:
            ipc_push_to_core0(IPC_ENCODE(IPC_SPLIT_ROLE_SLAVE, peer_side));
            C1_DBG_STATUS("[SPLIT] role=SLAVE\n");
            break;
        case SPLIT_MISMATCH:
            ipc_push_to_core0(IPC_ENCODE(IPC_SPLIT_ROLE_MISMATCH, peer_side));
            C1_DBG_STATUS("[SPLIT] role=MISMATCH (peer split_mode=%u, ours=%u)\n",
                          (unsigned)s_split_peer_mode, (unsigned)s_split_enabled);
            break;
        case SPLIT_STANDALONE:
        default:
            ipc_push_to_core0(IPC_ENCODE(IPC_SPLIT_ROLE_STANDALONE, 0));
            C1_DBG_STATUS("[SPLIT] role=STANDALONE\n");
            break;
    }
}

// Reset the SLAVE-side transport state on leaving the SLAVE role (unsubscribe or
// master disconnect). Without this the snapshot bitmap and the TX ring stayed
// stale, so a later re-engage resynced GHOST keys from the previous session.
static void split_slave_reset_state(void) {
    memset(s_split_snapshot, 0, sizeof(s_split_snapshot));
    s_split_tx_tail = s_split_tx_head;   // drop any queued, now-meaningless events
}

// ---------------------------------------------------------------------------
// Init — ATT split lane handler + snapshot resync timer
// ---------------------------------------------------------------------------

void ble_split_init(uint16_t evt_value_handle, uint16_t evt_cccd_handle,
                    uint16_t snap_value_handle, uint16_t side_value_handle,
                    uint16_t mode_value_handle, uint16_t mode_cccd_handle,
                    uint16_t rgb_value_handle, uint16_t power_value_handle, uint16_t power_cccd_handle,
                    uint16_t batt_value_handle) {
    s_power_value=power_value_handle; s_power_cccd=power_cccd_handle;
    s_att_batt_value_handle = batt_value_handle;
    s_att_evt_value_handle  = evt_value_handle;
    s_att_evt_cccd_handle   = evt_cccd_handle;
    s_att_snap_value_handle = snap_value_handle;
    s_att_side_value_handle = side_value_handle;
    s_att_mode_value_handle = mode_value_handle;
    s_att_mode_cccd_handle  = mode_cccd_handle;
    s_att_rgb_value_handle  = rgb_value_handle;

    // CMK-Split (Phase 18): dedicated ATT can-send-now lane for matrix_event /
    // split_mode notifications (peripheral→central), independent of the HIDS lane.
    att_server_register_packet_handler(att_split_packet_handler);

    // Maintenance timer — snapshot resync while MASTER (R3) + probe/re-eval retry.
    s_split_snap_timer.process = &split_snap_timer_handler;
    btstack_run_loop_set_timer(&s_split_snap_timer, SPLIT_SNAP_RESYNC_MS);
    btstack_run_loop_add_timer(&s_split_snap_timer);
}

uint8_t ble_split_role_value(void) {
    return (uint8_t)s_split_role;
}

// ---------------------------------------------------------------------------
// ATT callback hooks (called from ble_manager.c att_read_cb / att_write_cb)
// ---------------------------------------------------------------------------

// Returns 0 if the attribute is not a CMK-Split characteristic (caller falls
// through to the default ATT handling).
uint16_t ble_split_att_read(uint16_t attribute_handle, uint16_t offset,
                            uint8_t *buffer, uint16_t buffer_size) {
    if (attribute_handle==s_power_value) {
        uint8_t frame[POWER_WIRE_SIZE]; ble_power_frame(frame);
        return att_read_callback_handle_blob(frame,sizeof(frame),offset,buffer,buffer_size);
    }
    // CMK-Split side_id (Phase 18): 1 byte = this half's board side (0=L, 1=R).
    if (attribute_handle == s_att_side_value_handle) {
        uint8_t side = (uint8_t)board_side_get();
        return att_read_callback_handle_blob(&side, 1, offset, buffer, buffer_size);
    }
    // CMK-Split matrix_snapshot (Phase 18): 3-byte 24-bit pressed bitmap.
    if (attribute_handle == s_att_snap_value_handle) {
        return att_read_callback_handle_blob(s_split_snapshot, sizeof(s_split_snapshot),
                                             offset, buffer, buffer_size);
    }
    // CMK-Split split_mode: this half's eeconfig split_mode (strict matching).
    if (attribute_handle == s_att_mode_value_handle) {
        uint8_t mode = s_split_enabled ? 1 : 0;
        return att_read_callback_handle_blob(&mode, 1, offset, buffer, buffer_size);
    }
    // CMK-Split peer_batt (bug #3): this half's SoC, 0xFF = unknown.
    if (attribute_handle == s_att_batt_value_handle) {
        uint8_t soc = ble_batt_local_value();
        return att_read_callback_handle_blob(&soc, 1, offset, buffer, buffer_size);
    }
    return 0;
}

// CMK-Split (Phase 18): the downstream subscribing to our matrix_event CCCD
// means it is acting as our split MASTER → we become SLAVE (ship raw matrix).
// Unsubscribe (or that connection dropping) reverts us to STANDALONE.
// The split_mode CCCD tracks the central subscribed to our mode (hot-switch).
// Returns -1 if the attribute is not ours (caller falls through), 0 on success,
// or an ATT error code (the write is rejected at the ATT level).
int ble_split_att_write(hci_con_handle_t con_handle, uint16_t attribute_handle,
                        uint8_t *buffer, uint16_t buffer_size) {
    uint16_t cccd;
    if (attribute_handle==s_power_value || attribute_handle==s_power_cccd) {
        if (gap_encryption_key_size(con_handle)<16 ||
            (con_handle!=s_mode_sub_handle && con_handle!=s_split_master_handle))
            return ATT_ERROR_WRITE_NOT_PERMITTED;
        if (attribute_handle==s_power_cccd) {
            if (buffer_size!=2 || !buffer || buffer[1] || buffer[0]>1)
                return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
            s_power_sub=buffer[0] ? con_handle : HCI_CON_HANDLE_INVALID;
            s_power_tx_at=0;
            return 0;
        }
        ble_power_link(con_handle,true,s_power_sub==con_handle);
        return ble_power_receive(con_handle,buffer,buffer_size) ? 0 : ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
    }

    // rgb_sync (Phase 24): MASTER→SLAVE RGB effect state, write-without-response.
    // Forward to Core 0 only while we are the SLAVE (the central writes it after
    // engaging the split). The CM task re-checks the role before applying.
    if (attribute_handle == s_att_rgb_value_handle) {
        if (s_split_role == SPLIT_SLAVE && buffer && buffer_size >= 1) {
            ipc_put_rgb_sync_rx(buffer, (uint8_t)buffer_size);
        }
        return 0;
    }

    // split_mode CCCD: the central wants live updates of our toggle.
    if (attribute_handle == s_att_mode_cccd_handle) {
        cccd = (buffer && buffer_size >= 2) ? little_endian_read_16(buffer, 0) : 0;
        if (cccd & 0x0001) {
            s_mode_sub_handle = con_handle;
        } else if (con_handle == s_mode_sub_handle) {
            s_mode_sub_handle     = HCI_CON_HANDLE_INVALID;
            s_mode_notify_pending = false;
        }
        return 0;
    }

    if (attribute_handle != s_att_evt_cccd_handle) {
        return -1;
    }
    cccd = (buffer && buffer_size >= 2) ? little_endian_read_16(buffer, 0) : 0;
    if ((cccd & 0x0001) && s_split_enabled) {  // notifications enabled + split allowed
        s_split_master_handle = con_handle;
        split_set_role(SPLIT_SLAVE, 0);
    } else if ((cccd & 0x0001) && !s_split_enabled) {
        // Strict matching: split disabled → REJECT the subscribe at the ATT level.
        // A same-version central never attempts it (it arbitrates on our split_mode
        // characteristic first); this kills the "ghost MASTER" against any central
        // that tries anyway (the old FW used to accept silently → fake MASTER role).
        C1_DBG_SPLIT("[SPLIT] disabled → reject matrix_event subscribe\n");
        return ATT_ERROR_WRITE_NOT_PERMITTED;
    } else if (con_handle == s_split_master_handle) {
        s_split_master_handle = HCI_CON_HANDLE_INVALID;
        split_slave_reset_state();
        if (s_split_role == SPLIT_SLAVE) split_set_role(SPLIT_STANDALONE, 0);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Disconnect hooks
// ---------------------------------------------------------------------------

// Split: if our master (the downstream that subscribed matrix_event) dropped,
// fall back to STANDALONE (resume local typing) and reset the SLAVE transport.
void ble_split_on_periph_disconnect(hci_con_handle_t handle) {
    if (handle==s_power_sub) {
        s_power_sub=HCI_CON_HANDLE_INVALID; s_power_notify_pending=false;
        ble_power_link(HCI_CON_HANDLE_INVALID,false,false);
    }
    if (handle == s_mode_sub_handle) {
        s_mode_sub_handle     = HCI_CON_HANDLE_INVALID;
        s_mode_notify_pending = false;
    }
    if (handle == s_split_master_handle) {
        s_split_master_handle = HCI_CON_HANDLE_INVALID;
        split_slave_reset_state();
        if (s_split_role == SPLIT_SLAVE) split_set_role(SPLIT_STANDALONE, 0);
    }
}

// Split: the upstream dropped → abort discovery, forget everything we knew
// about the peer (re-probed on the next connection) and fall back to STANDALONE.
void ble_split_on_central_disconnect(void) {
    if (s_power_listening) gatt_client_stop_listening_for_characteristic_value_updates(&s_power_listener);
    s_power_listening=false; s_power_client_ready=false; s_power_char.value_handle=0;
    s_power_remote_matrix=0; ble_power_peer_keys(0);
    ble_power_link(HCI_CON_HANDLE_INVALID,false,false);
    s_split_discovering     = false;
    s_split_disc_step       = SD_IDLE;
    s_split_peer_probed     = false;
    s_split_peer_is_cmk_f   = false;
    s_split_peer_mode_valid = false;
    s_pending_reeval        = false;
    s_split_rgb_char.value_handle = 0;   // Phase 24: forget the slave's rgb_sync handle
    s_split_batt_char.value_handle = 0;  // bug #3 (the SoC itself is reset by ble_batt)
    s_rgb_ring_tail = s_rgb_ring_head;   // drop any queued RGB frames
    if (s_split_role == SPLIT_MASTER || s_split_role == SPLIT_MISMATCH) {
        split_set_role(SPLIT_STANDALONE, 0);
    }
}

// ---------------------------------------------------------------------------
// IPC command body (dispatched from ble_manager_handle_ipc)
// ---------------------------------------------------------------------------

// Core 0 synced the eeconfig split_mode (boot + CMK_SPLIT_TOGG). Hot-switch on
// the live link — the inter-half connection is NEVER torn down by a mode flip:
//  - peripheral side: notify the new value to the subscribed central, which
//    re-arbitrates (it will unsubscribe matrix_event if we left the split). A
//    SLAVE that toggles OFF stays SLAVE until the central's unsubscribe lands
//    (~1 conn interval) — the CCCD remains the single source of truth of the
//    role, so there is no window with two key paths active at once.
//  - central side: re-evaluate the desired role against the peer's mode.
void ble_split_set_mode(bool en) {
    bool changed = (en != s_split_enabled);
    s_split_enabled = en;
    if (!changed) return;
    C1_DBG_SPLIT("[SPLIT] split_mode=%d (hot-switch)\n", (int)en);
    if (s_mode_sub_handle != HCI_CON_HANDLE_INVALID) {
        s_mode_notify_pending = true;
        att_server_request_can_send_now_event(s_mode_sub_handle);
    }
    split_reevaluate();
}

// ---------------------------------------------------------------------------
// SLAVE side — matrix_event notify queue
// ---------------------------------------------------------------------------

// Peripheral → central: one notification per ATT can-send-now. A pending
// split_mode notify (rare: only on toggle) takes priority over the matrix_event
// ring; re-arms while there is more work. Registered as an att_server handler.
static void att_split_packet_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz) {
    UNUSED(ch); UNUSED(sz);
    if (ptype != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(pkt) != ATT_EVENT_CAN_SEND_NOW) return;

    // 0) split_mode notify (hot-switch): tell the central our toggle changed.
    if (s_mode_notify_pending && s_mode_sub_handle != HCI_CON_HANDLE_INVALID) {
        uint8_t mode = s_split_enabled ? 1 : 0;
        s_mode_notify_pending = false;
        att_server_notify(s_mode_sub_handle, s_att_mode_value_handle, &mode, 1);
        C1_DBG_SPLIT("[SPLIT] notify->central split_mode=%u\n", mode);
        if (s_split_master_handle != HCI_CON_HANDLE_INVALID &&
            s_split_tx_tail != s_split_tx_head) {
            att_server_request_can_send_now_event(s_split_master_handle);
        }
        return;
    }

    // 1) matrix_event ring (SLAVE → MASTER).
    if (s_split_master_handle == HCI_CON_HANDLE_INVALID || s_split_tx_tail == s_split_tx_head) {
        // Only use the split lane after all key events; retry without dropping.
        if (s_power_notify_pending && s_power_sub!=HCI_CON_HANDLE_INVALID) {
            uint8_t frame[POWER_WIRE_SIZE]; ble_power_frame(frame);
            if (att_server_notify(s_power_sub,s_power_value,frame,sizeof(frame))==ERROR_CODE_SUCCESS)
                s_power_notify_pending=false;
        }
        return;
    }
    uint8_t evt = s_split_tx_ring[s_split_tx_tail];
    s_split_tx_tail = (uint8_t)((s_split_tx_tail + 1) % SPLIT_TX_RING);
    att_server_notify(s_split_master_handle, s_att_evt_value_handle, &evt, 1);
    C1_DBG_SPLIT("[SPLIT] notify->master evt=0x%02x (r=%u c=%u %s)\n",
                 evt, CMK_SPLIT_EVT_ROW(evt), CMK_SPLIT_EVT_COL(evt),
                 CMK_SPLIT_EVT_PRESSED(evt) ? "DN" : "UP");
    if (s_split_tx_tail != s_split_tx_head) {
        att_server_request_can_send_now_event(s_split_master_handle);
    }
}

// Queue a SLAVE matrix event for notification + keep the snapshot bitmap current.
void split_tx_enqueue(uint8_t evt) {
    uint8_t row = CMK_SPLIT_EVT_ROW(evt), col = CMK_SPLIT_EVT_COL(evt);
    if (row < MATRIX_ROWS && col < MATRIX_COLS) {
        uint16_t bit = (uint16_t)(row * MATRIX_COLS + col);
        if (CMK_SPLIT_EVT_PRESSED(evt)) s_split_snapshot[bit >> 3] |=  (uint8_t)(1u << (bit & 7));
        else                            s_split_snapshot[bit >> 3] &= (uint8_t)~(1u << (bit & 7));
    }
    if (s_split_master_handle == HCI_CON_HANDLE_INVALID) return;
    uint8_t next = (uint8_t)((s_split_tx_head + 1) % SPLIT_TX_RING);
    if (next != s_split_tx_tail) {            // not full (else drop → snapshot heals)
        s_split_tx_ring[s_split_tx_head] = evt;
        s_split_tx_head = next;
    } else {
        DIAG_INC(split_tx_drop);
    }
    att_server_request_can_send_now_event(s_split_master_handle);
}

// ---------------------------------------------------------------------------
// MASTER → SLAVE rgb_sync TX (Phase 24)
// ---------------------------------------------------------------------------
// The central writes the staged RGB frame to the slave's rgb_sync characteristic
// WITHOUT response: no ATT round-trip, and if the ACL buffer is full (HID report
// in flight) the write is deferred to the next can-write event — HID always wins.

static void split_rgb_pump(void);

static void split_rgb_can_write_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz) {
    UNUSED(ptype); UNUSED(ch); UNUSED(pkt); UNUSED(sz);
    split_rgb_pump();
}

static void split_rgb_ring_push(const uint8_t *frame, uint8_t len) {
    if (len == 0 || len > CMK_RGB_SYNC_FRAME_MAX) return;
    uint8_t next = (uint8_t)((s_rgb_ring_head + 1) % SPLIT_RGB_TX_RING);
    if (next == s_rgb_ring_tail) {   // full → drop oldest
        s_rgb_ring_tail = (uint8_t)((s_rgb_ring_tail + 1) % SPLIT_RGB_TX_RING);
    }
    memcpy(s_rgb_ring[s_rgb_ring_head], frame, len);
    s_rgb_ring_len[s_rgb_ring_head] = len;
    s_rgb_ring_head = next;
}

// Drain the ring to the slave's rgb_sync characteristic. Each write without
// response either ships (consumes an ACL buffer) or reports buffers full → retry
// on the next can-write event, so HID notifications always win the buffer race.
static void split_rgb_pump(void) {
    if (s_split_role != SPLIT_MASTER ||
        g_ble_central_handle == HCI_CON_HANDLE_INVALID ||
        s_split_rgb_char.value_handle == 0) {
        s_rgb_ring_tail = s_rgb_ring_head;   // not a master / no peer → drop queue
        return;
    }
    while (s_rgb_ring_tail != s_rgb_ring_head) {
        uint8_t st = gatt_client_write_value_of_characteristic_without_response(
            g_ble_central_handle, s_split_rgb_char.value_handle,
            s_rgb_ring_len[s_rgb_ring_tail], s_rgb_ring[s_rgb_ring_tail]);
        if (st != ERROR_CODE_SUCCESS) {
            gatt_client_request_can_write_without_response_event(
                split_rgb_can_write_handler, g_ble_central_handle);
            return;
        }
        s_rgb_ring_tail = (uint8_t)((s_rgb_ring_tail + 1) % SPLIT_RGB_TX_RING);
    }
}

void ble_split_send_rgb_sync(void) {
    uint8_t frame[CMK_RGB_SYNC_FRAME_MAX];
    uint8_t len = 0;
    if (!ipc_get_rgb_sync_tx(frame, &len) || len == 0) return;
    split_rgb_ring_push(frame, len);
    split_rgb_pump();
}

void ble_split_send_rgb_hit(uint8_t packed) {
    uint8_t frame[CMK_RGB_SYNC_HIT_LEN] = {
        CMK_RGB_SYNC_TAG_HIT,
        CMK_RGB_HIT_SIDE(packed),
        CMK_RGB_HIT_ROW(packed),
        CMK_RGB_HIT_COL(packed),
    };
    split_rgb_ring_push(frame, CMK_RGB_SYNC_HIT_LEN);
    split_rgb_pump();
}

// ---------------------------------------------------------------------------
// CMK-Split discovery (Central / MASTER side, Phase 18)
// ---------------------------------------------------------------------------
// Kicked off after the upstream HID service is ready (the hids_host has released
// the gatt_client). Runs REGARDLESS of our own split_mode (strict matching needs
// to know whether the upstream is a CMK sibling and what its mode is). Steps chain
// strictly on GATT_EVENT_QUERY_COMPLETE: discover the CMK-Split service → its
// characteristics → read side_id → read split_mode → subscribe split_mode → then
// arbitrate (split_reevaluate). If the service is absent the upstream is a
// 3rd-party HID → stay STANDALONE/relay.

void split_start_discovery(void) {
    if (g_ble_central_handle == HCI_CON_HANDLE_INVALID) return;
    if (s_power_listening) gatt_client_stop_listening_for_characteristic_value_updates(&s_power_listener);
    s_power_listening=false; s_power_client_ready=false; s_power_char.value_handle=0;
    s_split_svc_found              = false;
    s_split_evt_char.value_handle  = 0;
    s_split_side_char.value_handle = 0;
    s_split_mode_char.value_handle = 0;
    s_split_rgb_char.value_handle  = 0;
    s_split_batt_char.value_handle = 0;
    s_batt_next_read_ms            = btstack_run_loop_get_time_ms();
    s_split_peer_probed            = false;
    s_split_peer_is_cmk_f          = false;
    s_split_peer_mode_valid        = false;
    s_split_disc_step              = SD_SVC;
    s_split_discovering            = true;
    uint8_t st = gatt_client_discover_primary_services_by_uuid128(
                     split_gatt_client_handler, g_ble_central_handle, cmk_split_service_uuid);
    if (st != ERROR_CODE_SUCCESS) {
        // R4: sync reject (gatt client busy) → reset; the maintenance timer retries.
        s_split_discovering = false;
        s_split_disc_step   = SD_IDLE;
        C1_DBG_STATUS("[SPLIT] discover start failed 0x%02x → retry later\n", st);
    }
}

static void split_gatt_client_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz) {
    UNUSED(ptype); UNUSED(ch); UNUSED(sz);
    switch (hci_event_packet_get_type(pkt)) {

    case GATT_EVENT_SERVICE_QUERY_RESULT:
        gatt_event_service_query_result_get_service(pkt, &s_split_svc);
        s_split_svc_found = true;
        break;

    case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
        gatt_client_characteristic_t c;
        gatt_event_characteristic_query_result_get_characteristic(pkt, &c);
        if      (memcmp(c.uuid128, cmk_split_matrix_event_uuid,    16) == 0) s_split_evt_char  = c;
        else if (memcmp(c.uuid128, cmk_split_matrix_snapshot_uuid, 16) == 0) s_split_snap_char = c;
        else if (memcmp(c.uuid128, cmk_split_side_id_uuid,         16) == 0) s_split_side_char = c;
        else if (memcmp(c.uuid128, cmk_split_mode_uuid,            16) == 0) s_split_mode_char = c;
        else if (memcmp(c.uuid128, cmk_split_rgb_sync_uuid,        16) == 0) s_split_rgb_char  = c;
        else if (memcmp(c.uuid128, cmk_split_power_uuid,           16) == 0) s_power_char = c;
        else if (memcmp(c.uuid128, cmk_split_peer_batt_uuid,       16) == 0) s_split_batt_char = c;
        break;
    }

    case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT: {
        // Used for the side_id (SD_READ_SIDE), split_mode (SD_READ_MODE) and
        // matrix_snapshot (SD_READ_SNAP) reads, distinguished by the current step.
        const uint8_t *val = gatt_event_characteristic_value_query_result_get_value(pkt);
        uint16_t vlen      = gatt_event_characteristic_value_query_result_get_value_length(pkt);
        if (s_split_disc_step == SD_READ_SIDE && vlen >= 1) {
            s_split_peer_side = val[0];
        } else if (s_split_disc_step == SD_READ_MODE && vlen >= 1) {
            s_split_peer_mode       = val[0];
            s_split_peer_mode_valid = true;
        } else if (s_split_disc_step == SD_READ_SNAP && vlen >= 3) {
            uint32_t snap = (uint32_t)val[0] | ((uint32_t)val[1] << 8) | ((uint32_t)val[2] << 16);
            s_power_remote_matrix=snap & 0x00FFFFFFu;
            ble_power_peer_keys(s_power_remote_matrix);
            ipc_push_to_core0(IPC_ENCODE(IPC_SPLIT_RX_SNAPSHOT, snap & 0x00FFFFFFu));
        } else if (s_split_disc_step == SD_READ_BATT && vlen >= 1) {
            ble_batt_set_peer(val[0]);
        }
        break;
    }

    case GATT_EVENT_NOTIFICATION: {
        // Two subscriptions share this handler — dispatch on the value handle:
        // matrix_event (raw key change from the slave) vs split_mode (the peer
        // toggled → re-arbitrate the role on the live link).
        uint16_t vhandle = gatt_event_notification_get_value_handle(pkt);
        if (vhandle==s_power_char.value_handle && s_power_client_ready) {
            ble_power_receive(gatt_event_notification_get_handle(pkt),
                gatt_event_notification_get_value(pkt),gatt_event_notification_get_value_length(pkt));
            break;
        }
        if (gatt_event_notification_get_value_length(pkt) < 1) break;
        uint8_t v0 = gatt_event_notification_get_value(pkt)[0];
        if (vhandle == s_split_mode_char.value_handle) {
            C1_DBG_SPLIT("[SPLIT] peer split_mode=%u (notify) → re-eval\n", v0);
            s_split_peer_mode       = v0;
            s_split_peer_mode_valid = true;
            split_reevaluate();
        } else if (vhandle == s_split_evt_char.value_handle) {
            C1_DBG_SPLIT("[SPLIT] rx<-slave evt=0x%02x (r=%u c=%u %s)\n",
                         v0, CMK_SPLIT_EVT_ROW(v0), CMK_SPLIT_EVT_COL(v0),
                         CMK_SPLIT_EVT_PRESSED(v0) ? "DN" : "UP");
            uint8_t row=CMK_SPLIT_EVT_ROW(v0), col=CMK_SPLIT_EVT_COL(v0);
            if (row<MATRIX_ROWS && col<MATRIX_COLS) {
                uint32_t bit=1u<<(row*MATRIX_COLS+col);
                if (CMK_SPLIT_EVT_PRESSED(v0)) s_power_remote_matrix|=bit;
                else s_power_remote_matrix&=~bit;
                ble_power_peer_keys(s_power_remote_matrix);
            }
            ipc_push_to_core0(IPC_ENCODE(IPC_SPLIT_RX_EVENT, v0));
        }
        break;
    }

    case GATT_EVENT_QUERY_COMPLETE: {
        uint8_t status = gatt_event_query_complete_get_att_status(pkt);
        switch (s_split_disc_step) {
        case SD_SVC:
            if (status == ATT_ERROR_SUCCESS && s_split_svc_found) {
                s_split_disc_step = SD_CHARS;
                if (gatt_client_discover_characteristics_for_service(
                        split_gatt_client_handler, g_ble_central_handle, &s_split_svc)
                    != ERROR_CODE_SUCCESS) {
                    // R4: a synchronous reject (gatt client busy) produced no
                    // QUERY_COMPLETE → discovery stuck until disconnect. Reset;
                    // the maintenance timer retries the probe.
                    s_split_discovering = false; s_split_disc_step = SD_IDLE;
                }
            } else {
                // Definitive: no CMK-Split service → 3rd-party HID upstream.
                s_split_discovering = false; s_split_disc_step = SD_IDLE;
                s_split_peer_probed = true;  s_split_peer_is_cmk_f = false;
                C1_DBG_STATUS("[SPLIT] no CMK-Split on upstream → STANDALONE/relay\n");
                ble_batt_on_upstream_probed(false);   // bug #3: try its Battery Service
            }
            break;
        case SD_CHARS:
            if (status == ATT_ERROR_SUCCESS &&
                s_split_evt_char.value_handle && s_split_side_char.value_handle &&
                s_split_mode_char.value_handle) {
                s_split_disc_step = SD_READ_SIDE;
                if (gatt_client_read_value_of_characteristic(
                        split_gatt_client_handler, g_ble_central_handle, &s_split_side_char)
                    != ERROR_CODE_SUCCESS) {
                    s_split_discovering = false; s_split_disc_step = SD_IDLE;   // R4
                }
            } else {
                // Definitive: chars incomplete (or split_mode absent = pre-strict-
                // matching FW — mixed versions unsupported) → treat as 3rd-party.
                s_split_discovering = false; s_split_disc_step = SD_IDLE;
                s_split_peer_probed = true;  s_split_peer_is_cmk_f = false;
                C1_DBG_STATUS("[SPLIT] CMK-Split chars incomplete/old FW → STANDALONE/relay\n");
                ble_batt_on_upstream_probed(false);   // bug #3: try its Battery Service
            }
            break;
        case SD_READ_SIDE:
            s_split_disc_step = SD_READ_MODE;
            if (gatt_client_read_value_of_characteristic(
                    split_gatt_client_handler, g_ble_central_handle, &s_split_mode_char)
                != ERROR_CODE_SUCCESS) {
                s_split_discovering = false; s_split_disc_step = SD_IDLE;   // R4
            }
            break;
        case SD_READ_MODE:
            // Peer mode read → subscribe its NOTIFY so a remote toggle re-arbitrates
            // live (hot-switch from the other half, no reconnection needed).
            s_split_disc_step = SD_SUB_MODE;
            gatt_client_listen_for_characteristic_value_updates(
                &s_split_mode_notification, split_gatt_client_handler,
                g_ble_central_handle, &s_split_mode_char);
            if (gatt_client_write_client_characteristic_configuration(
                    split_gatt_client_handler, g_ble_central_handle, &s_split_mode_char,
                    GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION)
                != ERROR_CODE_SUCCESS) {
                gatt_client_stop_listening_for_characteristic_value_updates(&s_split_mode_notification);
                s_split_discovering = false; s_split_disc_step = SD_IDLE;   // R4
            }
            break;
        case SD_SUB_MODE:
            s_split_discovering = false; s_split_disc_step = SD_IDLE;
            if (status == ATT_ERROR_SUCCESS && s_split_peer_mode_valid) {
                // Probe complete: a CMK sibling, mode known → arbitrate the role.
                s_split_peer_probed   = true;
                s_split_peer_is_cmk_f = true;
                C1_DBG_STATUS("[SPLIT] sibling probed: side=%u mode=%u (ours=%u)\n",
                              s_split_peer_side, s_split_peer_mode, (unsigned)s_split_enabled);
                if (s_power_char.value_handle) {
                    s_split_disc_step=SD_SUB_POWER; s_split_discovering=true;
                    gatt_client_listen_for_characteristic_value_updates(&s_power_listener,
                        split_gatt_client_handler,g_ble_central_handle,&s_power_char);
                    s_power_listening=true;
                    if (gatt_client_write_client_characteristic_configuration(split_gatt_client_handler,
                        g_ble_central_handle,&s_power_char,GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION)
                        == ERROR_CODE_SUCCESS) break;
                    s_split_disc_step=SD_IDLE; s_split_discovering=false;
                }
                split_reevaluate();
            } else {
                // Transient failure → probed stays false, maintenance timer retries.
                gatt_client_stop_listening_for_characteristic_value_updates(&s_split_mode_notification);
                C1_DBG_STATUS("[SPLIT] split_mode subscribe failed 0x%02x → retry later\n", status);
            }
            break;
        case SD_SUB_POWER:
            s_split_discovering=false; s_split_disc_step=SD_IDLE;
            s_power_client_ready=status==ATT_ERROR_SUCCESS;
            s_power_tx_at=0;
            C1_DBG_STATUS("[LP] peer control subscription %s\n",s_power_client_ready?"ready":"failed");
            split_reevaluate();
            break;
        case SD_SUBSCRIBE:
            s_split_discovering = false; s_split_disc_step = SD_IDLE;
            if (status == ATT_ERROR_SUCCESS) {
                split_set_role(SPLIT_MASTER, s_split_peer_side);
                // The sibling stops sending HID reports from this instant, so the
                // last relay report would stay merged forever — release any keys
                // it was holding (a key held through the engage stuck at the PC).
                ble_relay_on_central_disconnect();
                // Tighten the inter-half link for low typing latency, at an
                // interval deliberately coprime with the mouse link's — see
                // SPLIT_LINK_INTERVAL in cmk_config.h for why a multiple of it
                // lets the radio latch into losing most of our events.
                // We are MASTER on this handle → direct HCI update. The L2CAP
                // request API (gap_request_connection_parameter_update) is
                // slave→master only — the peer rejects it without effect.
                gap_update_connection_parameters(g_ble_central_handle,
                                                 SPLIT_LINK_INTERVAL, SPLIT_LINK_INTERVAL, 0, 400);
                // Read the slave's full pressed bitmap once for an initial re-sync.
                if (s_split_snap_char.value_handle) {
                    s_split_disc_step   = SD_READ_SNAP;
                    s_split_discovering = true;
                    if (gatt_client_read_value_of_characteristic(
                            split_gatt_client_handler, g_ble_central_handle, &s_split_snap_char)
                        != ERROR_CODE_SUCCESS) {
                        // R4: the periodic resync timer will retry the read.
                        s_split_discovering = false; s_split_disc_step = SD_IDLE;
                    }
                }
            } else {
                gatt_client_stop_listening_for_characteristic_value_updates(&s_split_notification);
                C1_DBG_STATUS("[SPLIT] matrix_event subscribe failed 0x%02x → STANDALONE\n", status);
            }
            break;
        case SD_UNSUBSCRIBE:
            // Live disengage (hot-switch OFF or peer left the split): land on the
            // role decided at re-eval time. The link keeps BLE_LINK_INTERVAL: no
            // relaxing here, ble_link_policy.c would only undo it (every master
            // link stays at the one interval — relay mode still carries the mouse
            // over it). On a remote-side error the link is likely dying anyway —
            // the disconnect hooks finish the cleanup.
            s_split_discovering = false; s_split_disc_step = SD_IDLE;
            if (s_split_unsub_target == SPLIT_MISMATCH) {
                ble_relay_on_central_disconnect();   // release any upstream-held keys
                split_set_role(SPLIT_MISMATCH, s_split_peer_side);
            } else {
                split_set_role(SPLIT_STANDALONE, 0);
            }
            break;
        case SD_READ_SNAP:
            // Initial snapshot delivered in the VALUE_QUERY_RESULT above; done.
            s_split_discovering = false; s_split_disc_step = SD_IDLE;
            break;
        case SD_READ_BATT:
            // Value delivered in VALUE_QUERY_RESULT. Best-effort: a failure
            // only retries sooner, it never touches the split state.
            s_split_disc_step = SD_IDLE;
            s_batt_next_read_ms = btstack_run_loop_get_time_ms() +
                (status == ATT_ERROR_SUCCESS ? SPLIT_BATT_POLL_MS : SPLIT_BATT_RETRY_MS);
            break;
        default:
            break;
        }
        // States may have flipped while a GATT operation was in flight — the
        // deferred re-evaluation runs as soon as the step machine goes idle.
        if (s_pending_reeval && s_split_disc_step == SD_IDLE && !s_split_discovering) {
            s_pending_reeval = false;
            split_reevaluate();
        }
        break;
    }

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Strict-matching arbitration (hot-switch core)
// ---------------------------------------------------------------------------
// Decide the desired role from (our mode, peer mode) and transition LIVE on the
// existing link: ON/ON → MASTER (subscribe matrix_event), OFF/OFF → STANDALONE
// relay, mixed → MISMATCH (the sibling's HID reports are dropped in ble_relay).
// Called on: probe completion, local toggle (ble_split_set_mode), peer toggle
// (split_mode notify). Idempotent; if the gatt_client is busy the request is
// parked in s_pending_reeval and retried (QUERY_COMPLETE tail / maintenance timer).

static void split_reevaluate(void) {
    if (g_ble_central_handle == HCI_CON_HANDLE_INVALID) return;
    if (!s_split_peer_is_cmk_f || !s_split_peer_mode_valid) return;  // 3rd-party/unknown → relay
    if (s_split_discovering || s_split_disc_step != SD_IDLE) {
        s_pending_reeval = true;   // a GATT op is in flight — re-run when idle
        return;
    }

    split_role_t desired;
    bool peer_on = (s_split_peer_mode != 0);
    if (s_split_enabled && peer_on)        desired = SPLIT_MASTER;
    else if (!s_split_enabled && !peer_on) desired = SPLIT_STANDALONE;  // relay
    else                                   desired = SPLIT_MISMATCH;
    if (desired == s_split_role) return;

    if (desired == SPLIT_MASTER) {
        // Engage on the already-discovered handles (no re-discovery needed).
        s_split_disc_step   = SD_SUBSCRIBE;
        s_split_discovering = true;
        gatt_client_listen_for_characteristic_value_updates(
            &s_split_notification, split_gatt_client_handler,
            g_ble_central_handle, &s_split_evt_char);
        if (gatt_client_write_client_characteristic_configuration(
                split_gatt_client_handler, g_ble_central_handle, &s_split_evt_char,
                GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION)
            != ERROR_CODE_SUCCESS) {
            gatt_client_stop_listening_for_characteristic_value_updates(&s_split_notification);
            s_split_discovering = false; s_split_disc_step = SD_IDLE;
            s_pending_reeval = true;   // R4: retry from the maintenance timer
        }
        return;
    }

    if (s_split_role == SPLIT_MASTER) {
        // Disengage: unsubscribe matrix_event on the live link, then land on
        // STANDALONE (relay) or MISMATCH once the CCCD write completes.
        s_split_unsub_target = desired;
        s_split_disc_step    = SD_UNSUBSCRIBE;
        s_split_discovering  = true;
        if (gatt_client_write_client_characteristic_configuration(
                split_gatt_client_handler, g_ble_central_handle, &s_split_evt_char,
                GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NONE)
            != ERROR_CODE_SUCCESS) {
            s_split_discovering = false; s_split_disc_step = SD_IDLE;
            s_pending_reeval = true;   // R4: retry from the maintenance timer
            return;
        }
        gatt_client_stop_listening_for_characteristic_value_updates(&s_split_notification);
        return;
    }

    // Pure state change (STANDALONE ↔ MISMATCH) — no GATT operation involved.
    if (desired == SPLIT_MISMATCH) {
        // The sibling's HID stops being merged from this instant: release any
        // keys it was holding so nothing stays stuck at the PC.
        ble_relay_on_central_disconnect();
        split_set_role(SPLIT_MISMATCH, s_split_peer_side);
    } else {
        split_set_role(SPLIT_STANDALONE, 0);
    }
}

// ---------------------------------------------------------------------------
// Maintenance timer (500 ms) — three duties, one timer:
//  (a) retry a deferred re-evaluation (the gatt_client was busy at request time);
//  (b) retry the sibling probe if it never reached a definitive answer (R4 sync
//      rejects reset the step machine; without a retry the probe was lost until
//      the next disconnect);
//  (c) snapshot resync while MASTER (R3) — every hop of a split matrix event
//      (HW FIFO ×2 + TX/RX rings) can drop under burst; re-reading the slave's
//      pressed bitmap turns any drop into a ≤500 ms glitch (keyscan XOR-diffs
//      the bitmap and emits the deltas).
// ---------------------------------------------------------------------------

static void split_snap_timer_handler(btstack_timer_source_t *ts) {
    bool idle = !s_split_discovering && s_split_disc_step == SD_IDLE &&
                g_ble_central_handle != HCI_CON_HANDLE_INVALID;

    if (idle && s_pending_reeval) {                              // (a)
        s_pending_reeval = false;
        split_reevaluate();
    } else if (idle && !s_split_peer_probed &&                   // (b)
               g_ble_upstream_map_valid) {
        // map_valid ⇒ the hids_host finished its discovery (gatt_client free).
        split_start_discovery();
    } else if (idle && s_split_peer_is_cmk_f && s_power_char.value_handle && !s_power_client_ready) {
        // Subscription may initially race encryption; retry without disrupting
        // the established matrix/HID connection or appending a listener twice.
        if (!s_power_listening) {
            gatt_client_listen_for_characteristic_value_updates(&s_power_listener,
                split_gatt_client_handler,g_ble_central_handle,&s_power_char);
            s_power_listening=true;
        }
        s_split_disc_step=SD_SUB_POWER; s_split_discovering=true;
        if (gatt_client_write_client_characteristic_configuration(split_gatt_client_handler,
            g_ble_central_handle,&s_power_char,GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION)
            != ERROR_CODE_SUCCESS) { s_split_disc_step=SD_IDLE; s_split_discovering=false; }
    } else if (idle && s_split_peer_is_cmk_f && s_split_batt_char.value_handle != 0 &&
               (int32_t)(btstack_run_loop_get_time_ms() - s_batt_next_read_ms) >= 0) {
        // Bug #3: poll the sibling's SoC (any mode: relay and mismatch too —
        // the PC's battery reading must not depend on the split role).
        s_split_disc_step = SD_READ_BATT;
        if (gatt_client_read_value_of_characteristic(
                split_gatt_client_handler, g_ble_central_handle, &s_split_batt_char)
            != ERROR_CODE_SUCCESS) {
            s_split_disc_step = SD_IDLE;   // gatt client busy — retry next period
        }
    } else if (idle && s_split_role == SPLIT_MASTER &&           // (c)
               s_split_snap_char.value_handle != 0) {
        s_split_disc_step = SD_READ_SNAP;
        uint8_t st = gatt_client_read_value_of_characteristic(
            split_gatt_client_handler, g_ble_central_handle, &s_split_snap_char);
        if (st != ERROR_CODE_SUCCESS) {
            s_split_disc_step = SD_IDLE;   // gatt client busy — retry next period
        }
    }
    btstack_run_loop_set_timer(ts, SPLIT_SNAP_RESYNC_MS);
    btstack_run_loop_add_timer(ts);
}

// ---------------------------------------------------------------------------
// Strict-matching accessors (used by ble_relay.c to gate the sibling's HID)
// ---------------------------------------------------------------------------

bool ble_split_peer_is_cmk(void) {
    return s_split_peer_is_cmk_f;
}

bool ble_split_states_match(void) {
    return s_split_peer_mode_valid &&
           ((s_split_peer_mode != 0) == s_split_enabled);
}

// 4 Hz bounded state refresh; each frame retains the age of the original input.
// Failed TX keeps the newest state pending. It cannot create fictitious activity.
void ble_split_power_poll(void) {
    bool central=s_split_peer_is_cmk_f && g_ble_central_handle!=HCI_CON_HANDLE_INVALID;
    hci_con_handle_t handle=central ? g_ble_central_handle : s_mode_sub_handle;
    bool present=handle!=HCI_CON_HANDLE_INVALID;
    bool ready=present && gap_encryption_key_size(handle)>=16 &&
               (central ? s_power_client_ready : s_power_sub==handle);
    ble_power_link(handle,present,ready);
    uint64_t now=time_us_64();
    if (!ready || now<s_power_tx_at) return;
    if (central) {
        if (s_split_discovering) return;
        uint8_t frame[POWER_WIRE_SIZE]; ble_power_frame(frame);
        if (gatt_client_write_value_of_characteristic_without_response(handle,
            s_power_char.value_handle,sizeof(frame),frame)!=ERROR_CODE_SUCCESS) return;
    } else {
        if (!s_power_notify_pending) {
            s_power_notify_pending=true;
            att_server_request_can_send_now_event(handle);
        } else {
            // A busy HID lane can delay this callback. Re-request at bounded rate.
            att_server_request_can_send_now_event(handle);
        }
    }
    s_power_tx_at=now+250000;
}
bool ble_split_power_busy(void) {
    return s_split_tx_head!=s_split_tx_tail;
}

bool ble_split_peer_probe_complete(void) { return s_split_peer_probed; }
