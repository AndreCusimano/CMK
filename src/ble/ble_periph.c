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
// ble_periph.c — peripheral/downstream role (Core 1 only, R7 split)
//
// General + directed advertising, per-slot random static addresses, ADV
// timeout, deferred-ADV workaround (CYW43 vs gap_connect), reject mode,
// downstream connect/disconnect handling.
//

#include "ble_manager.h"
#include "ble_internal.h"
#include "ipc/ipc_core.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "btstack.h"
#include "hardware/gpio.h"
#include "board_side.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Shared state owned by this module (see ble_internal.h)
// ---------------------------------------------------------------------------

periph_state_t   g_ble_periph_state      = PERIPH_IDLE;
hci_con_handle_t g_ble_periph_handle     = HCI_CON_HANDLE_INVALID;
bool             g_ble_periph_subscribed = false;
bool             g_ble_periph_reject     = false;
uint8_t          g_ble_active_slot       = 0;   // which slot is currently set for ADV

// ---------------------------------------------------------------------------
// Module-private state
// ---------------------------------------------------------------------------

// Track how the current peripheral connection originated, so we know what to do
// on disconnect:
//   GENERAL  → restart general ADV (user wants new pairing, old Mac disruption)
//   DIRECTED → stop ADV (anti-loop for btstack_tlv_none)
//   NONE     → no special handling
typedef enum { ORIGIN_NONE, ORIGIN_GENERAL_ADV, ORIGIN_DIRECTED_ADV } periph_origin_t;
static periph_origin_t  s_periph_origin     = ORIGIN_NONE;
// Remember the general ADV timeout end-tick so we can restart within the window
static uint32_t         s_adv_timeout_end   = 0;

// Intentional disconnect — set when Core 0 sends IPC_DISCONNECT_PERIPH (slot switch).
// Suppresses auto-restart advertising on HCI_EVENT_DISCONNECTION_COMPLETE so that
// Core 0's deferred ADV (directed or general) takes over without interference.
static bool s_periph_disconnect_intentional = false;

// Deferred advertising — CYW43 rejects LE Set Advertising Enable (0x0C Command
// Disallowed) while LE Create Connection is outstanding (Initiating state).
// When ADV is requested during an active gap_connect(), we cancel the connection
// first and defer the ADV start until GAP_SUBEVENT_LE_CONNECTION_COMPLETE fires.
typedef enum {
    DEFERRED_ADV_NONE      = 0,
    DEFERRED_ADV_GENERAL   = 1,
    DEFERRED_ADV_DIRECTED  = 2,
} deferred_adv_t;
static deferred_adv_t s_deferred_adv       = DEFERRED_ADV_NONE;
static uint8_t        s_deferred_adv_slot  = 0;
static bd_addr_t      s_deferred_adv_addr;
static bd_addr_type_t s_deferred_adv_addr_type;

// Fallback timer — CYW43 sometimes does not fire LE_CONNECTION_COMPLETE after
// gap_connect_cancel(), leaving the deferred ADV stuck forever.
// This timer fires after 500ms and force-executes the pending ADV.
static btstack_timer_source_t s_deferred_adv_fallback_timer;

// ADV timeout timer (BTstack, Core 1)
static btstack_timer_source_t s_adv_timeout_timer;

static void adv_timeout_handler(btstack_timer_source_t *ts);
static void deferred_adv_fallback_handler(btstack_timer_source_t *ts);

// ---------------------------------------------------------------------------
// Advertising data — flags + appearance + HID service UUID (slot identity is by
// address). The device NAME is per-side and longer than fits here alongside the
// rest, so it is carried in the scan response (s_scan_resp) instead.
// ---------------------------------------------------------------------------

static const uint8_t adv_data[] = {
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
    0x03, BLUETOOTH_DATA_TYPE_APPEARANCE, 0xC1, 0x03,
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS,
          ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE & 0xff,
          ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE >> 8,
};

// Scan response — carries the full per-side device name (e.g.
// "CMK-Keeeyboard-Split-L"), which won't fit in the 31-byte adv packet next to
// flags/appearance/UUID. Active scanners (macOS/Windows/iOS pairing UIs) read
// it. Built once at init from board_side_name(); must persist for the program
// lifetime — gap_scan_response_set_data() does NOT copy the buffer.
static uint8_t  s_scan_resp[31];
static uint16_t s_scan_resp_len = 0;

void build_scan_response(void) {
    const char *name = board_side_name();
    size_t nlen = strlen(name);
    if (nlen > 29) nlen = 29;          // 31 - (length byte + AD-type byte)
    s_scan_resp[0] = (uint8_t)(nlen + 1);
    s_scan_resp[1] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(&s_scan_resp[2], name, nlen);
    s_scan_resp_len = (uint16_t)(nlen + 2);
}

// Set the random static address for the given DS slot.
// Must be called before gap_advertisements_set_params / gap_advertisements_enable.
static void set_slot_address(uint8_t slot) {
    if (!g_ble_slot_addrs_ready || slot >= NUM_DS_SLOTS) return;
    g_ble_active_slot = slot;
    gap_random_address_set(g_ble_slot_addr[slot]);
}

// ---------------------------------------------------------------------------
// ADV timeout
// ---------------------------------------------------------------------------

// Advertising timed out — no downstream connected within ADV_TIMEOUT_MS
// Fires for both general ADV and directed ADV low duty (software timeout).
static void adv_timeout_handler(btstack_timer_source_t *ts) {
    UNUSED(ts);
    if (g_ble_periph_state != PERIPH_ADVERTISING &&
        g_ble_periph_state != PERIPH_ADV_DIRECTED) return;  // already connected, ignore
    C1_DBG_STATUS("[PERIPH] Advertising timeout — stopping\n");
    gap_advertisements_enable(0);
    g_ble_periph_state = PERIPH_IDLE;
    ipc_push_to_core0(IPC_ENCODE(IPC_PERIPH_ADV_TIMEOUT, 0));
}

// ---------------------------------------------------------------------------
// Deferred ADV
// ---------------------------------------------------------------------------

// Fallback for deferred ADV — CYW43 sometimes swallows LE_CONNECTION_COMPLETE
// after gap_connect_cancel(), leaving the deferred ADV stuck.
#define DEFERRED_ADV_FALLBACK_MS  500

// Execute pending deferred ADV and cancel fallback timer.
// Called from GAP_SUBEVENT_LE_CONNECTION_COMPLETE when the cancel event arrives.
void execute_deferred_adv(void) {
    btstack_run_loop_remove_timer(&s_deferred_adv_fallback_timer);
    if (s_deferred_adv == DEFERRED_ADV_GENERAL) {
        s_deferred_adv = DEFERRED_ADV_NONE;
        periph_start_advertising(s_deferred_adv_slot);
    } else if (s_deferred_adv == DEFERRED_ADV_DIRECTED) {
        s_deferred_adv = DEFERRED_ADV_NONE;
        periph_start_advertising_directed(s_deferred_adv_slot,
                                          s_deferred_adv_addr,
                                          s_deferred_adv_addr_type);
    }
}

static void deferred_adv_fallback_handler(btstack_timer_source_t *ts) {
    UNUSED(ts);
    if (s_deferred_adv == DEFERRED_ADV_NONE) return;  // already executed normally
    C1_DBG_STATUS("[PERIPH] Deferred ADV fallback (gap_connect_cancel event missing)\n");
    // Force central to IDLE so ADV functions don't re-defer.
    // R4: tell Core 0 too — its upstream state is still CONNECTING and, without
    // this event, the pending US auto-restart could never fire (upstream dead
    // until reboot whenever the CYW43 swallowed the cancel completion).
    g_ble_central_state = CENTRAL_IDLE;
    ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_DISCONNECTED, 0));
    if (s_deferred_adv == DEFERRED_ADV_GENERAL) {
        s_deferred_adv = DEFERRED_ADV_NONE;
        periph_start_advertising(s_deferred_adv_slot);
    } else if (s_deferred_adv == DEFERRED_ADV_DIRECTED) {
        s_deferred_adv = DEFERRED_ADV_NONE;
        periph_start_advertising_directed(s_deferred_adv_slot,
                                          s_deferred_adv_addr,
                                          s_deferred_adv_addr_type);
    }
}

// ---------------------------------------------------------------------------
// Peripheral — advertising
// ---------------------------------------------------------------------------

void periph_start_advertising(uint8_t slot) {
    if (g_ble_periph_state == PERIPH_CONNECTED) {
        C1_DBG_STATUS("[PERIPH] Already connected to downstream — cannot advertise\n");
        // R4: NACK instead of swallowing — Core 0 already advanced to ADVERTISING.
        ipc_push_to_core0(IPC_ENCODE(IPC_CMD_REJECTED, IPC_START_ADV));
        return;
    }
    // CYW43 rejects LE Set Advertising Enable while LE Create Connection is
    // outstanding (Initiating state → error 0x0C Command Disallowed).
    // Cancel the connection attempt first; ADV will start when the cancel completes.
    if (g_ble_central_state == CENTRAL_CONNECTING) {
        s_deferred_adv = DEFERRED_ADV_GENERAL;
        s_deferred_adv_slot = slot;
        gap_connect_cancel();
        // Arm fallback — CYW43 may not fire LE_CONNECTION_COMPLETE after cancel
        btstack_run_loop_remove_timer(&s_deferred_adv_fallback_timer);
        s_deferred_adv_fallback_timer.process = &deferred_adv_fallback_handler;
        btstack_run_loop_set_timer(&s_deferred_adv_fallback_timer, DEFERRED_ADV_FALLBACK_MS);
        btstack_run_loop_add_timer(&s_deferred_adv_fallback_timer);
        C1_DBG_STATUS("[PERIPH] Cancel gap_connect before general ADV — deferred\n");
        return;
    }

    // Set per-slot random static address
    set_slot_address(slot);

    uint16_t adv_int = 0x0030;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int, adv_int, 0, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(sizeof(adv_data), (uint8_t *)adv_data);
    gap_scan_response_set_data(s_scan_resp_len, s_scan_resp);
    gap_advertisements_enable(1);

    // Arm ADV timeout — stops advertising if no downstream connects within ADV_TIMEOUT_MS
    btstack_run_loop_remove_timer(&s_adv_timeout_timer);
    s_adv_timeout_timer.process = &adv_timeout_handler;
    btstack_run_loop_set_timer(&s_adv_timeout_timer, ADV_TIMEOUT_MS);
    btstack_run_loop_add_timer(&s_adv_timeout_timer);

    g_ble_periph_state  = PERIPH_ADVERTISING;
    s_periph_origin = ORIGIN_GENERAL_ADV;
    s_adv_timeout_end = btstack_run_loop_get_time_ms() + ADV_TIMEOUT_MS;
    ipc_push_to_core0(IPC_ENCODE(IPC_PERIPH_ADVERTISING, 0));
    C1_DBG_STATUS("[PERIPH] Advertising as \"%s\" slot %u (timeout %ds)\n",
               board_side_name(), slot, ADV_TIMEOUT_MS / 1000);
}

// ---------------------------------------------------------------------------
// Peripheral — directed advertising (high duty cycle, hardware 1.28s timeout)
// ---------------------------------------------------------------------------

void periph_start_advertising_directed(uint8_t slot, const bd_addr_t peer_addr,
                                       bd_addr_type_t peer_addr_type) {
    if (g_ble_periph_state == PERIPH_CONNECTED) {
        C1_DBG_STATUS("[PERIPH] Already connected — cannot start directed ADV\n");
        ipc_push_to_core0(IPC_ENCODE(IPC_CMD_REJECTED, IPC_START_ADV_DIRECTED));   // R4 NACK
        return;
    }
    // CYW43 rejects LE Set Advertising Enable while LE Create Connection is
    // outstanding — cancel and defer (same as periph_start_advertising).
    if (g_ble_central_state == CENTRAL_CONNECTING) {
        s_deferred_adv = DEFERRED_ADV_DIRECTED;
        s_deferred_adv_slot = slot;
        memcpy(s_deferred_adv_addr, peer_addr, 6);
        s_deferred_adv_addr_type = peer_addr_type;
        gap_connect_cancel();
        // Arm fallback — CYW43 may not fire LE_CONNECTION_COMPLETE after cancel
        btstack_run_loop_remove_timer(&s_deferred_adv_fallback_timer);
        s_deferred_adv_fallback_timer.process = &deferred_adv_fallback_handler;
        btstack_run_loop_set_timer(&s_deferred_adv_fallback_timer, DEFERRED_ADV_FALLBACK_MS);
        btstack_run_loop_add_timer(&s_deferred_adv_fallback_timer);
        C1_DBG_STATUS("[PERIPH] Cancel gap_connect before directed ADV — deferred\n");
        return;
    }

    // Set per-slot random static address (no ADV data for directed ADV)
    set_slot_address(slot);

    // ADV_DIRECT_IND low duty cycle: type=1, interval ~100ms (0x00A0).
    // Low duty gives the peer more time to respond (10s software timeout vs 1.28s hw).
    // ADV_DIRECT_IND PDUs carry NO advertising data — do NOT call
    // gap_advertisements_set_data() here (CYW43 may reject/crash).
    gap_advertisements_set_params(0x00A0, 0x00A0, 0x01,
                                  peer_addr_type, (uint8_t *)peer_addr, 0x07, 0x00);
    gap_advertisements_enable(1);

    // Arm software timeout — same as general ADV
    btstack_run_loop_remove_timer(&s_adv_timeout_timer);
    s_adv_timeout_timer.process = &adv_timeout_handler;
    btstack_run_loop_set_timer(&s_adv_timeout_timer, ADV_TIMEOUT_MS);
    btstack_run_loop_add_timer(&s_adv_timeout_timer);

    g_ble_periph_state  = PERIPH_ADV_DIRECTED;
    s_periph_origin = ORIGIN_DIRECTED_ADV;
    C1_DBG_STATUS("[PERIPH] Directed ADV slot %u → %s (low duty, %ds sw timeout)\n",
               slot, bd_addr_to_str(peer_addr), ADV_TIMEOUT_MS / 1000);
}

// ---------------------------------------------------------------------------
// IPC command bodies (dispatched from ble_manager_handle_ipc)
// ---------------------------------------------------------------------------

// Core 0 rapid-disconnect STOP: enter reject mode.
// Stop any active ADV and reject all incoming peripheral connections
// until a new ADV command (IPC_START_ADV / IPC_START_ADV_DIRECTED) arrives.
void ble_periph_stop_and_reject(void) {
    g_ble_periph_reject = true;
    gap_advertisements_enable(0);
    btstack_run_loop_remove_timer(&s_adv_timeout_timer);
    if (g_ble_periph_handle != HCI_CON_HANDLE_INVALID) {
        gap_disconnect(g_ble_periph_handle);
    }
    if (g_ble_periph_state == PERIPH_ADVERTISING || g_ble_periph_state == PERIPH_ADV_DIRECTED) {
        g_ble_periph_state = PERIPH_IDLE;
    }
    C1_DBG_STATUS("[PERIPH] Reject mode ON — ADV stopped, connections refused\n");
}

// IPC_DISCONNECT_PERIPH (slot switch): drop the downstream link intentionally.
void ble_periph_disconnect_intentional(void) {
    if (g_ble_periph_handle != HCI_CON_HANDLE_INVALID) {
        s_periph_disconnect_intentional = true;
        C1_DBG_STATUS("[PERIPH] Disconnect richiesto da Core 0 (slot switch)\n");
        gap_disconnect(g_ble_periph_handle);
    }
}

// ---------------------------------------------------------------------------
// HCI event hooks (called by the router in ble_manager.c)
// ---------------------------------------------------------------------------

// GAP_SUBEVENT_LE_CONNECTION_COMPLETE, role == SLAVE, status == SUCCESS
void ble_periph_on_connected(hci_con_handle_t handle, uint8_t *pkt) {
    // Reject mode: Core 0 told us to stop accepting connections
    // (rapid-disconnect loop from btstack_tlv_none after reboot).
    // Disconnect immediately without notifying Core 0 to break the loop.
    if (g_ble_periph_reject) {
        gap_disconnect(handle);
        C1_DBG_STATUS("[PERIPH] Reject mode — disconnecting peer immediately\n");
        return;
    }
    g_ble_periph_handle = handle;
    g_ble_periph_state  = PERIPH_CONNECTED;
    // Cancel general ADV timeout (no-op if not armed)
    btstack_run_loop_remove_timer(&s_adv_timeout_timer);
    // Send peer address to Core 0 so it can save to the pending DS slot
    bd_addr_t peer_addr;
    bd_addr_type_t peer_addr_type;
    gap_subevent_le_connection_complete_get_peer_address(pkt, peer_addr);
    peer_addr_type = (bd_addr_type_t)gap_subevent_le_connection_complete_get_peer_address_type(pkt);
    ipc_put_periph_addr(peer_addr, (uint8_t)peer_addr_type);
    // R4: carry the SLOT in the payload (bits 16..23) — Core 0 used to
    // infer it from its own pending slot, which an ADV-timeout race
    // could clear (peer accepted but never saved → wrong LED slot +
    // bonding_store/TLV divergence).
    ipc_push_to_core0(IPC_ENCODE(IPC_PERIPH_CONNECTED,
                                 ((uint32_t)g_ble_active_slot << 16) | (handle & 0xFFFFu)));
    C1_DBG_STATUS("[PERIPH] Downstream connected (handle=0x%04x, addr=%s)\n",
               handle, bd_addr_to_str(peer_addr));
}

// GAP_SUBEVENT_LE_CONNECTION_COMPLETE, role == SLAVE, status != SUCCESS
void ble_periph_on_conn_failed(uint8_t status) {
    // Directed ADV timed out (0x3C) or other failure — no connection
    if (g_ble_periph_state == PERIPH_ADV_DIRECTED) {
        g_ble_periph_state = PERIPH_IDLE;
        ipc_push_to_core0(IPC_ENCODE(IPC_PERIPH_ADV_TIMEOUT, 0));
        C1_DBG_STATUS("[PERIPH] Directed ADV timeout (status=0x%02x)\n", status);
    }
}

// HCI_EVENT_DISCONNECTION_COMPLETE for the downstream connection
void ble_periph_on_disconnected(hci_con_handle_t handle) {
    C1_DBG_STATUS("[PERIPH] Downstream disconnected\n");
    g_ble_periph_handle     = HCI_CON_HANDLE_INVALID;
    g_ble_periph_state      = PERIPH_IDLE;
    g_ble_periph_subscribed = false;
    // Cancel any deferred US reconnect (retry timer + flag)
    ble_central_cancel_deferred_reconnect();

    // R4: flush every per-connection send state. Stale queued reports
    // must not leak into the next session — the first CAN_SEND_NOW after
    // a reconnect (possibly to a DIFFERENT host) would type ghost keys.
    ble_relay_on_periph_disconnect();
    // Split: if our master (the downstream that subscribed matrix_event)
    // dropped, fall back to STANDALONE (resume local typing).
    ble_split_on_periph_disconnect(handle);
#ifdef PIN_LED_CONN
    gpio_put(PIN_LED_CONN, 0);
#endif

    periph_origin_t origin = s_periph_origin;
    s_periph_origin = ORIGIN_NONE;

    if (origin == ORIGIN_GENERAL_ADV && !g_ble_periph_reject
        && !s_periph_disconnect_intentional
        && btstack_run_loop_get_time_ms() < s_adv_timeout_end) {
        // Connection came from general ADV (user-initiated new pairing).
        // Peer connected then disconnected (e.g. encryption failure).
        // Restart general ADV on the same slot so the intended device
        // can still connect within the timeout window.
        C1_DBG_STATUS("[PERIPH] General ADV restart slot %u (%lums left)\n",
                   g_ble_active_slot,
                   (unsigned long)(s_adv_timeout_end - btstack_run_loop_get_time_ms()));
        ipc_push_to_core0(IPC_ENCODE(IPC_PERIPH_DISCONNECTED, 0));
        periph_start_advertising(g_ble_active_slot);
    } else {
        // Directed ADV or reject mode: ensure CYW43 advertising is truly
        // stopped.  Some controllers may auto-resume ADV after disconnect,
        // allowing the peer to reconnect endlessly.
        gap_advertisements_enable(0);
        ipc_push_to_core0(IPC_ENCODE(IPC_PERIPH_DISCONNECTED, 0));
    }

    s_periph_disconnect_intentional = false;
}
