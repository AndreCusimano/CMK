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
// ble_central.c — central/upstream role (Core 1 only, R7 split)
//
// Scanning, connect/reconnect (+timeouts), US-reconnect retry while the DS
// host hasn't subscribed yet, hids_client (HID-over-GATT) handling, upstream
// HID descriptor parsing (hid_parser → g_ble_upstream_map).
//

#include "ble_manager.h"
#include "ble_internal.h"
#include "ipc/ipc_core.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"
#include "hid/hid_parser.h"

#include "btstack.h"
#include "ble/gatt-service/hids_client.h"
#include "hardware/gpio.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Shared state owned by this module (see ble_internal.h)
// ---------------------------------------------------------------------------

central_state_t  g_ble_central_state  = CENTRAL_IDLE;
hci_con_handle_t g_ble_central_handle = HCI_CON_HANDLE_INVALID;

// Upstream HID descriptor — parsed after GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED.
// Used to decode any upstream mouse report format into our standard layout.
hid_report_map_t g_ble_upstream_map;
bool             g_ble_upstream_map_valid = false;

// ---------------------------------------------------------------------------
// Module-private state
// ---------------------------------------------------------------------------

static uint16_t       s_hids_cid = 0;
static bd_addr_t      s_central_addr;
static bd_addr_type_t s_central_addr_type;

static uint8_t s_hid_descriptor_storage[512];

// Timeout timers (BTstack, Core 1)
static btstack_timer_source_t s_scan_timeout_timer;
static btstack_timer_source_t s_connect_timeout_timer;

// US reconnect retry timer — delays central_reconnect() while the peripheral
// connection is active but the host hasn't subscribed to HID yet.
// CYW43 Initiating scanning (gap_connect) starves the peripheral connection's
// GATT service discovery, preventing the host from subscribing.
#define US_RECONNECT_RETRY_MS     500
#define US_RECONNECT_MAX_WAIT_MS  10000
static btstack_timer_source_t s_us_reconnect_retry_timer;
static uint32_t               s_us_reconnect_wait_start = 0;
static bool                   s_us_reconnect_deferred   = false;

static void hids_client_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz);
static void central_connect(void);
static void scan_timeout_handler(btstack_timer_source_t *ts);
static void connect_timeout_handler(btstack_timer_source_t *ts);

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void ble_central_init(void) {
    hids_client_init(s_hid_descriptor_storage, sizeof(s_hid_descriptor_storage));
}

// ---------------------------------------------------------------------------
// Timeout handlers
// ---------------------------------------------------------------------------

// Scan timed out — no upstream found within SCAN_TIMEOUT_MS
static void scan_timeout_handler(btstack_timer_source_t *ts) {
    UNUSED(ts);
    if (g_ble_central_state != CENTRAL_SCANNING) return;  // already connecting/connected, ignore
    C1_DBG_STATUS("[CENTRAL] Scan timeout — stopping\n");
    gap_stop_scan();
    g_ble_central_state = CENTRAL_IDLE;
    ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_SCAN_TIMEOUT, 0));
}

// Connect attempt timed out — upstream didn't respond within CONNECT_TIMEOUT_MS
static void connect_timeout_handler(btstack_timer_source_t *ts) {
    UNUSED(ts);
    if (g_ble_central_state != CENTRAL_CONNECTING) return;
    C1_DBG_STATUS("[CENTRAL] Connect timeout — aborting\n");
    gap_connect_cancel();
    g_ble_central_state = CENTRAL_IDLE;
    ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_CONNECT_TIMEOUT, 0));
}

// ---------------------------------------------------------------------------
// Central — scanning
// ---------------------------------------------------------------------------

void central_start_scanning(void) {
    if (g_ble_central_state != CENTRAL_IDLE) {
        C1_DBG_STATUS("[CENTRAL] Cannot scan: state=%d (not IDLE)\n", g_ble_central_state);
        ipc_push_to_core0(IPC_ENCODE(IPC_CMD_REJECTED, IPC_START_SCAN));   // R4 NACK
        return;
    }
    // Use fixed central address for upstream operations (distinct from DS slot addresses)
    if (g_ble_slot_addrs_ready) gap_random_address_set(g_ble_central_random_addr);
    gap_stop_scan();  // defensive: clear any lingering scan
    gap_set_scan_parameters(0, 0x0060, 0x0030);
    gap_start_scan();

    // Arm SCAN timeout — stops scanning if no upstream found within SCAN_TIMEOUT_MS
    btstack_run_loop_remove_timer(&s_scan_timeout_timer);
    s_scan_timeout_timer.process = &scan_timeout_handler;
    btstack_run_loop_set_timer(&s_scan_timeout_timer, SCAN_TIMEOUT_MS);
    btstack_run_loop_add_timer(&s_scan_timeout_timer);

    g_ble_central_state = CENTRAL_SCANNING;
    ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_SCANNING, 0));
    C1_DBG_STATUS("[CENTRAL] Scanning for HID devices (timeout %ds)...\n", SCAN_TIMEOUT_MS / 1000);
}

static void central_connect(void) {
    // Device found — cancel scan timeout before it fires
    btstack_run_loop_remove_timer(&s_scan_timeout_timer);
    gap_stop_scan();
    // Ensure central address is set (scanning already set it, but be defensive)
    if (g_ble_slot_addrs_ready) gap_random_address_set(g_ble_central_random_addr);
    if (gap_connect(s_central_addr, s_central_addr_type) != ERROR_CODE_SUCCESS) {   // R4
        g_ble_central_state = CENTRAL_IDLE;
        ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_CONNECT_TIMEOUT, 0));
        C1_DBG_STATUS("[CENTRAL] gap_connect rejected synchronously\n");
        return;
    }
    g_ble_central_state = CENTRAL_CONNECTING;
    // Arm connect timeout
    btstack_run_loop_remove_timer(&s_connect_timeout_timer);
    s_connect_timeout_timer.process = &connect_timeout_handler;
    btstack_run_loop_set_timer(&s_connect_timeout_timer, CONNECT_TIMEOUT_MS);
    btstack_run_loop_add_timer(&s_connect_timeout_timer);
    ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_FOUND, 0));
    C1_DBG_STATUS("[CENTRAL] Connecting to %s...\n", bd_addr_to_str(s_central_addr));
}

// Direct reconnect to a previously bonded upstream device (no scan needed).
// Core 0 writes the address to the IPC reconnect-addr slot before sending IPC_RECONNECT_UPSTREAM.
static void us_reconnect_retry_handler(btstack_timer_source_t *ts) {
    UNUSED(ts);
    central_reconnect();
}

void central_reconnect(void) {
    if (g_ble_central_state != CENTRAL_IDLE) {
        C1_DBG_STATUS("[CENTRAL] Cannot reconnect: state=%d (not IDLE)\n", g_ble_central_state);
        ipc_push_to_core0(IPC_ENCODE(IPC_CMD_REJECTED, IPC_RECONNECT_UPSTREAM));   // R4 NACK
        return;
    }
    // Delay US reconnect while DS is connected but not yet subscribed.
    // CYW43 Initiating scanning (gap_connect) starves the peripheral GATT
    // service discovery, preventing the downstream host from subscribing.
    // Retry every 500ms until subscribed or 10s max.
    if (g_ble_periph_state == PERIPH_CONNECTED && !g_ble_periph_subscribed) {
        uint32_t now = btstack_run_loop_get_time_ms();
        if (!s_us_reconnect_deferred) {
            s_us_reconnect_deferred = true;
            s_us_reconnect_wait_start = now;
        }
        if (now - s_us_reconnect_wait_start < US_RECONNECT_MAX_WAIT_MS) {
            btstack_run_loop_remove_timer(&s_us_reconnect_retry_timer);
            s_us_reconnect_retry_timer.process = &us_reconnect_retry_handler;
            btstack_run_loop_set_timer(&s_us_reconnect_retry_timer, US_RECONNECT_RETRY_MS);
            btstack_run_loop_add_timer(&s_us_reconnect_retry_timer);
            C1_DBG_STATUS("[CENTRAL] US reconnect delayed — waiting for DS subscription\n");
            return;
        }
        C1_DBG_STATUS("[CENTRAL] US reconnect max wait reached — proceeding anyway\n");
    }
    s_us_reconnect_deferred = false;
    // Use fixed central address for upstream (distinct from DS slot addresses)
    if (g_ble_slot_addrs_ready) gap_random_address_set(g_ble_central_random_addr);
    uint8_t addr_type_raw;
    ipc_get_reconnect_addr(s_central_addr, &addr_type_raw);
    s_central_addr_type = (bd_addr_type_t)addr_type_raw;
    // R4: a synchronously-rejected gap_connect (0x0C while ADV in a bad ordering,
    // out of memory, ...) produces NO completion event — without this check the
    // state stuck in CONNECTING forever and every later command was refused.
    if (gap_connect(s_central_addr, s_central_addr_type) != ERROR_CODE_SUCCESS) {
        g_ble_central_state = CENTRAL_IDLE;
        ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_CONNECT_TIMEOUT, 0));
        C1_DBG_STATUS("[CENTRAL] gap_connect rejected synchronously\n");
        return;
    }
    g_ble_central_state = CENTRAL_CONNECTING;
    // No timeout — controller keeps trying forever until upstream appears
    ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_FOUND, 0));
    C1_DBG_STATUS("[CENTRAL] Reconnecting to %s (forever, no timeout)...\n",
               bd_addr_to_str(s_central_addr));
}

// DS subscribed — if US reconnect was deferred, trigger it now.
void ble_central_resume_deferred_reconnect(void) {
    if (s_us_reconnect_deferred) {
        btstack_run_loop_remove_timer(&s_us_reconnect_retry_timer);
        s_us_reconnect_deferred = false;
        central_reconnect();
    }
}

// Cancel a deferred US reconnect (periph disconnect cleanup).
void ble_central_cancel_deferred_reconnect(void) {
    btstack_run_loop_remove_timer(&s_us_reconnect_retry_timer);
    s_us_reconnect_deferred = false;
}

// Start HID-over-GATT discovery on the connected upstream.
// Called after pairing complete / re-encryption complete (SM handler).
void ble_central_start_hid_discovery(void) {
    hids_client_connect(g_ble_central_handle, hids_client_handler,
                        HID_PROTOCOL_MODE_REPORT, &s_hids_cid);
}

// ---------------------------------------------------------------------------
// IPC command body (dispatched from ble_manager_handle_ipc)
// ---------------------------------------------------------------------------

// R4: explicit upstream teardown (clear US bond + rescan, factory reset).
// Kill the reconnect retry timer FIRST — it would otherwise fire later
// and gap_connect() to a bond that was just purged.
void ble_central_disconnect(void) {
    btstack_run_loop_remove_timer(&s_us_reconnect_retry_timer);
    s_us_reconnect_deferred = false;
    if (g_ble_central_handle != HCI_CON_HANDLE_INVALID) {
        // Connected (DISCOVERING/CONNECTED): drop the HID client first (avoids
        // a context leak), then the link. The HCI disconnect handler notifies
        // Core 0 with IPC_CENTRAL_DISCONNECTED.
        if (s_hids_cid != 0) { hids_client_disconnect(s_hids_cid); s_hids_cid = 0; }
        gap_disconnect(g_ble_central_handle);
        C1_DBG_STATUS("[CENTRAL] Disconnect richiesto da Core 0\n");
    } else if (g_ble_central_state == CENTRAL_CONNECTING) {
        gap_connect_cancel();
        g_ble_central_state = CENTRAL_IDLE;
        // The CYW43 may swallow the cancel completion — notify Core 0 directly.
        ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_DISCONNECTED, 0));
        C1_DBG_STATUS("[CENTRAL] Connect cancellato da Core 0\n");
    } else if (g_ble_central_state == CENTRAL_SCANNING) {
        gap_stop_scan();
        btstack_run_loop_remove_timer(&s_scan_timeout_timer);
        g_ble_central_state = CENTRAL_IDLE;
        ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_DISCONNECTED, 0));
    }
}

// ---------------------------------------------------------------------------
// HCI event hooks (called by the router in ble_manager.c)
// ---------------------------------------------------------------------------

// GAP_SUBEVENT_LE_CONNECTION_COMPLETE, role == MASTER, status == SUCCESS
void ble_central_on_connected(hci_con_handle_t handle) {
    g_ble_central_handle = handle;
    g_ble_central_state  = CENTRAL_DISCOVERING;
    // Cancel connect timeout — upstream responded
    btstack_run_loop_remove_timer(&s_connect_timeout_timer);
    // Note: advertising is NOT paused — CYW43 handles concurrent adv + GATT.
    // Pausing caused a state inconsistency bug where advertising could
    // get stuck disabled when periph_start_advertising() saw PERIPH_ADVERTISING
    // and returned early without calling gap_advertisements_enable(1).
    C1_DBG_STATUS("[CENTRAL] Upstream connected (handle=0x%04x) — pairing...\n", handle);
    sm_request_pairing(handle);
    // gap_connect_cancel() raced with connection completion — the cancel
    // didn't take effect.  Execute deferred ADV now (CYW43 supports
    // concurrent ADV + established GATT connections).
    execute_deferred_adv();
}

// GAP_SUBEVENT_LE_CONNECTION_COMPLETE, role == MASTER, status != SUCCESS
// Central connection attempt failed or was cancelled (e.g. gap_connect_cancel
// returns status=0x02 Unknown_Connection_Identifier).
void ble_central_on_conn_failed(uint8_t status) {
    g_ble_central_handle = HCI_CON_HANDLE_INVALID;
    g_ble_central_state  = CENTRAL_IDLE;
    btstack_run_loop_remove_timer(&s_connect_timeout_timer);
    ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_DISCONNECTED, 0));
    C1_DBG_STATUS("[CENTRAL] Connect failed/cancelled (status=0x%02x)\n", status);
    // If advertising was deferred because of the Initiating state, start it now
    execute_deferred_adv();
}

// HCI_EVENT_DISCONNECTION_COMPLETE for the upstream connection
void ble_central_on_disconnected(uint8_t reason) {
    C1_DBG_STATUS("[CENTRAL] Upstream disconnected (reason=0x%02x)\n", reason);
    g_ble_central_handle     = HCI_CON_HANDLE_INVALID;
    s_hids_cid               = 0;
    g_ble_central_state      = CENTRAL_IDLE;
    g_ble_upstream_map_valid = false;
    // Split: the upstream was our slave → abort discovery and fall back to
    // STANDALONE (Milestone D will also release any stale remote keys).
    ble_split_on_central_disconnect();
#ifdef PIN_LED_RELAY
    gpio_put(PIN_LED_RELAY, 0);
#endif
    ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_DISCONNECTED, 0));

    // Release any keys that were held by upstream — zero upstream state and
    // re-send the now local-only report.
    ble_relay_on_central_disconnect();
    // Press FN+GP13 to reconnect upstream
}

// GAP_EVENT_ADVERTISING_REPORT — pick the first HID device found
void ble_central_on_adv_report(uint8_t *pkt) {
    if (g_ble_central_state != CENTRAL_SCANNING) return;
    const uint8_t *ad_data = gap_event_advertising_report_get_data(pkt);
    uint8_t        ad_len  = gap_event_advertising_report_get_data_length(pkt);
    if (!ad_data_contains_uuid16(ad_len, ad_data,
                                 ORG_BLUETOOTH_SERVICE_HUMAN_INTERFACE_DEVICE)) return;
    gap_event_advertising_report_get_address(pkt, s_central_addr);
    s_central_addr_type = gap_event_advertising_report_get_address_type(pkt);
    C1_DBG_STATUS("[CENTRAL] Found HID device: %s\n", bd_addr_to_str(s_central_addr));
    central_connect();
}

// ---------------------------------------------------------------------------
// HIDS Client handler (central — upstream)
// ---------------------------------------------------------------------------

static void hids_client_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz) {
    UNUSED(ch); UNUSED(sz); UNUSED(ptype);
    // NOTE: hids_client calls this handler with ptype=HCI_EVENT_PACKET for connection events
    // but with ptype=HCI_EVENT_GATTSERVICE_META for notifications (handle_notification_event).
    // Do NOT filter on ptype — check only the packet content.
    if (hci_event_packet_get_type(pkt) != HCI_EVENT_GATTSERVICE_META) return;

    switch (hci_event_gattservice_meta_get_subevent_code(pkt)) {

    case GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED: {
        uint8_t status = gattservice_subevent_hid_service_connected_get_status(pkt);
        if (status == ERROR_CODE_SUCCESS) {
            uint8_t num = gattservice_subevent_hid_service_connected_get_num_instances(pkt);
            C1_DBG_STATUS("[CENTRAL] HID service ready (%d instance%s)\n", num, num == 1 ? "" : "s");

            // Parse upstream HID descriptor so the mouse relay can handle any format
            g_ble_upstream_map_valid = false;
            const uint8_t *up_desc = hids_client_descriptor_storage_get_descriptor_data(s_hids_cid, 0);
            uint16_t       up_len  = hids_client_descriptor_storage_get_descriptor_len(s_hids_cid, 0);
            if (up_desc && up_len > 0) {
                int n = hid_parse_descriptor(up_desc, up_len, &g_ble_upstream_map);
                g_ble_upstream_map_valid = (n >= 0);
                C1_DBG_STATUS("[CENTRAL] Upstream HID descriptor: %d bytes → %d Input fields\n", up_len, n);
                // One-shot field-map dump — pinpoints how THIS upstream lays out
                // buttons/axes when a relay symptom needs HW diagnosis.
                for (uint8_t i = 0; g_ble_upstream_map_valid && i < g_ble_upstream_map.count; i++) {
                    const hid_field_t *f = &g_ble_upstream_map.fields[i];
                    C1_DBG_STATUS("[CENTRAL]  f%u: id=%u off=%u bits=%u usage=%04x:%04x..%04x log=[%d,%d]\n",
                                  i, f->report_id, (unsigned)f->bit_offset, f->bit_count,
                                  HID_USAGE_PAGE(f->usage_first), HID_USAGE_ID(f->usage_first),
                                  HID_USAGE_ID(f->usage_last),
                                  (int)f->logical_min, (int)f->logical_max);
                }
            } else {
                C1_DBG_STATUS("[CENTRAL] Upstream HID descriptor unavailable — raw relay fallback\n");
            }

            g_ble_central_state = CENTRAL_CONNECTED;
#ifdef PIN_LED_RELAY
            gpio_put(PIN_LED_RELAY, 1);
#endif
            ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_CONNECTED, s_hids_cid));
            // Send upstream address to Core 0 so it can persist to flash for auto-reconnect
            ipc_put_upstream_addr(s_central_addr, (uint8_t)s_central_addr_type);

            // Phase 18: probe the upstream for the CMK-Split service. If present,
            // it is a sibling half → we become MASTER and subscribe to its matrix
            // events. The hids_client has finished its discovery, so the gatt_client
            // is free now. (No-op for a 3rd-party HID upstream → stay STANDALONE.)
            split_start_discovery();
        } else {
            C1_DBG_STATUS("[CENTRAL] HID connect failed (0x%02x)\n", status);
            if (s_hids_cid != 0) { hids_client_disconnect(s_hids_cid); s_hids_cid = 0; }
            gap_disconnect(g_ble_central_handle);
        }
        break;
    }

    case GATTSERVICE_SUBEVENT_HID_SERVICE_REPORTS_NOTIFICATION:
        C1_DBG_STATUS("[CENTRAL] Notifications config: value=0x%02x\n",
               gattservice_subevent_hid_service_reports_notification_get_configuration(pkt));
        break;

    case GATTSERVICE_SUBEVENT_HID_SERVICE_DISCONNECTED:
        // The HCI disconnect handler already reset state and sent IPC_CENTRAL_DISCONNECTED.
        // Only act here if HCI handler hasn't fired yet (g_ble_central_handle still valid).
        if (g_ble_central_handle != HCI_CON_HANDLE_INVALID || s_hids_cid != 0) {
            C1_DBG_STATUS("[CENTRAL] HID service disconnected\n");
            s_hids_cid           = 0;
            g_ble_central_handle = HCI_CON_HANDLE_INVALID;
            g_ble_central_state  = CENTRAL_IDLE;
#ifdef PIN_LED_RELAY
            gpio_put(PIN_LED_RELAY, 0);
#endif
            ipc_push_to_core0(IPC_ENCODE(IPC_CENTRAL_DISCONNECTED, 0));
        }
        break;

    case GATTSERVICE_SUBEVENT_HID_REPORT: {
        const uint8_t *report     = gattservice_subevent_hid_report_get_report(pkt);
        uint16_t       report_len = gattservice_subevent_hid_report_get_report_len(pkt);
        uint8_t        report_id  = gattservice_subevent_hid_report_get_report_id(pkt);

        // BTstack hids_client prepends the Report ID as report[0].
        // hids_device_send_input_report_for_id() sends raw data WITHOUT Report ID prefix
        // (the downstream BLE host knows the Report ID from the GATT characteristic's
        // Report Reference descriptor). Strip byte[0] before relaying.
        const uint8_t *relay_data = (report_len > 0 && report[0] == report_id)
                                    ? report + 1 : report;
        uint16_t       relay_len  = (report_len > 0 && report[0] == report_id)
                                    ? report_len - 1 : report_len;

        // Merge/relay/USB-forward — see ble_relay.c
        ble_relay_on_upstream_report(report_id, relay_data, relay_len);
        break;
    }

    default:
        C1_DBG_STATUS("[CENTRAL] Unhandled GATTSERVICE subevent: 0x%02x\n",
               hci_event_gattservice_meta_get_subevent_code(pkt));
        break;
    }
}
