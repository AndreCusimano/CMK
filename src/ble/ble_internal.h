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
// ble_internal.h — INTERNAL header shared by the BLE manager modules (R7 split).
//
// NOT a public API: only the src/ble/*.c modules may include this.
//
// ble_manager.c was split into single-responsibility modules:
//   ble_manager.c — init, HCI/SM event ROUTING, IPC dispatch, ATT callbacks
//   ble_periph.c  — peripheral/downstream: advertising (general/directed/deferred)
//   ble_central.c — central/upstream: scan, connect/reconnect, hids_client
//   ble_split.c   — CMK-Split GATT service + discovery (Phase 18)
//   ble_relay.c   — merge/relay/output: kbd merge, mouse relay, CAN_SEND_NOW
//
// CONCURRENCY NOTE: everything declared here runs on Core 1 ONLY, inside the
// single-threaded BTstack run loop. Event handlers, timers and the IPC dispatch
// never preempt each other, so NO locking is needed for any of this shared
// state. Cross-core traffic goes exclusively through ipc_core (FIFO + slots).
//
// Naming: state shared between modules uses the g_ble_ prefix; state used by a
// single module stays static (s_ prefix) in that module.
//

#ifndef CMK_BLE_INTERNAL_H
#define CMK_BLE_INTERNAL_H

#include <stdint.h>
#include <stdbool.h>

#include "btstack.h"
#include "ble_manager.h"
#include "hid/hid_parser.h"

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

#define NUM_DS_SLOTS  3

// Per-slot random static addresses — derived from BD_ADDR at HCI_STATE_WORKING
// (derivation owned by ble_manager.c; periph/central apply them to the controller).
extern bd_addr_t g_ble_slot_addr[NUM_DS_SLOTS];  // one random static addr per DS slot
extern bd_addr_t g_ble_central_random_addr;      // fixed random static addr for central role (upstream)
extern bool      g_ble_slot_addrs_ready;
extern uint8_t   g_ble_active_slot;              // which slot is currently set for ADV (owned by ble_periph.c)

// Peripheral / downstream — owned by ble_periph.c
extern periph_state_t   g_ble_periph_state;
extern hci_con_handle_t g_ble_periph_handle;
extern bool             g_ble_periph_subscribed;
// When true, Core 1 immediately disconnects any incoming peripheral connection
// and suppresses advertising.  Set by IPC_STOP_ADV (Core 0 rapid-disconnect STOP).
// Cleared by any new ADV command (IPC_START_ADV, IPC_START_ADV_DIRECTED).
extern bool             g_ble_periph_reject;

// Central / upstream — owned by ble_central.c
extern central_state_t  g_ble_central_state;
extern hci_con_handle_t g_ble_central_handle;

// Upstream HID descriptor map — parsed by ble_central.c after
// GATTSERVICE_SUBEVENT_HID_SERVICE_CONNECTED; used by ble_relay.c to decode
// any upstream mouse report format into our standard layout.
extern hid_report_map_t g_ble_upstream_map;
extern bool             g_ble_upstream_map_valid;

// ---------------------------------------------------------------------------
// ble_periph.c — peripheral/downstream (advertising)
// ---------------------------------------------------------------------------

void build_scan_response(void);             // bake per-side device name (call once at init)
void periph_start_advertising(uint8_t slot);
void periph_start_advertising_directed(uint8_t slot, const bd_addr_t peer_addr,
                                       bd_addr_type_t peer_addr_type);
// Execute pending deferred ADV and cancel its fallback timer (called from the
// central GAP_SUBEVENT_LE_CONNECTION_COMPLETE paths).
void execute_deferred_adv(void);
void ble_periph_stop_and_reject(void);      // IPC_STOP_ADV body
void ble_periph_disconnect_intentional(void); // IPC_DISCONNECT_PERIPH body
// HCI event router hooks
void ble_periph_on_connected(hci_con_handle_t handle, uint8_t *pkt);
void ble_periph_on_conn_failed(uint8_t status);
void ble_periph_on_disconnected(hci_con_handle_t handle);

// ---------------------------------------------------------------------------
// ble_central.c — central/upstream (scan/connect/hids_client)
// ---------------------------------------------------------------------------

void ble_central_init(void);                // hids_client_init with descriptor storage
void central_start_scanning(void);
void central_reconnect(void);
// Start HID-over-GATT discovery on the connected upstream (hids_client_connect).
void ble_central_start_hid_discovery(void);
// DS subscribed → if a US reconnect was deferred, trigger it now.
void ble_central_resume_deferred_reconnect(void);
// Cancel a deferred US reconnect (retry timer + flag) — periph disconnect cleanup.
void ble_central_cancel_deferred_reconnect(void);
void ble_central_disconnect(void);          // IPC_DISCONNECT_CENTRAL body
// HCI event router hooks
void ble_central_on_connected(hci_con_handle_t handle);
void ble_central_on_conn_failed(uint8_t status);
void ble_central_on_disconnected(uint8_t reason);
void ble_central_on_adv_report(uint8_t *pkt);

// ---------------------------------------------------------------------------
// ble_split.c — CMK-Split (Phase 18)
// ---------------------------------------------------------------------------

// ATT split lane handler + maintenance timer. The CMK-Split ATT handles come
// from the generated hid_keyboard.h, which DEFINES profile_data and so can
// only be included by ble_manager.c — the handles are wired in here at init.
void ble_split_init(uint16_t evt_value_handle, uint16_t evt_cccd_handle,
                    uint16_t snap_value_handle, uint16_t side_value_handle,
                    uint16_t mode_value_handle, uint16_t mode_cccd_handle,
                    uint16_t rgb_value_handle);
// Phase 24: MASTER pushes a staged RGB sync frame to the slave (IPC_SEND_RGB_SYNC).
// Reads the frame from ipc_get_rgb_sync_tx and writes it over rgb_sync (write
// without response, retried on can-write so HID reports keep buffer priority).
void ble_split_send_rgb_sync(void);
// Phase 24b: MASTER forwards one local key hit (packed CMK_RGB_HIT) to the slave.
void ble_split_send_rgb_hit(uint8_t packed);
uint8_t ble_split_role_value(void);         // current split role as uint8_t (for status snapshot)
void split_start_discovery(void);           // sibling probe after upstream HID ready (always runs)
void split_tx_enqueue(uint8_t evt);         // SLAVE: queue matrix event + update snapshot
void ble_split_set_mode(bool en);           // IPC_SET_SPLIT_MODE body (hot-switch, no disconnect)
// Strict split-mode matching (used by ble_relay.c to gate the sibling's HID):
// peer_is_cmk = the upstream exposes the CMK-Split service (probed); states_match
// = its split_mode equals ours. Mismatch ⇒ drop the sibling's HID reports.
bool ble_split_peer_is_cmk(void);
bool ble_split_states_match(void);
// ATT callback hooks (called from ble_manager.c att_read_cb / att_write_cb):
// read returns 0 if the handle is not a CMK-Split characteristic.
uint16_t ble_split_att_read(uint16_t attribute_handle, uint16_t offset,
                            uint8_t *buffer, uint16_t buffer_size);
// write returns -1 if the handle is not a CMK-Split CCCD (caller falls through),
// 0 on success, or an ATT error code (write rejected at the ATT level).
int ble_split_att_write(hci_con_handle_t con_handle, uint16_t attribute_handle,
                        uint8_t *buffer, uint16_t buffer_size);
// Disconnect hooks
void ble_split_on_periph_disconnect(hci_con_handle_t handle);
void ble_split_on_central_disconnect(void);

// ---------------------------------------------------------------------------
// ble_relay.c — merge/relay/output
// ---------------------------------------------------------------------------

void ble_relay_init(void);                  // hids_device packet handler + HID log timer
void kbd_drain_ipc_ring(void);
void mouse_slot_take(void);
void pb_slot_take(void);
// Upstream HID report (Report ID already stripped) → merge/relay/USB-forward.
void ble_relay_on_upstream_report(uint8_t report_id, const uint8_t *data, uint16_t len);
// R4: flush every per-connection send state on downstream disconnect.
void ble_relay_on_periph_disconnect(void);
// Release any keys that were held by upstream (zero + re-send merged/USB).
void ble_relay_on_central_disconnect(void);

#endif // CMK_BLE_INTERNAL_H
