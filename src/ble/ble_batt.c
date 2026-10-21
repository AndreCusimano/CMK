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
// ble_batt.c — battery levels seen over BLE (bug #3, Core 1 only)
//
// Three SoC values, 0..100, 0xFF = unknown:
//   local — this half's fuel gauge (IPC_SET_BATTERY_LEVEL from the CM task)
//   peer  — the CMK sibling's, polled by ble_split.c over peer_batt (c3a50007)
//   mouse — a 3rd-party upstream's, via BTstack's battery_service_client
//
// The PC sees ONE Battery Service (the stock server, first 0x180F instance):
// it carries min(local, peer), so a draining SLAVE is no longer invisible
// behind a full MASTER. peer/mouse also reach Core 0 via the R3 status slot
// for the config protocol (GET_BATTERY). Deliberately no extra BAS instances:
// host OSes show only the first, and in a split the mouse hangs off the SLAVE,
// whose GATT the PC never reads.
//

#include "ble_internal.h"
#include "ble/gatt-service/battery_service_server.h"
#include "ble/gatt-service/battery_service_client.h"
#include "ipc/ipc_core.h"          // core1_log — required by C1_DBG_*
#include "config/debug_config.h"

// Mouse BAS poll period, used only when the mouse cannot notify.
#define BAS_CLIENT_POLL_MS 60000

static uint8_t  s_local_batt = 0xFF;
static uint8_t  s_peer_batt  = 0xFF;
static uint8_t  s_mouse_batt = 0xFF;
static uint16_t s_bas_cid    = 0;

static void bas_client_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz);

static uint8_t soc_clamp(uint8_t soc) {
    return (soc != 0xFF && soc > 100) ? 100 : soc;
}

// BAS #1 = min(local, peer), 0xFF-aware. Nothing known → keep the last value.
static void bas1_refresh(void) {
    uint8_t v = s_local_batt;
    if (s_peer_batt != 0xFF && (v == 0xFF || s_peer_batt < v)) v = s_peer_batt;
    if (v == 0xFF) return;
    battery_service_server_set_battery_value(v);
}

void ble_batt_init(void) {
    battery_service_client_init();
}

void ble_batt_set_local(uint8_t soc) {
    soc = soc_clamp(soc);
    if (soc == s_local_batt) return;
    s_local_batt = soc;
    bas1_refresh();
}

void ble_batt_set_peer(uint8_t soc) {
    soc = soc_clamp(soc);
    if (soc == s_peer_batt) return;
    s_peer_batt = soc;
    C1_DBG_STATUS("[BATT] peer=%u%%\n", soc);
    bas1_refresh();   // peer gone (0xFF) → BAS #1 falls back to the local SoC
}

uint8_t ble_batt_local_value(void) { return s_local_batt; }
uint8_t ble_batt_peer_value(void)  { return s_peer_batt;  }
uint8_t ble_batt_mouse_value(void) { return s_mouse_batt; }

// ---------------------------------------------------------------------------
// Upstream mouse (3rd-party) — battery_service_client
// ---------------------------------------------------------------------------

// The split probe reached a definitive answer. A CMK sibling is covered by
// peer_batt; a 3rd-party upstream may expose the standard Battery Service.
// The probe is over, so the gatt_client is free (the BAS client queues its
// queries anyway).
void ble_batt_on_upstream_probed(bool peer_is_cmk) {
    if (peer_is_cmk || s_bas_cid != 0) return;
    if (g_ble_central_handle == HCI_CON_HANDLE_INVALID) return;
    uint8_t st = battery_service_client_connect(g_ble_central_handle, bas_client_handler,
                                                BAS_CLIENT_POLL_MS, &s_bas_cid);
    if (st != ERROR_CODE_SUCCESS) {
        s_bas_cid = 0;   // best-effort: no battery from this upstream
        C1_DBG_STATUS("[BATT] mouse BAS client connect failed 0x%02x\n", st);
    }
}

// Upstream gone (or being dropped): release the client, forget both levels.
// The BAS client also finalizes itself on HCI disconnect; a second
// disconnect on a dead cid is a harmless UNKNOWN_CONNECTION_IDENTIFIER.
void ble_batt_on_central_disconnect(void) {
    if (s_bas_cid != 0) { battery_service_client_disconnect(s_bas_cid); s_bas_cid = 0; }
    s_mouse_batt = 0xFF;
    ble_batt_set_peer(0xFF);
}

static void bas_client_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz) {
    UNUSED(ptype); UNUSED(ch); UNUSED(sz);
    if (hci_event_packet_get_type(pkt) != HCI_EVENT_GATTSERVICE_META) return;

    switch (hci_event_gattservice_meta_get_subevent_code(pkt)) {

    case GATTSERVICE_SUBEVENT_BATTERY_SERVICE_CONNECTED: {
        uint8_t status = gattservice_subevent_battery_service_connected_get_status(pkt);
        if (status != ERROR_CODE_SUCCESS) {
            // No Battery Service on the upstream: the client finalized itself.
            C1_DBG_STATUS("[BATT] upstream has no BAS (0x%02x)\n", status);
            s_bas_cid = 0;
        } else {
            C1_DBG_STATUS("[BATT] upstream BAS ready (%u instance(s))\n",
                          gattservice_subevent_battery_service_connected_get_num_instances(pkt));
        }
        break;
    }

    case GATTSERVICE_SUBEVENT_BATTERY_SERVICE_LEVEL: {
        // First instance only — a mouse with several BAS instances is exotic.
        if (gattservice_subevent_battery_service_level_get_att_status(pkt) != ATT_ERROR_SUCCESS) break;
        if (gattservice_subevent_battery_service_level_get_service_index(pkt) != 0) break;
        uint8_t soc = soc_clamp(gattservice_subevent_battery_service_level_get_level(pkt));
        if (soc != s_mouse_batt) {
            s_mouse_batt = soc;
            C1_DBG_STATUS("[BATT] mouse=%u%%\n", soc);
        }
        break;
    }

    default:
        break;
    }
}
