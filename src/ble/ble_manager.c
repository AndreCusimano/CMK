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
// ble_manager.c — BLE dual-capability manager (Core 1 only)
//
// R7: this file is now the entry point / ROUTER of the BLE stack glue.
// It owns init (ble_manager_init), the HCI and SM event handlers (which
// dispatch to the role modules), the IPC command dispatch and the ATT
// server callbacks. The actual role logic lives in:
//   ble_periph.c  — advertising / downstream connection
//   ble_central.c — scanning / upstream connection / hids_client
//   ble_split.c   — CMK-Split transport (Phase 18)
//   ble_relay.c   — keyboard merge, mouse relay, CAN_SEND_NOW chain
// Shared state and cross-module hooks: ble_internal.h.
//

#include "ble_manager.h"
#include "ble_internal.h"
#include "ipc/ipc_core.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#include "btstack.h"
#include "ble/gatt-service/battery_service_server.h"
#include "ble/gatt-service/device_information_service_server.h"
#include "ble/gatt-service/hids_device.h"

#include "hid_keyboard.h"
#include "board_side.h"

// cyw43 driver — fallback MAC source when gap_local_bd_addr() returns all-zero
// (degraded BD_ADDR read at HCI_STATE_WORKING). Core 1 owns cyw43_arch.
#include "pico/cyw43_arch.h"

#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// HID report descriptor — composite: keyboard (Report ID 1) + mouse (Report ID 2)
// ---------------------------------------------------------------------------

static const uint8_t hid_descriptor[] = {
    // --- Keyboard — Report ID 1 (8 bytes: modifier, reserved, 6 keycodes) ---
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xa1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    // Modifier keys: 8 bits
    0x75, 0x01, 0x95, 0x08, 0x05, 0x07, 0x19, 0xe0,
    0x29, 0xe7, 0x15, 0x00, 0x25, 0x01, 0x81, 0x02,
    // Reserved byte
    0x75, 0x08, 0x95, 0x01, 0x81, 0x03,
    // LED output (5 bits + 3 pad)
    0x95, 0x05, 0x75, 0x01, 0x05, 0x08, 0x19, 0x01,
    0x29, 0x05, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x03,
    // Keycodes: 6 bytes
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0xff,
    0x05, 0x07, 0x19, 0x00, 0x29, 0xff, 0x81, 0x00,
    0xc0,              // End Collection

    // --- Mouse — Report ID 2 (7 bytes: buttons, X int16, Y int16, wheel int8, pan int8) ---
    //
    // Byte 0  Buttons  (bit0=L, bit1=R, bit2=M, bit3=Back, bit4=Fwd)
    // Byte 1  X_L
    // Byte 2  X_H      (int16 LE, signed, relative)
    // Byte 3  Y_L
    // Byte 4  Y_H      (int16 LE, signed, relative)
    // Byte 5  Wheel    (int8, vertical scroll)
    // Byte 6  Pan      (int8, horizontal scroll — Consumer AC Pan)
    //
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xa1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xa1, 0x00,        //   Collection (Physical)
    0x85, 0x02,        //   Report ID (2)
    // Buttons: 5 bits (L, R, M, Back, Fwd)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (Button 1)
    0x29, 0x05,        //   Usage Maximum (Button 5)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x05,        //   Report Count (5)
    0x81, 0x02,        //   Input (Data, Variable, Absolute)
    // Padding: 3 bits → total 8 bits = 1 byte for buttons
    0x75, 0x03,        //   Report Size (3)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x03,        //   Input (Constant)
    // X, Y: 16-bit signed relative
    0x05, 0x01,        //   Usage Page (Generic Desktop)
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x16, 0x01, 0x80,  //   Logical Minimum (-32767)
    0x26, 0xff, 0x7f,  //   Logical Maximum (32767)
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x06,        //   Input (Data, Variable, Relative)
    // Wheel: 8-bit signed (vertical scroll)
    0x09, 0x38,        //   Usage (Wheel)
    0x15, 0x81,        //   Logical Minimum (-127)
    0x25, 0x7f,        //   Logical Maximum (127)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x06,        //   Input (Data, Variable, Relative)
    // Pan: 8-bit signed (horizontal scroll — Consumer AC Pan 0x0238)
    0x05, 0x0c,        //   Usage Page (Consumer)
    0x0a, 0x38, 0x02,  //   Usage (AC Pan = 0x0238)
    0x15, 0x81,        //   Logical Minimum (-127)
    0x25, 0x7f,        //   Logical Maximum (127)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x06,        //   Input (Data, Variable, Relative)
    0xc0,              //   End Collection (Physical)
    0xc0,              // End Collection (Application)

    // --- Programmable Buttons — Report ID 3 (4 bytes: 32-bit bitmask) ---
    //
    // Declared on a VENDOR-DEFINED usage page (0xFF00), NOT the Button page (0x09).
    // macOS HID parsing dropped the MOUSE's buttons (Report ID 2) whenever a SECOND
    // Button-page usage range appeared elsewhere in the same composite descriptor:
    // pointer movement still worked (Generic Desktop X/Y are unambiguous) but clicks
    // were silently ignored — verified on HW (mouse worked direct, but not relayed
    // through us). Both reference composites that DO work on macOS (the 3rd-party
    // mouse itself, and ESP32 BLE keyboard+mouse builds) keep their auxiliary report
    // on a vendor page. Keeping these host-interpreted "programmable" buttons out of
    // the standard Button namespace leaves the mouse's Button 1..5 unambiguous.
    0x06, 0x00, 0xFF,  // Usage Page (Vendor-Defined 0xFF00)
    0x09, 0x01,        // Usage (Vendor 0x01)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x03,        //   Report ID (3)
    0x19, 0x01,        //   Usage Minimum (1)
    0x29, 0x20,        //   Usage Maximum (32)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x95, 0x20,        //   Report Count (32)
    0x75, 0x01,        //   Report Size (1)
    0x81, 0x02,        //   Input (Data, Var, Abs)
    0xC0,              // End Collection
};

// ---------------------------------------------------------------------------
// Per-slot random static addresses — derived from BD_ADDR at HCI_STATE_WORKING
// ---------------------------------------------------------------------------

bd_addr_t g_ble_slot_addr[NUM_DS_SLOTS];   // one random static addr per DS slot
bd_addr_t g_ble_central_random_addr;       // fixed random static addr for central role (upstream)
bool      g_ble_slot_addrs_ready = false;

// Storage for composite HID device reports (3× Input + 1× Output + 1× Feature)
static hids_device_report_t s_hid_report_storage[5];

static btstack_packet_callback_registration_t s_hci_cb_reg;
static btstack_packet_callback_registration_t s_sm_cb_reg;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

static void hci_event_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz);
static void sm_event_handler (uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz);

// ---------------------------------------------------------------------------
// ATT diagnostic callbacks — log GATT reads/writes, return 0 to let BTstack handle
// ---------------------------------------------------------------------------

// ATT read callback — return 0 = not handled, BTstack uses built-in / hids_device storage.
// MUST be non-NULL: BTstack skips dynamic attribute handling when the callback pointer
// is NULL, causing ATT errors that break Windows BLE HID setup.
static uint16_t att_read_cb(hci_con_handle_t con_handle, uint16_t attribute_handle,
                            uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    (void)con_handle;
    // GAP Device Name is DYNAMIC (per-side, decided at boot) — serve it here.
    if (attribute_handle == ATT_CHARACTERISTIC_GAP_DEVICE_NAME_01_VALUE_HANDLE) {
        const char *name = board_side_name();
        return att_read_callback_handle_blob((const uint8_t *)name, (uint16_t)strlen(name),
                                             offset, buffer, buffer_size);
    }
    // CMK-Split characteristics (Phase 18): side_id / matrix_snapshot.
    {
        uint16_t split_res = ble_split_att_read(attribute_handle, offset, buffer, buffer_size);
        if (split_res) return split_res;
    }
    return 0;
}

// ATT write callback — return 0 = success.
// MUST be non-NULL: same reason as att_read_cb.  Without this, Windows aborts
// HID setup because writes to HID Control Point / Protocol Mode get ATT errors.
//
// Also intercepts Output Report writes (1 byte LED indicator state from host).
static int att_write_cb(hci_con_handle_t con_handle, uint16_t attribute_handle,
                        uint16_t transaction_mode, uint16_t offset,
                        uint8_t *buffer, uint16_t buffer_size) {
    (void)transaction_mode; (void)offset;

    // CMK-Split (Phase 18): matrix_event / split_mode CCCD writes → role change /
    // mode subscriber. >= 0 means handled (0 = ok, >0 = ATT error → write rejected,
    // e.g. a matrix_event subscribe while our split_mode is OFF — strict matching).
    {
        int split_res = ble_split_att_write(con_handle, attribute_handle, buffer, buffer_size);
        if (split_res >= 0) return split_res;
    }

    (void)con_handle;

    // Output Report: host sends 1 byte with LED indicator bits
    // (Num Lock=bit0, Caps Lock=bit1, Scroll Lock=bit2)
    // HID Control Point also writes 1 byte (0x00 suspend / 0x01 exit suspend),
    // but those values are valid LED states too — harmless overlap.
    if (buffer_size == 1 && buffer != NULL) {
        ipc_push_to_core0(IPC_ENCODE(IPC_LED_STATE, buffer[0]));
    }

    return 0;
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void ble_manager_init(void) {
    build_scan_response();   // bake the per-side device name into the scan response
    // Central links (upstream mouse, inter-half) start at 7.5–15 ms / latency 0
    // instead of the BTstack defaults (10–30 ms, latency 4): the mouse relay rate
    // is capped by the slowest hop, and the slack interval starved it.
    gap_set_connection_parameters(0x0060, 0x0030, 6, 12, 0, 400, 0, 0);
    l2cap_init();
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_BONDING);
    att_server_init(profile_data, att_read_cb, att_write_cb);
    // CMK-Split (Phase 18): ATT split lane handler + snapshot resync timer (R3).
    // The ATT handles live in the generated hid_keyboard.h, which only this TU
    // may include (it DEFINES profile_data) — wire them into ble_split here.
    ble_split_init(ATT_CHARACTERISTIC_c3a50002_7a4e_4c1b_9b21_000000000001_01_VALUE_HANDLE,
                   ATT_CHARACTERISTIC_c3a50002_7a4e_4c1b_9b21_000000000001_01_CLIENT_CONFIGURATION_HANDLE,
                   ATT_CHARACTERISTIC_c3a50003_7a4e_4c1b_9b21_000000000001_01_VALUE_HANDLE,
                   ATT_CHARACTERISTIC_c3a50004_7a4e_4c1b_9b21_000000000001_01_VALUE_HANDLE,
                   ATT_CHARACTERISTIC_c3a50005_7a4e_4c1b_9b21_000000000001_01_VALUE_HANDLE,
                   ATT_CHARACTERISTIC_c3a50005_7a4e_4c1b_9b21_000000000001_01_CLIENT_CONFIGURATION_HANDLE,
                   ATT_CHARACTERISTIC_c3a50006_7a4e_4c1b_9b21_000000000001_01_VALUE_HANDLE);
    battery_service_server_init(100);
    device_information_service_server_init();
    hids_device_init_with_storage(0, hid_descriptor, sizeof(hid_descriptor),
                                  5, s_hid_report_storage);
    gatt_client_init();
    ble_central_init();      // hids_client + upstream HID descriptor storage

    s_hci_cb_reg.callback = &hci_event_handler;
    hci_add_event_handler(&s_hci_cb_reg);

    s_sm_cb_reg.callback = &sm_event_handler;
    sm_add_event_handler(&s_sm_cb_reg);

    ble_relay_init();        // hids_device packet handler + HID log timer

    C1_DBG_STATUS("[BLE] Manager ready — GP14=FN | FN+GP12=ADV | FN+GP13=SCAN | GP15=SCAN legacy\n");
}

// ---------------------------------------------------------------------------
// HCI event handler — routes events to the role modules
// ---------------------------------------------------------------------------

static void hci_event_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz) {
    UNUSED(ch); UNUSED(sz);
    if (ptype != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(pkt)) {

    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(pkt) == HCI_STATE_WORKING) {
            // Derive per-slot random static addresses from the chip's BD_ADDR.
            // Deterministic: same addresses across reboots (critical for bonding).
            //   slot 0..2: DS peripheral addresses (vary LSB bits 0-1)
            //   central:   fixed address for upstream role (LSB = 0x03)
            bd_addr_t base;
            const char *base_src = "hci";
            gap_local_bd_addr(base);
            // Defensive: a degraded controller init can return an all-zero
            // BD_ADDR at HCI_STATE_WORKING. With a zero base every derived
            // address collapses to C0:00:00:00:00:0X — the half goes on air
            // with a degenerate SC identity and split pairing fails (0x05).
            // Recover from the cyw43 WiFi MAC, which is read reliably during
            // cyw43_arch_init and equals the BD_ADDR on a healthy unit (so
            // sane halves are unaffected — same addresses, bonds intact).
            // Degenerate detection: a real RM2/Pico MAC never has its 4 middle
            // bytes (the OUI/NIC) all-zero. A degraded read yields base like
            // 00:00:00:00:00:0X → every derived address collapses to
            // C0:00:00:00:00:0X. Catch the whole family, not just all-six-zero
            // (the central LSB is (base[5]&0xFC)|3 = 3 for base[5] in 0..3).
            #define MAC_IS_DEGENERATE(b) (((b)[1] | (b)[2] | (b)[3] | (b)[4]) == 0)
            if (MAC_IS_DEGENERATE(base)) {
                cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA, base);
                base_src = "cyw43-mac";
                if (MAC_IS_DEGENERATE(base)) {
                    // Last resort: cyw43 MAC also degenerate (faulty RM2 module).
                    // Use a deterministic non-zero base, distinct per side
                    // (L != R), so the two halves never collide nor go zero.
                    static const uint8_t k[6] = { 0x02, 0x4B, 0x4D, 0x43, 0x01, 0x00 };
                    memcpy(base, k, 6);
                    base[4] ^= (board_side_get() == CMK_SIDE_RIGHT) ? 0x01 : 0x00;
                    base_src = "fallback-side";
                }
            }
            #undef MAC_IS_DEGENERATE
            for (int i = 0; i < NUM_DS_SLOTS; i++) {
                memcpy(g_ble_slot_addr[i], base, 6);
                g_ble_slot_addr[i][0] |= 0xC0;                           // random static marker (MSB bits 7:6 = 11)
                g_ble_slot_addr[i][5] = (uint8_t)((base[5] & 0xFC) | i); // vary LSB by slot
            }
            // Central (upstream) uses a distinct fixed address
            memcpy(g_ble_central_random_addr, base, 6);
            g_ble_central_random_addr[0] |= 0xC0;
            g_ble_central_random_addr[5] = (uint8_t)((base[5] & 0xFC) | NUM_DS_SLOTS); // LSB = 3
            g_ble_slot_addrs_ready = true;
            // Enable random static address mode for all LE operations
            gap_random_address_set_mode(GAP_RANDOM_ADDRESS_TYPE_STATIC);
            C1_DBG_STATUS("[BLE] HCI ready (base MAC src=%s) — slot 0: %s\n",
                          base_src, bd_addr_to_str(g_ble_slot_addr[0]));
            C1_DBG_STATUS("[BLE] slot 1: %s\n", bd_addr_to_str(g_ble_slot_addr[1]));
            C1_DBG_STATUS("[BLE] slot 2: %s\n", bd_addr_to_str(g_ble_slot_addr[2]));
            ipc_push_to_core0(IPC_ENCODE(IPC_BLE_WORKING, 0));
        }
        break;

    case HCI_EVENT_META_GAP:
        if (hci_event_gap_meta_get_subevent_code(pkt) == GAP_SUBEVENT_LE_CONNECTION_COMPLETE) {
            uint8_t          role   = gap_subevent_le_connection_complete_get_role(pkt);
            hci_con_handle_t handle = gap_subevent_le_connection_complete_get_connection_handle(pkt);
            uint8_t          status = gap_subevent_le_connection_complete_get_status(pkt);

            if (role == HCI_ROLE_SLAVE && status != ERROR_CODE_SUCCESS) {
                ble_periph_on_conn_failed(status);
            } else if (role == HCI_ROLE_SLAVE) {
                ble_periph_on_connected(handle, pkt);
            } else if (status != ERROR_CODE_SUCCESS) {
                ble_central_on_conn_failed(status);
            } else {
                ble_central_on_connected(handle);
            }
        }
        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE: {
        hci_con_handle_t handle = hci_event_disconnection_complete_get_connection_handle(pkt);

        if (handle == g_ble_periph_handle) {
            ble_periph_on_disconnected(handle);
        } else if (handle == g_ble_central_handle) {
            ble_central_on_disconnected(hci_event_disconnection_complete_get_reason(pkt));
        }
        break;
    }

    case GAP_EVENT_ADVERTISING_REPORT:
        ble_central_on_adv_report(pkt);
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// SM handler
// ---------------------------------------------------------------------------

static void sm_event_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz) {
    UNUSED(ch); UNUSED(sz);
    if (ptype != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(pkt)) {
    case SM_EVENT_JUST_WORKS_REQUEST:
        sm_just_works_confirm(sm_event_just_works_request_get_handle(pkt));
        C1_DBG_STATUS("[SM] Just Works confirmed\n");
        break;
    case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
        sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(pkt));
        break;
    case SM_EVENT_PAIRING_COMPLETE: {
        hci_con_handle_t handle = sm_event_pairing_complete_get_handle(pkt);
        uint8_t          status = sm_event_pairing_complete_get_status(pkt);
        if (handle == g_ble_central_handle) {
            // Central role: upstream device paired — start HID discovery
            if (status == ERROR_CODE_SUCCESS) {
                C1_DBG_STATUS("[SM] Central pairing complete — starting HID discovery\n");
                ble_central_start_hid_discovery();
            } else {
                C1_DBG_STATUS("[SM] Central pairing failed (0x%02x) — disconnecting\n", status);
                gap_disconnect(g_ble_central_handle);
            }
        } else if (handle == g_ble_periph_handle) {
            // Peripheral role: downstream (e.g. macOS) paired with us
            if (status == ERROR_CODE_SUCCESS) {
                C1_DBG_STATUS("[SM] Peripheral pairing complete (bonded with downstream)\n");
            } else {
                C1_DBG_STATUS("[SM] Peripheral pairing failed (0x%02x)\n", status);
            }
        }
        break;
    }
    case SM_EVENT_REENCRYPTION_COMPLETE: {
        // Bonded device reconnected — SM re-encrypted with stored keys (no new pairing).
        // Must start HID discovery here, same as SM_EVENT_PAIRING_COMPLETE.
        hci_con_handle_t handle = sm_event_reencryption_complete_get_handle(pkt);
        uint8_t          status = sm_event_reencryption_complete_get_status(pkt);
        // Identity address of the bonded peer — authoritative key to purge a stale bond.
        bd_addr_t id_addr;
        sm_event_reencryption_complete_get_address(pkt, id_addr);
        uint8_t id_addr_type = sm_event_reencryption_complete_get_addr_type(pkt);
        if (handle == g_ble_central_handle) {
            if (status == ERROR_CODE_SUCCESS) {
                C1_DBG_STATUS("[SM] Central re-encryption complete — starting HID discovery\n");
                ble_central_start_hid_discovery();
            } else {
                // Re-encryption with a stale LTK failed (peer was re-paired/reset on its
                // side). Purge the stale bond so the next attempt does a fresh pairing.
                C1_DBG_STATUS("[SM] Central re-encryption failed (0x%02x) — clearing stale bond\n", status);
                gap_delete_bonding((bd_addr_type_t)id_addr_type, id_addr);
                if (status == ERROR_CODE_PIN_OR_KEY_MISSING) {
                    // Link still up — re-pair fresh on the same connection.
                    sm_request_pairing(g_ble_central_handle);
                } else {
                    // Remote already terminated the link (e.g. 0x13) — drop it; the next
                    // reconnect/scan will fresh-pair now that no stored key remains.
                    gap_disconnect(g_ble_central_handle);
                }
            }
        } else if (handle == g_ble_periph_handle) {
            if (status == ERROR_CODE_SUCCESS) {
                C1_DBG_STATUS("[SM] Peripheral re-encryption complete\n");
            } else {
                // Downstream host lost its bond — purge ours so it can re-pair cleanly.
                C1_DBG_STATUS("[SM] Peripheral re-encryption failed (0x%02x) — clearing stale bond\n", status);
                gap_delete_bonding((bd_addr_type_t)id_addr_type, id_addr);
            }
        }
        break;
    }
    default: break;
    }
}

// ---------------------------------------------------------------------------
// IPC command handler — dispatches Core 0 commands to the role modules
// ---------------------------------------------------------------------------

void ble_manager_handle_ipc(uint32_t msg) {
    ipc_msg_type_t type    = IPC_TYPE(msg);
    uint32_t       payload = IPC_PAYLOAD(msg);

    switch (type) {

    case IPC_START_ADV:
        g_ble_periph_reject = false;   // new ADV command clears reject mode
        periph_start_advertising((uint8_t)payload);
        break;

    case IPC_START_SCAN:
        central_start_scanning();
        break;

    case IPC_START_ADV_DIRECTED: {
        g_ble_periph_reject = false;   // new ADV command clears reject mode
        bd_addr_t peer_addr;
        uint8_t   peer_type_raw;
        ipc_get_directed_adv_addr(peer_addr, &peer_type_raw);
        periph_start_advertising_directed((uint8_t)payload, peer_addr, (bd_addr_type_t)peer_type_raw);
        break;
    }

    case IPC_RECONNECT_UPSTREAM:
        central_reconnect();
        break;

    case IPC_SPLIT_TX_EVENT:
        // SLAVE: a raw matrix event from keyscan → queue + notify the master, and
        // keep the snapshot bitmap current (served on a matrix_snapshot READ).
        split_tx_enqueue((uint8_t)payload);
        break;

    case IPC_SET_SPLIT_MODE:
        // Core 0 synced the eeconfig split_mode (boot + CMK_SPLIT_TOGG).
        ble_split_set_mode(payload != 0);
        break;

    case IPC_SEND_RGB_SYNC:
        // MASTER: push the staged RGB effect-state frame to the slave (Phase 24).
        ble_split_send_rgb_sync();
        break;

    case IPC_SEND_RGB_HIT:
        // MASTER: forward a local key hit to the slave for cross-half reactive FX.
        ble_split_send_rgb_hit((uint8_t)payload);
        break;

    case IPC_SEND_KBD_REPORT:
        kbd_drain_ipc_ring();
        break;

    case IPC_SEND_MOUSE_REPORT:
        mouse_slot_take();
        break;

    case IPC_SEND_PB_REPORT:
        pb_slot_take();
        break;

    case IPC_STOP_ADV:
        // Core 0 rapid-disconnect STOP: reject mode (see ble_periph.c).
        ble_periph_stop_and_reject();
        break;

    case IPC_DISCONNECT_PERIPH:
        ble_periph_disconnect_intentional();
        break;

    case IPC_DISCONNECT_CENTRAL:
        // R4: explicit upstream teardown (see ble_central.c).
        ble_central_disconnect();
        break;

    case IPC_BLE_POWER_OFF:
        // Low-power DEEP-sleep prepare: stop the radio (discards all BLE links),
        // then break out of the run loop so core1_entry can power the CYW43 module
        // down. There is NO in-place resume — Core 0 reboots the whole chip on the
        // wake keypress (HCI_POWER_OFF/ON is unreliable on the CYW43 transport;
        // a clean boot reconnects from the bonding store).
        C1_DBG_STATUS("[BLE] deep-sleep power down\n");
        hci_power_control(HCI_POWER_OFF);
        btstack_run_loop_trigger_exit();
        break;

    case IPC_SET_BATTERY_LEVEL: {
        uint8_t soc = (uint8_t)payload;
        if (soc > 100) soc = 100;
        battery_service_server_set_battery_value(soc);
        break;
    }

    case IPC_DELETE_BONDING: {
        // Core 0 queued one or more identity addresses (US/DS slot cleared, or
        // factory reset). Drain them all and purge each SM bond so the next
        // connection does a fresh pairing instead of re-encryption with a stale LTK.
        bd_addr_t addr;
        uint8_t   addr_type;
        while (ipc_get_delete_bonding_addr(addr, &addr_type)) {
            gap_delete_bonding((bd_addr_type_t)addr_type, addr);
            C1_DBG_STATUS("[SM] Deleted stale bond for %s\n", bd_addr_to_str(addr));
        }
        break;
    }

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// State accessors
// ---------------------------------------------------------------------------

periph_state_t  ble_manager_periph_state(void)  { return g_ble_periph_state;  }
central_state_t ble_manager_central_state(void) { return g_ble_central_state; }
