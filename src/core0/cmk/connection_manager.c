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
// connection_manager.c — Core 0 FreeRTOS task: BLE/USB connection state management
//
// Loop ogni CM_LOOP_MS:
//   1. Drena IPC da Core 1 → aggiorna state machine
//   2. Controlla azioni automatiche (polling USB)
//
// Azioni combo (chiamate da cmk_keycodes.c via CMK_TD0-2 / CMK_TU):
//   connection_manager_ds_adv_directed(slot)  — Td{x} short
//   connection_manager_ds_adv_general(slot)   — Td{x} long
//   connection_manager_us_scan_general()       — Tu long
//

#include "connection_manager.h"
#include "bonding_store.h"
#include "eeconfig.h"      // eeconfig_read_split_mode() — split gate sync to Core 1
#include "led_indicators.h"
#include "output_select.h"
#include "action_util.h"   // action_util_set_usb_upstream_kbd() — USB upstream merge
#include "power_manager.h" // pm_get_battery_status() — BLE Battery Service feed
#include "tlv/btstack_tlv_eeprom.h"
#include "ipc/ipc_core.h"
#include "ble/ble_manager.h"   // periph_state_t / central_state_t — R3 reconciliation
#include "diag.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#ifdef USB_ENABLE
#include "usb/usb_hid.h"
#endif
#ifdef RGB_MATRIX_ENABLE   // AFTER cmk_config.h — the flag lives there
#include "rgb_matrix.h"    // rgb_matrix_persist_poll() — R7: EEPROM off the keyscan tick
#endif

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Costanti
// ---------------------------------------------------------------------------

#define CM_TASK_STACK      2048   // needs headroom for eeprom_write (txbuf[258] + I2C SDK + printf)
#define CM_TASK_PRIORITY   2
#define CM_LOOP_MS         10

// ---------------------------------------------------------------------------
// Stato interno
// ---------------------------------------------------------------------------

static volatile cm_upstream_state_t s_upstream  = CM_UPSTREAM_IDLE;
static volatile cm_ds_ble_state_t   s_ds_ble    = CM_DS_BLE_IDLE;
static volatile cm_ds_usb_state_t   s_ds_usb    = CM_DS_USB_IDLE;

// Split runtime role (Fase 18) — set from Core 1 IPC role events; read by keyscan.
static volatile cm_split_role_t     s_split_role  = CM_SPLIT_STANDALONE;
static volatile uint8_t             s_remote_side = 0;   // peer half board side (MASTER)

// Quale slot DS è in attesa di connessione (-1 = nessuno).
// volatile: scritto dal keyscan task (API pubbliche), letto dal CM task.
static volatile int8_t s_pending_ds_slot = -1;

// Deferred IPC actions — ALL ipc_push_to_core1() calls go through here.
// Set by IPC_BLE_WORKING boot handler AND public API (keyscan task).
// Executed by cm_task on the NEXT loop iteration (10 ms after being set).
//
// Why defer even the boot handler?
//   ipc_push_to_core1() calls __sev() which can wake Core 1 from __wfe before
//   the IPC timer fires.  If Core 1 immediately prints inside central_reconnect()
//   or periph_start_advertising_directed() while Core 0 is still printing EEPROM
//   debug lines in the same IPC_BLE_WORKING handler, both cores contend for the
//   shared UART mutex.  If Core 1 then crashes inside BTstack while holding the
//   mutex, Core 0 blocks on __wfe forever.
//   Deferring all pushes to the next CM iteration ensures Core 0 finishes ALL
//   its handler prints before Core 1 receives any command.
// R3: a deferred flag is cleared ONLY when its IPC push actually succeeded (the
// HW FIFO is 8-deep and Core 1 drains 1 msg/5 ms — a full FIFO used to swallow
// the command while the flag was already cleared and the state optimistically
// advanced → permanent ghost ADVERTISING/CONNECTING). On a failed push the flag
// stays set and the block retries on the next 10 ms iteration; the EEPROM side
// effects are guarded by has_*() so a retry never repeats them.
static volatile bool    s_deferred_reconnect_us  = false;  // send IPC_RECONNECT_UPSTREAM (boot)
static volatile bool    s_deferred_adv_general   = false;  // clear DS slot + send IPC_START_ADV
static volatile bool    s_deferred_scan_general  = false;  // clear US addr + send IPC_START_SCAN
static volatile bool    s_deferred_adv_directed  = false;  // send IPC_START_ADV_DIRECTED
static volatile bool    s_deferred_disconnect_ds = false;  // disconnect current DS before ADV switch
static volatile bool    s_deferred_clear_all     = false;  // factory reset: clear all bonds + SM keys
static bool             s_delbond_notify_pending = false;  // IPC_DELETE_BONDING notify to retry

// Low-power DEEP-sleep prepare (set by connection_manager_request_radio from the
// keyscan/low_power context; pushed here). One-way: there is no power-on — Core 0
// reboots the chip on the wake keypress, so reconnect runs via the boot path.
static volatile bool    s_deferred_radio_off     = false;  // send IPC_BLE_POWER_OFF

// DS-priority sequencing — CYW43 cannot do Advertising + Initiating simultaneously.
// When DS ADV is requested, US reconnect is paused (Core 1 cancels gap_connect).
// When DS finishes (connected or idle), US auto-restarts if this flag is set.
static volatile bool    s_us_reconnect_pending   = false;

// Rapid-disconnect detection — after btstack_tlv_none reboot, the peer connects
// then disconnects immediately (encryption failure).  Without a limit, the auto-
// reconnect creates an infinite loop that also blocks upstream permanently.
#define DS_RAPID_DISCONNECT_THRESHOLD_MS   3000   // connection shorter than this = "rapid"
#define DS_RAPID_DISCONNECT_MAX            3      // stop after this many rapid disconnects
static uint8_t    s_ds_rapid_disconnect_count = 0;
static TickType_t s_ds_last_connect_tick      = 0;

// R4: upstream mirror of the DS rapid-disconnect guard — a pairing that fails
// systematically must not keep the radio in Initiating forever.
#define US_RAPID_DISCONNECT_THRESHOLD_MS   5000
#define US_RAPID_DISCONNECT_MAX            3
static uint8_t    s_us_rapid_disconnect_count = 0;
static TickType_t s_us_last_connect_tick      = 0;

// R4: "clear US bond + rescan" two-step — the central teardown was requested
// (IPC_DISCONNECT_CENTRAL); start the general scan on IPC_CENTRAL_DISCONNECTED.
static bool s_scan_after_us_disconnect = false;

// Distinguish user-initiated slot switch from auto-reconnect.
// When true, the pending s_deferred_adv_directed is a slot-switch and auto-reconnect
// must NOT overwrite it (different slot, different address).
static volatile bool s_deferred_is_slot_switch = false;

// True while DS is in general ADV mode (user-initiated new pairing).
static bool s_ds_general_adv_active = false;

// Set when CM rejects a peer during general ADV (already bonded on another slot).
// IPC_DISCONNECT_PERIPH sets s_periph_disconnect_intentional on Core 1, which
// suppresses Core 1's auto-restart.  So CM must restart ADV explicitly on the
// subsequent IPC_PERIPH_DISCONNECTED.
static volatile bool s_ds_reject_pending = false;

// ---------------------------------------------------------------------------
// USB output — stubs (futuro TinyUSB)
// ---------------------------------------------------------------------------

#ifdef USB_ENABLE
#include "usb/usb_hid.h"
#ifdef OS_DETECTION_ENABLE
#include "usb/os_detection.h"
#endif
#endif

static void usb_output_init(void) {
    // USB init is handled by usb_hid_task_create() in main.c
}

static bool usb_output_is_connected(void) {
#ifdef USB_ENABLE
    return usb_hid_is_active();
#else
    return false;
#endif
}

// ---------------------------------------------------------------------------
// Phase 24 — cross-half RGB sync helpers (engage/disengage on role transitions)
// ---------------------------------------------------------------------------

#if defined(RGB_MATRIX_ENABLE)
// engaged → apply the unified-X offset (continuous gradient across the split).
// restore_own → the half was a SLAVE rendering the master's state; reload its own
// persisted RGB config so it stops following once the split is gone.
static void cm_rgb_split_engage(bool engaged, bool restore_own) {
    rgb_matrix_set_split_engaged(engaged);
    if (restore_own) rgb_matrix_load_eeconfig();
}
#else
static inline void cm_rgb_split_engage(bool engaged, bool restore_own) {
    (void)engaged; (void)restore_own;
}
#endif

// ---------------------------------------------------------------------------
// IPC handler — messaggi da Core 1
// ---------------------------------------------------------------------------

static void cm_drain_upstream_hid(void);   // R3 — defined after the handler

static void cm_handle_ipc(uint32_t msg) {
    ipc_msg_type_t type    = IPC_TYPE(msg);
    uint32_t       payload = IPC_PAYLOAD(msg);

    switch (type) {

    case IPC_BLE_READY:
        DBG_STATUS("[CM] BLE stack %s\n", payload == 0 ? "inizializzato" : "ERRORE");
        break;

    // Radio pronta — boot behavior
    // Priority: DS first (ADV is time-limited), then US reconnect.
    // CYW43 cannot do Advertising + Initiating simultaneously, so when both
    // addresses are saved, DS ADV goes first and US waits until DS finishes.
    case IPC_BLE_WORKING: {
        DBG_STATUS("[CM] BLE working — boot\n");
#ifdef USB_ENABLE
        // BLE radio is up — safe to start USB now (tusb_init must not race the
        // Core 1 CYW43/BTstack bringup, which hangs it during BT FW download).
        usb_hid_start();
#endif
        bool has_us = bonding_store_has_us();
        uint8_t slot = bonding_store_get_active_slot();
        bool has_ds = bonding_store_has_ds(slot);

        // Upstream: prepare reconnect addr
        if (has_us) {
            uint8_t addr[6];
            uint8_t addr_type;
            bonding_store_get_us(addr, &addr_type);
            ipc_set_reconnect_addr(addr, addr_type);   // spinlock write — safe here
            if (has_ds) {
                // Both present: DS has priority — US waits
                s_us_reconnect_pending = true;
                DBG_STATUS("[CM] Boot: US bonding → reconnect dopo DS (priorità DS)\n");
            } else {
                // Only US: reconnect immediately
                s_upstream = CM_UPSTREAM_CONNECTING;
                s_deferred_reconnect_us = true;
                DBG_STATUS("[CM] Boot: US bonding → reconnect differito\n");
            }
        } else {
            DBG_STATUS("[CM] Boot: nessun US bonding → idle\n");
        }

        // Downstream: directed ADV if addr saved
        if (has_ds) {
            uint8_t addr[6];
            uint8_t addr_type;
            bonding_store_get_ds(slot, addr, &addr_type);
            ipc_set_directed_adv_addr(addr, addr_type);  // spinlock write — safe here
            s_pending_ds_slot        = (int8_t)slot;
            s_deferred_adv_directed  = true;
            s_deferred_is_slot_switch = false;  // boot reconnect, not user switch
            DBG_STATUS("[CM] Boot: DS slot %u → directed ADV differito\n", slot);
        } else {
            DBG_STATUS("[CM] Boot: nessun DS bonding → idle\n");
        }
        break;
    }

    // --- Downstream BLE (Peripheral) ---

    case IPC_PERIPH_ADVERTISING:
        s_ds_ble = CM_DS_BLE_ADVERTISING;
        s_ds_general_adv_active = true;
        DBG_STATUS("[CM] Downstream BLE: general advertising\n");
        break;

    case IPC_PERIPH_CONNECTED: {
        s_ds_ble = CM_DS_BLE_CONNECTED;
        // Don't clear s_ds_general_adv_active here — wait for IPC_PERIPH_CONNECTED_ADDR
        // to confirm the peer is accepted.  If rejected (duplicate from another slot),
        // the flag must stay true so IPC_PERIPH_DISCONNECTED restarts general ADV.
        s_ds_last_connect_tick = xTaskGetTickCount();
        // R4: Core 1 now reports WHICH slot connected (payload bits 16..23). If a
        // stale ADV timeout cleared our pending slot just before the connection
        // landed, restore it so IPC_PERIPH_CONNECTED_ADDR still saves the peer
        // (before: peer accepted + paired but never persisted → wrong LED slot,
        // bonding_store/TLV divergence, directed-ADV to the wrong identity).
        uint8_t conn_slot = (uint8_t)((payload >> 16) & 0xFF);
        if (s_pending_ds_slot < 0 && conn_slot < BONDING_DS_SLOTS) {
            s_pending_ds_slot = (int8_t)conn_slot;
            DBG_STATUS("[CM] DS slot pendente ripristinato dal payload: %u\n", conn_slot);
        }
        DBG_STATUS("[CM] Downstream BLE: connesso (slot=%u handle=0x%04lx)\n",
                   conn_slot, payload & 0xFFFF);
        break;
    }

    case IPC_PERIPH_CONNECTED_ADDR: {
        // Downstream peer addr — save to the pending DS slot.
        uint8_t addr[6];
        uint8_t addr_type;
        ipc_get_periph_addr(addr, &addr_type);
        if (s_pending_ds_slot >= 0) {
            uint8_t target_slot = (uint8_t)s_pending_ds_slot;

            // During general ADV (new pairing): reject if this peer is already
            // bonded on a DIFFERENT slot.  With per-slot random addresses the
            // peer won't auto-reconnect after disconnect (it only knows the
            // old slot's address), so this fires at most once.
            if (s_ds_general_adv_active) {
                for (uint8_t s = 0; s < BONDING_DS_SLOTS; s++) {
                    if (s == target_slot) continue;
                    if (!bonding_store_has_ds(s)) continue;
                    uint8_t saved[6]; uint8_t saved_type;
                    bonding_store_get_ds(s, saved, &saved_type);
                    if (memcmp(addr, saved, 6) == 0) {
                        DBG_STATUS("[CM] DS addr %02x:%02x:..:%02x già su slot %u — rifiuto (slot %u vuole nuovo)\n",
                                   addr[5], addr[4], addr[0], s, target_slot);
                        s_ds_reject_pending = true;
                        ipc_push_to_core1(IPC_ENCODE(IPC_DISCONNECT_PERIPH, 0));
                        goto done_periph_addr;
                    }
                }
            }

            // Accepted — save
            s_ds_general_adv_active = false;
            s_ds_reject_pending = false;
            bonding_store_save_ds(target_slot, addr, addr_type);
            bonding_store_set_active_slot(target_slot);
            DBG_STATUS("[CM] DS addr salvato slot %u (%02x:%02x:%02x:%02x:%02x:%02x)\n",
                       target_slot, addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
            s_pending_ds_slot = -1;
        } else {
            DBG_STATUS("[CM] DS addr ricevuto ma nessuno slot pendente — ignorato\n");
        }
        done_periph_addr:
        break;
    }

    case IPC_PERIPH_DISCONNECTED: {
        s_ds_ble = CM_DS_BLE_IDLE;
        DBG_STATUS("[CM] Downstream BLE: disconnesso\n");

        // Rejected peer during general ADV — Core 1 won't auto-restart because
        // s_periph_disconnect_intentional is set.  CM restarts ADV explicitly.
        if (s_ds_reject_pending) {
            s_ds_reject_pending = false;
            DBG_STATUS("[CM] DS reject completato — riavvio general ADV slot %d\n", s_pending_ds_slot);
            ipc_push_to_core1(IPC_ENCODE(IPC_START_ADV, (uint32_t)s_pending_ds_slot));
            s_ds_ble = CM_DS_BLE_ADVERTISING;
            break;
        }

        // During general ADV (user-initiated new pairing), Core 1 auto-restarts
        // the ADV on disconnect (non-intentional, e.g. encryption failure).
        // Don't count rapid disconnects and don't auto-reconnect.
        if (s_ds_general_adv_active) {
            DBG_STATUS("[CM] DS disconnect durante general ADV — Core 1 riavvia ADV\n");
            break;
        }

        // --- Rapid-disconnect detection ---
        // If connection lasted less than threshold, it's likely an encryption
        // failure (btstack_tlv_none after reboot).  After MAX rapid disconnects,
        // stop auto-reconnecting to break the infinite loop.
        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed_ms = (now - s_ds_last_connect_tick); // 1 tick = 1 ms (1000 Hz)
        if (s_ds_last_connect_tick != 0 && elapsed_ms < DS_RAPID_DISCONNECT_THRESHOLD_MS) {
            s_ds_rapid_disconnect_count++;
        } else {
            s_ds_rapid_disconnect_count = 0;
        }

        // Skip auto-reconnect if a user-initiated slot-switch is already pending
        // (s_deferred_adv_directed + s_deferred_is_slot_switch).
        // Also skip if general ADV is pending.
        if (s_deferred_adv_general ||
            (s_deferred_adv_directed && s_deferred_is_slot_switch)) {
            DBG_STATUS("[CM] DS disconnect: slot-switch pendente, skip auto-reconnect\n");
            break;
        }

        // Check rapid-disconnect limit
        if (s_ds_rapid_disconnect_count >= DS_RAPID_DISCONNECT_MAX) {
            DBG_STATUS("[CM] DS auto-reconnect: %u disconnect rapidi, STOP (rifare pairing)\n",
                       s_ds_rapid_disconnect_count);
            s_deferred_adv_directed = false;
            // Tell Core 1 to enter reject mode: stop ADV and refuse any
            // incoming connections from the peer (Mac keeps retrying on its own).
            ipc_push_to_core1(IPC_ENCODE(IPC_STOP_ADV, 0));
            // Let US reconnect proceed since DS is giving up
            break;
        }

        // Auto-reconnect via directed ADV to active slot
        uint8_t slot = bonding_store_get_active_slot();
        if (bonding_store_has_ds(slot)) {
            uint8_t addr[6];
            uint8_t addr_type;
            bonding_store_get_ds(slot, addr, &addr_type);
            ipc_set_directed_adv_addr(addr, addr_type);
            s_pending_ds_slot        = (int8_t)slot;  // needed for IPC payload (slot number)
            s_deferred_adv_directed  = true;
            s_deferred_is_slot_switch = false;
            DBG_STATUS("[CM] DS auto-reconnect: directed ADV slot %u\n", slot);
        }
        break;
    }

    // --- Upstream (Central) ---

    case IPC_CENTRAL_SCANNING:
        s_upstream = CM_UPSTREAM_SCANNING;
        DBG_STATUS("[CM] Upstream: scanning\n");
        break;

    case IPC_CENTRAL_FOUND:
        s_upstream = CM_UPSTREAM_CONNECTING;
        DBG_STATUS("[CM] Upstream: device trovato / reconnect in corso\n");
        break;

    case IPC_CENTRAL_CONNECTED:
        s_upstream = CM_UPSTREAM_CONNECTED;
        s_us_last_connect_tick = xTaskGetTickCount();   // R4: rapid-disconnect tracking
        DBG_STATUS("[CM] Upstream: connesso (cid=0x%04lx)\n", payload);
        break;

    case IPC_SAVE_UPSTREAM_ADDR: {
        uint8_t addr[6];
        uint8_t addr_type;
        ipc_get_upstream_addr(addr, &addr_type);
        bonding_store_save_us(addr, addr_type);
        DBG_STATUS("[CM] US addr salvato (%02x:%02x:%02x:%02x:%02x:%02x type=%u)\n",
                   addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], addr_type);
        break;
    }

    case IPC_CENTRAL_DISCONNECTED: {
        s_upstream = CM_UPSTREAM_IDLE;
        DBG_STATUS("[CM] Upstream: disconnesso → IDLE\n");

        // R4: "clear US + rescan" teardown completed — start the general scan now.
        // The bond was already purged, so the auto-reconnect below cannot fire.
        if (s_scan_after_us_disconnect) {
            s_scan_after_us_disconnect = false;
            s_deferred_scan_general = true;
            DBG_STATUS("[CM] US teardown per rescan completato → scan generale\n");
            break;
        }

        // R4: rapid-disconnect backoff (mirror of the DS one). A systematically
        // failing pairing/re-encryption loops connect→fail→disconnect forever,
        // keeping the radio in Initiating and starving every ADV request.
        TickType_t now_t = xTaskGetTickCount();
        if (s_us_last_connect_tick != 0 &&
            (now_t - s_us_last_connect_tick) < pdMS_TO_TICKS(US_RAPID_DISCONNECT_THRESHOLD_MS)) {
            s_us_rapid_disconnect_count++;
        } else {
            s_us_rapid_disconnect_count = 0;
        }
        if (s_us_rapid_disconnect_count >= US_RAPID_DISCONNECT_MAX) {
            DBG_STATUS("[CM] US auto-reconnect: %u disconnect rapidi, STOP (rifare pairing)\n",
                       s_us_rapid_disconnect_count);
            break;   // user action (Tu) resets the counter and restarts
        }

        // Auto-reconnect if bonding exists (same as boot behavior)
        if (bonding_store_has_us()) {
            uint8_t addr[6];
            uint8_t addr_type;
            bonding_store_get_us(addr, &addr_type);
            ipc_set_reconnect_addr(addr, addr_type);
            // Defer to avoid sending IPC inside IPC handler
            s_deferred_reconnect_us = true;
            s_upstream = CM_UPSTREAM_CONNECTING;
            DBG_STATUS("[CM] US bonding presente → reconnect differito\n");
        }
        break;
    }

    case IPC_CMD_REJECTED: {
        // R4: Core 1 refused a command (state mismatch). Roll back the optimistic
        // state instead of waiting forever for a completion that won't come.
        uint8_t cmd = (uint8_t)payload;
        DBG_STATUS("[CM] Core 1 ha rifiutato cmd 0x%02x — rollback stato\n", cmd);
        switch (cmd) {
        case IPC_START_ADV:
        case IPC_START_ADV_DIRECTED:
            if (s_ds_ble == CM_DS_BLE_ADVERTISING || s_ds_ble == CM_DS_BLE_ADV_DIRECTED) {
                s_ds_ble = CM_DS_BLE_IDLE;
            }
            break;
        case IPC_START_SCAN:
            if (s_upstream == CM_UPSTREAM_SCANNING) s_upstream = CM_UPSTREAM_IDLE;
            break;
        case IPC_RECONNECT_UPSTREAM:
            if (s_upstream == CM_UPSTREAM_CONNECTING) {
                s_upstream = CM_UPSTREAM_IDLE;
                s_us_reconnect_pending = true;   // retry when Core 1 frees up
            }
            break;
        default:
            break;
        }
        break;
    }

    case IPC_CENTRAL_CONNECT_TIMEOUT:
        s_upstream = CM_UPSTREAM_IDLE;
        DBG_STATUS("[CM] Upstream: connect timeout → IDLE\n");
        break;

    // --- Split runtime role (Fase 18), decided by BLE on Core 1 ---
    case IPC_SPLIT_ROLE_MASTER:
        s_split_role  = CM_SPLIT_MASTER;
        s_remote_side = (uint8_t)payload;   // peer (slave) board side
        cm_rgb_split_engage(true, false);   // Phase 24: unified-X offset on
        DBG_STATUS("[CM] Split: MASTER (peer side=%u)\n", (unsigned)payload);
        break;

    case IPC_SPLIT_ROLE_SLAVE:
        s_split_role = CM_SPLIT_SLAVE;
        cm_rgb_split_engage(true, false);   // Phase 24: become RGB follower of the master
        DBG_STATUS("[CM] Split: SLAVE\n");
        break;

    case IPC_SPLIT_ROLE_STANDALONE:
        // Leaving the split: a SLAVE was rendering the master's state → restore its
        // own persisted RGB config (cm_rgb_split_engage reloads it before s_split_role flips).
        cm_rgb_split_engage(false, s_split_role == CM_SPLIT_SLAVE);
        s_split_role = CM_SPLIT_STANDALONE;
        DBG_STATUS("[CM] Split: STANDALONE\n");
        break;

    case IPC_SPLIT_ROLE_MISMATCH:
        // Strict matching: the sibling is a CMK half but its split_mode disagrees
        // with ours. The halves don't cooperate (Core 1 drops its HID reports);
        // this half keeps working standalone towards the PC.
        cm_rgb_split_engage(false, s_split_role == CM_SPLIT_SLAVE);
        s_split_role  = CM_SPLIT_MISMATCH;
        s_remote_side = (uint8_t)payload;
        DBG_STATUS("[CM] Split: MISMATCH (sibling side=%u, split_mode discordante)\n",
                   (unsigned)payload);
        break;

    case IPC_SPLIT_RX_EVENT:
        // MASTER: a raw matrix event from the slave → hand to the keyscan task,
        // which injects it into the unified action pipeline (single owner).
        ipc_split_push_remote_event((uint8_t)payload);
        break;

    case IPC_RGB_SYNC_RX: {
        // SLAVE: the master pushed an RGB frame (Phase 24). STATE → apply as a
        // follower (live config, not persisted). HIT → hand to the keyscan task,
        // which places the reactive hit (record_hit_xy). Ignore unless SLAVE.
#if defined(RGB_MATRIX_ENABLE)
        uint8_t frame[CMK_RGB_SYNC_FRAME_MAX]; uint8_t flen = 0;
        if (ipc_get_rgb_sync_rx(frame, &flen) && flen >= 1 &&
            s_split_role == CM_SPLIT_SLAVE) {
            if (frame[0] == CMK_RGB_SYNC_TAG_STATE) {
                rgb_matrix_apply_sync_state(frame, flen);
            } else if (frame[0] == CMK_RGB_SYNC_TAG_HIT && flen >= CMK_RGB_SYNC_HIT_LEN) {
                ipc_rgb_hit_rx_push(CMK_RGB_HIT(frame[1], frame[2], frame[3]));
            }
        }
#endif
        break;
    }

    case IPC_SPLIT_RX_SNAPSHOT:
        // MASTER: slave's full 24-bit pressed bitmap (connect/resync) → keyscan
        // diffs it against the tracked remote state.
        ipc_split_set_remote_snapshot(payload);
        break;

    case IPC_HID_REPORT:
        // R3: the payload slot index is ignored — the ring is drained in order
        // by cm_drain_upstream_hid(), which the task loop also calls on every
        // iteration. A lost notify is now just latency, never a lost report.
        cm_drain_upstream_hid();
        break;

    case IPC_PERIPH_ADV_TIMEOUT:
        // General ADV o directed ADV timeout → IDLE
        s_ds_ble = CM_DS_BLE_IDLE;
        s_pending_ds_slot = -1;
        s_ds_general_adv_active = false;
        // ADV timeout is NOT a rapid disconnect — reset counter
        s_ds_rapid_disconnect_count = 0;
        DBG_STATUS("[CM] Downstream BLE: ADV timeout → IDLE\n");
        break;

    case IPC_CENTRAL_SCAN_TIMEOUT:
        s_upstream = CM_UPSTREAM_IDLE;
        DBG_STATUS("[CM] Upstream: scan timeout → IDLE\n");
        break;

    case IPC_LED_STATE:
        led_indicators_set_state((uint8_t)IPC_PAYLOAD(msg));
        break;

    default:
        DBG_STATUS("[CM] IPC sconosciuto: 0x%08lx\n", msg);
        break;
    }
}

// ---------------------------------------------------------------------------
// Upstream HID drain (R3) — pop-based, notify-independent
// ---------------------------------------------------------------------------

// Drain every pending upstream HID report from the IPC ring (Phase 14 ext:
// forwarded by Core 1 while output == USB). buf[0] = Report ID, buf[1..] = data.
//   Keyboard: hand the upstream contribution to action_util (merged with local).
//   Mouse:    relative deltas — forward straight to USB.
static void cm_drain_upstream_hid(void) {
    uint8_t buf[IPC_HID_SLOT_SIZE];
    uint8_t len;
    while (ipc_pop_hid_report(buf, &len)) {
#ifdef USB_ENABLE
        if (output_usb_enabled() && len >= 2) {
            uint8_t report_id = buf[0];
            if (report_id == HID_REPORT_ID_KEYBOARD && len >= 1 + 8) {
                action_util_set_usb_upstream_kbd(buf + 1);
            } else if (report_id == HID_REPORT_ID_MOUSE && len >= 1 + 7) {
                usb_hid_send_mouse_report(buf + 1);
            }
        }
#else
        (void)buf; (void)len;
#endif
    }
}

// ---------------------------------------------------------------------------
// Riconciliazione level-based con Core 1 (R3)
// ---------------------------------------------------------------------------

// Core 1 publishes its REAL state every 5 ms (ipc_publish_ble_status). The FIFO
// events stay the low-latency edge triggers, but if one is lost (full FIFO) the
// CM no longer diverges forever: a mismatch that persists past the threshold is
// healed here. Missed DISCONNECTs are healed by synthesizing the event through
// cm_handle_ipc (so the normal auto-reconnect/ADV logic runs); missed CONNECTs
// only adjust the state (the connect side-channel data is gone anyway).
static void cm_reconcile_with_core1(void) {
    // Thresholds in CM iterations (10 ms each): transitions in flight must not
    // trigger a "fix" — only a mismatch that persists is a lost event.
    enum { RECON_CONNECTED_TH = 50, RECON_INTERMEDIATE_TH = 100 };
    static uint16_t s_mm_ds_conn, s_mm_ds_ghost, s_mm_us_conn, s_mm_us_ghost;

    ipc_ble_status_t st;
    ipc_get_ble_status(&st);
    if (!st.valid) return;

    bool c1_ds_conn = (st.periph == (uint8_t)PERIPH_CONNECTED);
    bool c1_us_conn = (st.central == (uint8_t)CENTRAL_CONNECTED ||
                       st.central == (uint8_t)CENTRAL_DISCOVERING);

    // DS: CM says CONNECTED, Core 1 says not → missed IPC_PERIPH_DISCONNECTED.
    if (s_ds_ble == CM_DS_BLE_CONNECTED && !c1_ds_conn) {
        if (++s_mm_ds_conn >= RECON_CONNECTED_TH) {
            s_mm_ds_conn = 0;
            DBG_STATUS("[CM] RECON: DS CONNECTED fantasma → sintetizzo DISCONNECTED\n");
            cm_handle_ipc(IPC_ENCODE(IPC_PERIPH_DISCONNECTED, 0));
        }
    } else s_mm_ds_conn = 0;

    // DS: CM in ADV/IDLE, Core 1 says CONNECTED → missed IPC_PERIPH_CONNECTED.
    if (s_ds_ble != CM_DS_BLE_CONNECTED && c1_ds_conn) {
        if (++s_mm_ds_ghost >= RECON_CONNECTED_TH) {
            s_mm_ds_ghost = 0;
            DBG_STATUS("[CM] RECON: DS connesso su Core 1 ma non qui → CONNECTED\n");
            s_ds_ble = CM_DS_BLE_CONNECTED;
        }
    } else s_mm_ds_ghost = 0;

    // US: CM says CONNECTED, Core 1 says not → missed IPC_CENTRAL_DISCONNECTED.
    if (s_upstream == CM_UPSTREAM_CONNECTED && !c1_us_conn) {
        if (++s_mm_us_conn >= RECON_CONNECTED_TH) {
            s_mm_us_conn = 0;
            DBG_STATUS("[CM] RECON: US CONNECTED fantasma → sintetizzo DISCONNECTED\n");
            cm_handle_ipc(IPC_ENCODE(IPC_CENTRAL_DISCONNECTED, 0));
        }
    } else s_mm_us_conn = 0;

    // US: CM in SCANNING/CONNECTING/IDLE, Core 1 says CONNECTED → missed
    // IPC_CENTRAL_CONNECTED. Mirror of the DS branch above — it was missing,
    // so after a split toggle the CM could blink "scanning" forever against a
    // live link (or show OFF). Gate on CENTRAL_CONNECTED only (DISCOVERING is
    // a legitimate long-lived CONNECTING phase) and use the slower threshold.
    static uint16_t s_mm_us_missed_conn;
    if (s_upstream != CM_UPSTREAM_CONNECTED &&
        st.central == (uint8_t)CENTRAL_CONNECTED) {
        if (++s_mm_us_missed_conn >= RECON_INTERMEDIATE_TH) {
            s_mm_us_missed_conn = 0;
            DBG_STATUS("[CM] RECON: US connesso su Core 1 ma non qui → sintetizzo CONNECTED\n");
            cm_handle_ipc(IPC_ENCODE(IPC_CENTRAL_CONNECTED, 0));
        }
    } else s_mm_us_missed_conn = 0;

    // Intermediate ghosts (ADVERTISING/SCANNING/CONNECTING with Core 1 idle):
    // a lost command or a swallowed completion. Reset to IDLE so the deferred /
    // auto-restart logic can take over again (R4 adds the retry policy).
    bool us_ghost = ((s_upstream == CM_UPSTREAM_SCANNING   && st.central == (uint8_t)CENTRAL_IDLE) ||
                     (s_upstream == CM_UPSTREAM_CONNECTING && st.central == (uint8_t)CENTRAL_IDLE));
    bool ds_ghost = ((s_ds_ble == CM_DS_BLE_ADVERTISING  || s_ds_ble == CM_DS_BLE_ADV_DIRECTED) &&
                     st.periph == (uint8_t)PERIPH_IDLE);
    if (us_ghost || ds_ghost) {
        if (++s_mm_us_ghost >= RECON_INTERMEDIATE_TH) {
            s_mm_us_ghost = 0;
            if (us_ghost) {
                DBG_STATUS("[CM] RECON: US %d fantasma (Core1 IDLE) → IDLE\n", (int)s_upstream);
                s_upstream = CM_UPSTREAM_IDLE;
            }
            if (ds_ghost) {
                DBG_STATUS("[CM] RECON: DS ADV fantasma (Core1 IDLE) → IDLE\n");
                s_ds_ble = CM_DS_BLE_IDLE;
            }
        }
    } else s_mm_us_ghost = 0;
}

// ---------------------------------------------------------------------------
// Azioni automatiche (polling USB)
// ---------------------------------------------------------------------------

static void cm_check_auto(void) {
    cm_ds_usb_state_t new_usb = usb_output_is_connected()
                                ? CM_DS_USB_CONNECTED
                                : CM_DS_USB_IDLE;
    if (new_usb != s_ds_usb) {
        s_ds_usb = new_usb;
        DBG_STATUS("[CM] Downstream USB: %s\n",
               s_ds_usb == CM_DS_USB_CONNECTED ? "connesso" : "disconnesso");
    }

#if defined(USB_ENABLE) && defined(OS_DETECTION_ENABLE)
    os_detection_tick(xTaskGetTickCount());
#endif
}

// ---------------------------------------------------------------------------
// Task principale
// ---------------------------------------------------------------------------

static void cm_task(void *arg) {
    (void)arg;
    DBG_STATUS("[CM] Connection Manager avviato\n");

    for (;;) {
        static bool s_hwm_logged = false;
        if (!s_hwm_logged) {
            s_hwm_logged = true;
            UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
            DBG_STATUS("[CM] Stack high-watermark: %lu words free (stack=%u words)\n",
                       (unsigned long)hwm, CM_TASK_STACK);
        }

        // Drain Core 1 log ring buffer → UART (only Core 0 touches UART)
        ipc_log_drain();

        // Flush one dirty TLV page to EEPROM (max ~10ms per page)
        btstack_tlv_eeprom_flush_if_dirty();

        uint32_t msg;
        while (ipc_pop_from_core1(&msg)) {
            cm_handle_ipc(msg);
        }

        // R3: notify-independent drains + level-based reconciliation.
        cm_drain_upstream_hid();
        cm_reconcile_with_core1();

#if defined(RGB_MATRIX_ENABLE)
        // R7: flush a pending RGB persist here (EEPROM write off the keyscan tick).
        rgb_matrix_persist_poll();

#if CMK_SPLIT_RGB_SYNC
        // Phase 24: while MASTER, push the RGB effect state (+ animation phase) to
        // the slave at a slow cadence. Core 1 writes it without response, yielding
        // ACL buffers to HID reports, so this never disturbs the keyboard/mouse path.
        if (s_split_role == CM_SPLIT_MASTER) {
            static TickType_t s_rgb_sync_last = 0;
            TickType_t now = xTaskGetTickCount();
            if ((now - s_rgb_sync_last) >= pdMS_TO_TICKS(CMK_SPLIT_RGB_SYNC_MS)) {
                s_rgb_sync_last = now;
                uint8_t frame[CMK_RGB_SYNC_STATE_LEN];
                rgb_matrix_build_sync_state(frame);
                ipc_put_rgb_sync_tx(frame, CMK_RGB_SYNC_STATE_LEN);
            }
        }
        // Phase 24b: forward the MASTER's local key hits to the slave (one IPC per
        // hit). The ring is only fed while MASTER, so this is a no-op otherwise.
        uint8_t hit_packed;
        while (ipc_rgb_hit_tx_pop(&hit_packed)) {
            ipc_push_to_core1(IPC_ENCODE(IPC_SEND_RGB_HIT, hit_packed));
        }
#endif
#endif

        // Deferred IPC actions — set by the public API (keyscan task context).
        // Executed here (CM task) to keep all IPC pushes to Core 1 within
        // a single owner task, avoiding cross-task FIFO contention.

        // Low-power DEEP-sleep prepare: tell Core 1 to power the radio/CYW43 down.
        // One-way — the wake path reboots the chip (no power-on IPC).
        if (s_deferred_radio_off &&
            ipc_push_to_core1(IPC_ENCODE(IPC_BLE_POWER_OFF, 0))) {
            s_deferred_radio_off = false;
            DBG_STATUS("[CM] Deferred: IPC_BLE_POWER_OFF (deep sleep)\n");
        }

        // DS disconnect — must fire first so DS goes IDLE before ADV starts.
        // The actual ADV (directed or general) will execute on a subsequent
        // iteration after IPC_PERIPH_DISCONNECTED sets s_ds_ble = IDLE.
        if (s_deferred_disconnect_ds) {
            if (s_ds_ble != CM_DS_BLE_CONNECTED) {
                s_deferred_disconnect_ds = false;   // nothing to disconnect
            } else if (ipc_push_to_core1(IPC_ENCODE(IPC_DISCONNECT_PERIPH, 0))) {
                s_deferred_disconnect_ds = false;
                DBG_STATUS("[CM] Deferred: IPC_DISCONNECT_PERIPH inviato (switch slot)\n");
                // Don't clear s_ds_ble here — wait for IPC_PERIPH_DISCONNECTED from Core 1
            }   // push failed → retry next iteration
        }

        // Retry a lost IPC_DELETE_BONDING notify — the addresses already sit in
        // the delete-bonding ring; without the notify Core 1 would never purge
        // them and a stale LTK would survive the bond clear (re-encryption 0x13).
        if (s_delbond_notify_pending &&
            ipc_push_to_core1(IPC_ENCODE(IPC_DELETE_BONDING, 0))) {
            s_delbond_notify_pending = false;
        }

        if (s_deferred_reconnect_us) {
            // Don't start US reconnect while DS is advertising —
            // CYW43 can't do ADV + Initiating simultaneously.
            // Convert to pending; auto-restart will pick it up when DS finishes.
            if (s_ds_ble == CM_DS_BLE_ADVERTISING || s_ds_ble == CM_DS_BLE_ADV_DIRECTED) {
                s_deferred_reconnect_us = false;
                s_us_reconnect_pending = true;
                // Reset to IDLE so auto-restart check can fire later
                // (the actual gap_connect was already cancelled by Core 1)
                s_upstream = CM_UPSTREAM_IDLE;
                DBG_STATUS("[CM] US reconnect → pending (DS sta advertisando)\n");
            } else if (ipc_push_to_core1(IPC_ENCODE(IPC_RECONNECT_UPSTREAM, 0))) {
                s_deferred_reconnect_us = false;
                s_upstream = CM_UPSTREAM_CONNECTING;
                DBG_STATUS("[CM] Deferred: IPC_RECONNECT_UPSTREAM inviato\n");
            }   // push failed → flag stays set, retry next iteration
        }
        if (s_deferred_adv_general && s_ds_ble != CM_DS_BLE_CONNECTED) {
            // Guard: a stale IPC_PERIPH_ADV_TIMEOUT drained earlier in this same
            // iteration can have reset the pending slot. Encoding -1 would reach
            // Core 1 as slot 255 → set_slot_address no-op → ADV with the WRONG
            // slot identity. Abort instead.
            if (s_pending_ds_slot < 0) {
                s_deferred_adv_general = false;
                DBG_STATUS("[CM] Deferred ADV general scartato: slot pendente non valido\n");
            } else {
            // bonding_store I2C write runs here (CM task), never from keyscan task.
            // has_ds() guard makes this idempotent across push retries.
            {
                uint8_t del_slot = (uint8_t)s_pending_ds_slot;
                // Purge the SM bond too (see s_deferred_scan_general for rationale).
                if (bonding_store_has_ds(del_slot)) {
                    uint8_t del_addr[6], del_type;
                    bonding_store_get_ds(del_slot, del_addr, &del_type);
                    if (ipc_put_delete_bonding_addr(del_addr, del_type) &&
                        !ipc_push_to_core1(IPC_ENCODE(IPC_DELETE_BONDING, 0))) {
                        s_delbond_notify_pending = true;
                    }
                    bonding_store_clear_ds(del_slot);
                }
            }
            // DS has priority: if US is connecting, mark it for auto-restart.
            // Core 1 will cancel gap_connect() before enabling ADV.
            if (s_upstream == CM_UPSTREAM_CONNECTING && bonding_store_has_us()) {
                s_us_reconnect_pending = true;
                DBG_STATUS("[CM] US reconnect sospeso — DS ADV ha priorità\n");
            }
            if (ipc_push_to_core1(IPC_ENCODE(IPC_START_ADV, (uint32_t)s_pending_ds_slot))) {
                s_deferred_adv_general = false;
                s_ds_ble = CM_DS_BLE_ADVERTISING;
                DBG_STATUS("[CM] Deferred: clear DS slot %d + IPC_START_ADV inviato\n", s_pending_ds_slot);
            }   // push failed → retry next iteration (state NOT advanced)
            }
        }
        if (s_deferred_scan_general) {
            // Purge the SM bond too, not just the address. Clearing only the address
            // leaves the stale LTK in the TLV store → the next connect re-encrypts and
            // fails ("re-encryption failed 0x13"). Queue the addr for Core 1 to delete.
            // has_us() guard → idempotent across push retries.
            if (bonding_store_has_us()) {
                uint8_t del_addr[6], del_type;
                bonding_store_get_us(del_addr, &del_type);
                if (ipc_put_delete_bonding_addr(del_addr, del_type) &&
                    !ipc_push_to_core1(IPC_ENCODE(IPC_DELETE_BONDING, 0))) {
                    s_delbond_notify_pending = true;
                }
                // bonding_store I2C write runs here (CM task), never from keyscan task
                bonding_store_clear_us();
            }
            // Clear any pending US reconnect — user explicitly wants a new scan
            s_us_reconnect_pending = false;
            s_us_rapid_disconnect_count = 0;   // user action resets the backoff
            if (s_upstream != CM_UPSTREAM_IDLE) {
                // R4: bond cleared but the link / connect attempt is still up —
                // tear the central side down first; IPC_CENTRAL_DISCONNECTED
                // then starts the scan (s_scan_after_us_disconnect).
                if (ipc_push_to_core1(IPC_ENCODE(IPC_DISCONNECT_CENTRAL, 0))) {
                    s_deferred_scan_general    = false;
                    s_scan_after_us_disconnect = true;
                    DBG_STATUS("[CM] Deferred scan: teardown US prima dello scan\n");
                }   // push failed → retry next iteration
            } else if (ipc_push_to_core1(IPC_ENCODE(IPC_START_SCAN, 0))) {
                s_deferred_scan_general = false;
                s_upstream = CM_UPSTREAM_SCANNING;
                DBG_STATUS("[CM] Deferred: clear US + IPC_START_SCAN inviato\n");
            }   // push failed → retry next iteration
        }
        if (s_deferred_clear_all) {
            s_deferred_clear_all = false;
            // Factory reset: clear every bond record AND its SM keys. Runs in the CM
            // task (not keyscan) so the 258-byte EEPROM write buffer and the FIFO push
            // are safe. Queue all identity addresses first, then one IPC_DELETE_BONDING.
            bool any = false;
            for (uint8_t s = 0; s < BONDING_DS_SLOTS; s++) {
                if (bonding_store_has_ds(s)) {
                    uint8_t del_addr[6], del_type;
                    bonding_store_get_ds(s, del_addr, &del_type);
                    if (ipc_put_delete_bonding_addr(del_addr, del_type)) any = true;
                }
                bonding_store_clear_ds(s);
            }
            if (bonding_store_has_us()) {
                uint8_t del_addr[6], del_type;
                bonding_store_get_us(del_addr, &del_type);
                if (ipc_put_delete_bonding_addr(del_addr, del_type)) any = true;
            }
            bonding_store_clear_us();
            s_us_reconnect_pending = false;
            s_us_rapid_disconnect_count = 0;
            if (any && !ipc_push_to_core1(IPC_ENCODE(IPC_DELETE_BONDING, 0))) {
                s_delbond_notify_pending = true;   // notify retried by the loop
            }
            // R4: drop any live link too — its bond no longer exists (best
            // effort; the reconciliation heals a lost push).
            if (s_upstream != CM_UPSTREAM_IDLE) {
                ipc_push_to_core1(IPC_ENCODE(IPC_DISCONNECT_CENTRAL, 0));
            }
            if (s_ds_ble == CM_DS_BLE_CONNECTED) {
                ipc_push_to_core1(IPC_ENCODE(IPC_DISCONNECT_PERIPH, 0));
            }
            DBG_STATUS("[CM] Deferred: factory reset (all bonds + SM keys cleared)\n");
        }
        if (s_deferred_adv_directed && s_ds_ble != CM_DS_BLE_CONNECTED) {
            // Guard: same -1 → slot-255 hazard as the general ADV block above.
            if (s_pending_ds_slot < 0) {
                s_deferred_adv_directed   = false;
                s_deferred_is_slot_switch = false;
                DBG_STATUS("[CM] Deferred ADV directed scartato: slot pendente non valido\n");
            } else {
            // DS has priority: if US is connecting, mark it for auto-restart.
            if (s_upstream == CM_UPSTREAM_CONNECTING && bonding_store_has_us()) {
                s_us_reconnect_pending = true;
                DBG_STATUS("[CM] US reconnect sospeso — DS directed ADV ha priorità\n");
            }
            // directed addr already set by the caller (slot-switch or auto-reconnect)
            if (ipc_push_to_core1(IPC_ENCODE(IPC_START_ADV_DIRECTED, (uint32_t)s_pending_ds_slot))) {
                s_deferred_adv_directed   = false;
                s_deferred_is_slot_switch = false;
                s_ds_ble = CM_DS_BLE_ADV_DIRECTED;
                DBG_STATUS("[CM] Deferred: IPC_START_ADV_DIRECTED inviato (slot=%d)\n", s_pending_ds_slot);
            }   // push failed → retry next iteration (state NOT advanced)
            }
        }

        // --- US auto-restart ---
        // When DS finishes (connected or idle) and US reconnect is pending,
        // re-send IPC_RECONNECT_UPSTREAM. CYW43 can handle established
        // Connection + Initiating; the conflict is only ADV + Initiating.
        if (s_us_reconnect_pending
            && s_upstream == CM_UPSTREAM_IDLE
            && s_ds_ble != CM_DS_BLE_ADVERTISING
            && s_ds_ble != CM_DS_BLE_ADV_DIRECTED) {
            if (bonding_store_has_us()) {
                uint8_t addr[6];
                uint8_t addr_type;
                bonding_store_get_us(addr, &addr_type);
                ipc_set_reconnect_addr(addr, addr_type);
                if (ipc_push_to_core1(IPC_ENCODE(IPC_RECONNECT_UPSTREAM, 0))) {
                    s_us_reconnect_pending = false;
                    s_upstream = CM_UPSTREAM_CONNECTING;
                    DBG_STATUS("[CM] US reconnect auto-restart (DS completato)\n");
                }   // push failed → pending stays set, retry next iteration
            } else {
                s_us_reconnect_pending = false;
            }
        }

        cm_check_auto();

        // --- BLE Battery Service feed ---
        // Mirror the fuel-gauge SoC into the GATT Battery Service. Pushed only
        // on change (SoC moves slowly) so the FIFO stays quiet. The CM task is
        // the single owner of the Core 0 → Core 1 FIFO, so this push is safe here.
        {
            static int16_t s_last_soc_sent = -1;
            pm_battery_status_t bat;
            pm_get_battery_status(&bat);
            if (bat.valid && (int16_t)bat.soc_pct != s_last_soc_sent &&
                ipc_push_to_core1(IPC_ENCODE(IPC_SET_BATTERY_LEVEL, bat.soc_pct))) {
                s_last_soc_sent = (int16_t)bat.soc_pct;   // committed only on success
            }
        }

#if DEBUG_DIAG
        // --- Telemetry dump (R1) ---
        // Print every non-zero drop/error counter at a slow cadence. Silent when
        // everything is zero, so a quiet line means no losses since boot.
        {
            static TickType_t s_next_diag = 0;
            TickType_t now = xTaskGetTickCount();
            if ((int32_t)(now - s_next_diag) >= 0) {
                s_next_diag = now + pdMS_TO_TICKS(10000);
                diag_dump();
            }
        }
#endif

        // --- Split-mode sync to Core 1 ---
        // The BLE split runtime is gated by eeconfig split_mode (Core 1 owns the gate).
        // Push it on change only (boot sets the initial value; CMK_SPLIT_TOGG flips it).
        // CM task is the sole FIFO owner, so this push is safe here (not from keyscan).
        // A runtime flip hot-switches the split role on the live link (subscribe/
        // unsubscribe GATT) — no disconnect, so no auto-reconnect guard is needed.
        {
            static int8_t s_last_split_sent = -1;
            int8_t split_now = (int8_t)(eeconfig_read_split_mode() ? 1 : 0);
            if (split_now != s_last_split_sent &&
                ipc_push_to_core1(IPC_ENCODE(IPC_SET_SPLIT_MODE, (uint32_t)split_now))) {
                s_last_split_sent = split_now;   // committed only on success
            }
        }

        DIAG_TASK_ALIVE(DIAG_TASK_CM);
        vTaskDelay(pdMS_TO_TICKS(CM_LOOP_MS));
    }
}

// ---------------------------------------------------------------------------
// API pubblica — azioni combo (Core 0 only, chiamate da cmk_keycodes.c)
// ---------------------------------------------------------------------------

bool connection_manager_ds_adv_directed(uint8_t slot) {
    if (slot >= BONDING_DS_SLOTS) return false;
    if (!bonding_store_has_ds(slot)) {
        DBG_STATUS("[CM] Td%u short: nessun addr salvato\n", slot);
        return false;
    }
    // RAM-only reads — safe from any task context
    uint8_t addr[6];
    uint8_t addr_type;
    bonding_store_get_ds(slot, addr, &addr_type);
    ipc_set_directed_adv_addr(addr, addr_type);
    // R4: publish the request ATOMICALLY (keyscan task vs CM task) and make the
    // two ADV requests mutually exclusive — a new request supersedes the other
    // (last-wins). Before: two flags + one shared slot could interleave into an
    // ADV with the wrong slot/address pairing.
    taskENTER_CRITICAL();
    s_deferred_adv_general    = false;        // directed supersedes general
    s_pending_ds_slot         = (int8_t)slot;
    s_deferred_is_slot_switch = true;
    s_deferred_adv_directed   = true;         // commit flag last
    s_ds_rapid_disconnect_count = 0;          // user action resets the backoff
    if (s_ds_ble == CM_DS_BLE_CONNECTED) s_deferred_disconnect_ds = true;
    taskEXIT_CRITICAL();
    DBG_STATUS("[CM] Td%u short: directed ADV slot %u — differito al CM task\n", slot, slot);
    return true;
}

void connection_manager_ds_adv_general(uint8_t slot) {
    if (slot >= BONDING_DS_SLOTS) return;
    // Defer both bonding_store I2C write AND ipc_push to CM task.
    // bonding_store_clear_ds() calls eeprom_write() with a 258-byte stack
    // buffer — too deep for the keyscan task stack (1024 words).
    // R4: atomic publish + mutual exclusion (see ds_adv_directed).
    taskENTER_CRITICAL();
    s_deferred_adv_directed   = false;        // general supersedes directed
    s_deferred_is_slot_switch = false;
    s_pending_ds_slot         = (int8_t)slot;
    s_deferred_adv_general    = true;         // commit flag last
    s_ds_rapid_disconnect_count = 0;          // user action resets the backoff
    if (s_ds_ble == CM_DS_BLE_CONNECTED) s_deferred_disconnect_ds = true;
    taskEXIT_CRITICAL();
    DBG_STATUS("[CM] Td%u long: clear DS slot %u — differito al CM task\n", slot, slot);
}

void connection_manager_us_scan_general(void) {
    // Same reasoning: defer bonding_store I2C write to CM task.
    s_us_rapid_disconnect_count = 0;   // user action resets the backoff (R4)
    s_deferred_scan_general = true;
    DBG_STATUS("[CM] Tu long: clear US addr — differito al CM task\n");
}

void connection_manager_us_reconnect(void) {
    // Tu short: manual upstream re-link (mirror of Td short). With a bond →
    // directed reconnect; without → general scan (the deferred path's bond
    // clear is a no-op when no bond exists). No-op while a link is already
    // up or a connect is in flight.
    if (s_upstream == CM_UPSTREAM_CONNECTED ||
        s_upstream == CM_UPSTREAM_CONNECTING) {
        DBG_STATUS("[CM] Tu short: US già %s — ignorato\n",
                   s_upstream == CM_UPSTREAM_CONNECTED ? "connesso" : "in connessione");
        return;
    }
    s_us_rapid_disconnect_count = 0;       // user action resets the backoff
    if (bonding_store_has_us()) {
        uint8_t addr[6], addr_type;
        bonding_store_get_us(addr, &addr_type);
        ipc_set_reconnect_addr(addr, addr_type);
        s_deferred_reconnect_us = true;    // CM task pushes IPC_RECONNECT_UPSTREAM
        DBG_STATUS("[CM] Tu short: reconnect diretto al bond US — differito\n");
    } else {
        s_deferred_scan_general = true;    // no bond → plain general scan
        DBG_STATUS("[CM] Tu short: nessun bond US → scan generale — differito\n");
    }
}

void connection_manager_request_radio(bool on) {
    // Low-power DEEP enter. Called from the low_power module (keyscan task): set a
    // deferred flag only — the CM task owns the FIFO to Core 1 (R3). There is no
    // power-on (the wake path reboots the chip); on==true just cancels a pending
    // power-down that has not been sent yet.
    if (on) {
        s_deferred_radio_off = false;
    } else {
        s_deferred_radio_off = true;
    }
}

void connection_manager_clear_all_bonds(void) {
    // Defer to the CM task: bonding_store clears (258-byte EEPROM write buffer) and the
    // SM-key delete (FIFO push) must never run from the keyscan task. Set a flag only.
    s_deferred_clear_all = true;
    DBG_STATUS("[CM] EE_CLR: factory reset — differito al CM task\n");
}

// ---------------------------------------------------------------------------
// API pubblica — init / create
// ---------------------------------------------------------------------------

void connection_manager_init(void) {
    usb_output_init();
}

void connection_manager_create_task(void) {
    xTaskCreate(cm_task, "conn_mgr", CM_TASK_STACK, NULL, CM_TASK_PRIORITY, NULL);
}

cm_upstream_state_t connection_manager_upstream_state(void) { return s_upstream; }
cm_ds_ble_state_t   connection_manager_ds_ble_state(void)   { return s_ds_ble;   }
cm_ds_usb_state_t   connection_manager_ds_usb_state(void)   { return s_ds_usb;   }

cm_ds_slot_state_t connection_manager_ds_slot_state(uint8_t slot) {
    if (slot >= BONDING_DS_SLOTS) return CM_SLOT_EMPTY;
    // Snapshot volatile reads once — coherent enough for LED purposes.
    int8_t            pending = s_pending_ds_slot;
    cm_ds_ble_state_t ds      = s_ds_ble;
    // ADV states live on the PENDING slot (the one the radio is working on)…
    if (ds == CM_DS_BLE_ADVERTISING  && pending == (int8_t)slot) return CM_SLOT_ADV_GENERAL;
    if (ds == CM_DS_BLE_ADV_DIRECTED && pending == (int8_t)slot) return CM_SLOT_ADV_DIRECTED;
    // …a live connection on the ACTIVE slot.
    if (ds == CM_DS_BLE_CONNECTED && bonding_store_get_active_slot() == slot) {
        return CM_SLOT_CONNECTED;
    }
    return bonding_store_has_ds(slot) ? CM_SLOT_SAVED : CM_SLOT_EMPTY;
}
cm_split_role_t     connection_manager_split_role(void)     { return s_split_role;  }
uint8_t             connection_manager_remote_side(void)    { return s_remote_side; }
