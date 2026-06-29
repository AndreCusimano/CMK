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
// ipc_core.h — Inter-core communication between Core 0 (FreeRTOS) and Core 1 (BTstack)
//
// Protocol: 32-bit messages via hardware multicore FIFO.
//   bits [31:24] = message type (ipc_msg_type_t)
//   bits [23:0]  = inline payload (connection handle, error code, ring-buffer slot index)
//
// For HID reports (>4 bytes), the payload is the ring-buffer slot index.
// The ring buffer is protected by a hardware spinlock.
//

#ifndef CMK_IPC_CORE_H
#define CMK_IPC_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// Message types
// ---------------------------------------------------------------------------

typedef enum {
    // Core 1 → Core 0
    IPC_BLE_READY            = 0x01,  // BTstack init done (stack up, radio not yet ready)
    IPC_BLE_WORKING          = 0x02,  // HCI_STATE_WORKING: radio ready, can now ADV/SCAN
    IPC_PERIPH_ADVERTISING   = 0x03,  // started advertising
    IPC_PERIPH_CONNECTED     = 0x04,  // downstream device connected; payload=con_handle lsb
    IPC_PERIPH_DISCONNECTED  = 0x05,  // downstream device disconnected
    IPC_CENTRAL_SCANNING     = 0x06,  // started scanning
    IPC_CENTRAL_FOUND        = 0x07,  // found a candidate; connecting…
    IPC_CENTRAL_CONNECTED    = 0x08,  // HID service on upstream device ready; payload=hids_cid
    IPC_CENTRAL_DISCONNECTED = 0x09,  // upstream device disconnected
    IPC_HID_REPORT           = 0x0A,  // HID report from upstream; payload=ring-buffer slot index
    IPC_PERIPH_ADV_TIMEOUT   = 0x0B,  // advertising timed out (ADV_TIMEOUT_MS), no downstream connected
    IPC_CENTRAL_SCAN_TIMEOUT = 0x0C,  // scan timed out (SCAN_TIMEOUT_MS), no upstream found
    IPC_SAVE_UPSTREAM_ADDR      = 0x0D,  // upstream connected: addr in shared buffer → Core 0 saves to flash
    IPC_CENTRAL_CONNECT_TIMEOUT = 0x0E, // gap_connect() timed out (CONNECT_TIMEOUT_MS), no upstream
    IPC_PERIPH_CONNECTED_ADDR   = 0x0F, // downstream connected: peer addr in shared buffer → Core 0 saves

    // Core 0 → Core 1
    IPC_SEND_TEST_KEY        = 0x10,  // send a test keystroke; payload=HID keycode (0=key-up) // (R7: handler rimosso — nessun sender)
    IPC_START_ADV            = 0x11,  // start general advertising (undirected)
    IPC_START_SCAN           = 0x12,  // start general scanning (all addresses)
    IPC_STOP_ADV             = 0x13,  // stop advertising explicitly (futuro)
    IPC_STOP_SCAN            = 0x14,  // stop scanning explicitly (futuro)
    IPC_SEND_KBD_REPORT      = 0x15,  // full 8-byte keyboard HID report; data in shared buffer
    IPC_RECONNECT_UPSTREAM   = 0x16,  // reconnect to saved upstream addr (no timeout, forever)
    IPC_START_ADV_DIRECTED   = 0x17,  // start directed ADV (high duty) to peer addr in shared buffer
    IPC_DISCONNECT_PERIPH    = 0x18,  // disconnect downstream peripheral (gap_disconnect)
    IPC_SEND_MOUSE_REPORT    = 0x19,  // full 7-byte mouse HID report; data in shared buffer
    IPC_SEND_PB_REPORT       = 0x1A,  // full 4-byte programmable button report; data in shared buffer
    IPC_SET_BATTERY_LEVEL    = 0x1C,  // BLE Battery Service level; payload = SoC 0..100

    // Core 1 → Core 0
    IPC_LED_STATE            = 0x1B,  // host LED indicator state; payload = LED bitmask byte

    // Split keyboard (Phase 18) — Core 1 → Core 0 role transitions. The role is
    // decided by BLE (discovery/subscription); Core 0 (connection_manager) tracks
    // it. STANDALONE = behave as a normal single half (current behaviour).
    IPC_SPLIT_ROLE_STANDALONE = 0x1D,  // not in a split (peer absent / non-CMK upstream)
    IPC_SPLIT_ROLE_MASTER     = 0x1E,  // we resolve the unified keymap; payload = peer side (0/1)
    IPC_SPLIT_ROLE_SLAVE      = 0x1F,  // we only ship raw matrix events; payload = peer side (0/1)

    // Split matrix transport (Phase 18 Milestone C). Payload = matrix-event byte
    // (bit7=pressed, bits5:3=row, bits2:0=col) or a 24-bit snapshot bitmap.
    IPC_SPLIT_TX_EVENT        = 0x20,  // C0→C1: SLAVE keyscan change → notify to master
    IPC_SPLIT_RX_EVENT        = 0x21,  // C1→C0: MASTER received a matrix event from slave
    IPC_SPLIT_RX_SNAPSHOT     = 0x22,  // C1→C0: MASTER read slave's full 24-bit pressed bitmap

    // Bond management (re-association fix). Payload unused — addresses to delete
    // are queued in the delete-bonding ring buffer first.
    IPC_DELETE_BONDING        = 0x23,  // C0→C1: gap_delete_bonding() for each queued addr

    // Split enable (Phase 18). Payload = split_mode (0/1). Pushed by the CM task on
    // change (boot + CMK_SPLIT_TOGG); Core 1 gates split discovery / SLAVE acceptance.
    IPC_SET_SPLIT_MODE        = 0x24,  // C0→C1: enable/disable the BLE split runtime
    IPC_CMD_REJECTED          = 0x25,  // C1→C0: command refused (payload = cmd type) — R4
    IPC_DISCONNECT_CENTRAL    = 0x26,  // C0→C1: tear down upstream central (cancel/disconnect/stop scan) — R4

    // Strict split-mode matching: the upstream peer IS a CMK sibling but its
    // split_mode disagrees with ours → the halves do NOT cooperate (no split,
    // no relay of the sibling's HID). Central-side only (the peripheral can't
    // observe the central's mode). Payload = peer side (0/1).
    IPC_SPLIT_ROLE_MISMATCH   = 0x27,  // C1→C0: sibling found, split_mode mismatch

    // Low-power DEEP sleep prepare. Pushed by the CM task (deferred flag, R3)
    // when low_power enters DEEP. Core 1 stops the radio + powers the CYW43 down
    // and halts. There is NO power-on counterpart: Core 0 reboots the chip on the
    // wake keypress (watchdog), so reconnect runs via the clean boot path.
    IPC_BLE_POWER_OFF         = 0x28,  // C0→C1: deep-sleep power down (hci off + cyw43 deinit)

    // RGB cross-half sync (Phase 24) — MASTER→SLAVE rgb_sync GATT channel. The
    // frame (state or hit, ≤16 B) lives in the rgb_sync frame buffer; payload unused.
    IPC_SEND_RGB_SYNC         = 0x29,  // C0→C1: MASTER writes the RGB state frame to the slave
    IPC_RGB_SYNC_RX           = 0x2A,  // C1→C0: SLAVE received an RGB frame from the master
    IPC_SEND_RGB_HIT          = 0x2B,  // C0→C1: MASTER forwards a local key hit; payload = packed hit
} ipc_msg_type_t;

// ---------------------------------------------------------------------------
// Message encoding/decoding helpers
// ---------------------------------------------------------------------------

#define IPC_ENCODE(type, payload) (((uint32_t)(type) << 24) | ((uint32_t)(payload) & 0x00FFFFFFu))
#define IPC_TYPE(msg)             ((ipc_msg_type_t)(((msg) >> 24) & 0xFF))
#define IPC_PAYLOAD(msg)          ((uint32_t)((msg) & 0x00FFFFFFu))

// R4 — command-level feedback + explicit central teardown:
//   IPC_CMD_REJECTED   (C1→C0, 0x25): Core 1 refused a command because its state
//                      didn't allow it (payload = the rejected command type).
//                      Core 0 rolls back its optimistic state instead of waiting
//                      forever for a completion that will never come.
//   IPC_DISCONNECT_CENTRAL (C0→C1, 0x26): tear down the upstream central side
//                      (cancel connect / disconnect link / stop scan + kill the
//                      reconnect retry timer). Used by "clear US bond + rescan"
//                      and factory reset — before R4 no such command existed and
//                      the link to a just-unbonded device stayed up.

// Split matrix-event byte (Phase 18): bit7=pressed, bits5:3=row(0..3), bits2:0=col(0..5).
// Shared by keyscan (Core 0) and BLE (Core 1) and the on-wire matrix_event char.
#define CMK_SPLIT_EVT(pressed, row, col) \
    ((uint8_t)(((pressed) ? 0x80u : 0x00u) | (((row) & 0x07u) << 3) | ((col) & 0x07u)))
#define CMK_SPLIT_EVT_PRESSED(b)  (((b) & 0x80u) != 0u)
#define CMK_SPLIT_EVT_ROW(b)      (uint8_t)(((b) >> 3) & 0x07u)
#define CMK_SPLIT_EVT_COL(b)      (uint8_t)((b) & 0x07u)

// RGB cross-half sync wire frame (Phase 24), carried over the rgb_sync GATT char.
// byte[0] = tag; the rest is tag-specific. Single fixed-size buffer (≤16 B).
//   STATE (13 B): [tag][enable][mode][hue][sat][val][speed][wave_speed]
//                 [wave_width][fade][hue_shift][phase_lo][phase_hi]
//   HIT    (4 B): [tag][side][row][phys_col]   (Phase 24b — reactive cross-half)
#define CMK_RGB_SYNC_TAG_STATE  0x01u
#define CMK_RGB_SYNC_TAG_HIT    0x02u
#define CMK_RGB_SYNC_FRAME_MAX  16
#define CMK_RGB_SYNC_STATE_LEN  13
#define CMK_RGB_SYNC_HIT_LEN    4

// Packed reactive hit (Phase 24b): bit5=side(0=L,1=R), bits4:3=row(0..3), bits2:0=col(0..5).
// Carried in the IPC_SEND_RGB_HIT payload and the Core-0-internal hit rings.
#define CMK_RGB_HIT(side, row, col) \
    ((uint8_t)((((side) & 1u) << 5) | (((row) & 3u) << 3) | ((col) & 7u)))
#define CMK_RGB_HIT_SIDE(b)  (uint8_t)(((b) >> 5) & 1u)
#define CMK_RGB_HIT_ROW(b)   (uint8_t)(((b) >> 3) & 3u)
#define CMK_RGB_HIT_COL(b)   (uint8_t)((b) & 7u)

// ---------------------------------------------------------------------------
// HID report ring buffer (shared between Core 0 and Core 1)
// ---------------------------------------------------------------------------

#define IPC_HID_SLOTS      4    // number of concurrent slots
#define IPC_HID_SLOT_SIZE  64   // max bytes per HID report

typedef struct {
    uint8_t  data[IPC_HID_SLOT_SIZE];
    uint8_t  len;
    volatile bool valid;
} ipc_hid_slot_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Called once from Core 0 before launching Core 1
void ipc_init(void);

// Core 1 → Core 0: push a simple 32-bit message (non-blocking, returns false if full)
bool ipc_push_to_core0(uint32_t msg);

// Core 0: check if a message is available and pop it (non-blocking)
bool ipc_pop_from_core1(uint32_t *msg_out);

// Core 0 → Core 1: push a command (non-blocking, returns false if full)
bool ipc_push_to_core1(uint32_t msg);

// Core 1: check if a command is available and pop it (non-blocking)
bool ipc_pop_from_core0(uint32_t *msg_out);

// Core 1: write a HID report to the ring buffer and push IPC_HID_REPORT
// Returns false if no slot is available (report dropped)
bool ipc_put_hid_report(const uint8_t *data, uint8_t len);

// Core 0: read a HID report from the ring buffer slot indicated by the IPC payload
void ipc_get_hid_report(uint32_t slot_idx, uint8_t *data_out, uint8_t *len_out);

// Core 0 (R3): pop the next pending HID report in ring order, independent of the
// IPC notify. Lets the CM task drain the ring even when an IPC_HID_REPORT notify
// was lost (full FIFO) — a lost notify becomes latency, not a lost report.
bool ipc_pop_hid_report(uint8_t *data_out, uint8_t *len_out);

// ---------------------------------------------------------------------------
// BLE status slot (R3) — LEVEL-based state published by Core 1, sampled by the
// CM task. The FIFO events remain the low-latency triggers, but a lost event no
// longer leaves Core 0 with a divergent state forever: the CM reconciles its
// own state machine against this snapshot. Values use the ble_manager.h enums
// (periph_state_t / central_state_t) and the split role (0/1/2 = standalone/
// master/slave). `valid` flips true at the first publish.
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t periph;        // periph_state_t
    uint8_t central;       // central_state_t
    uint8_t active_slot;   // DS slot whose identity is currently set
    uint8_t split_role;    // 0=standalone 1=master 2=slave
    bool    valid;
} ipc_ble_status_t;

// Core 1: publish the current BLE state (called every IPC poll tick — cheap).
void ipc_publish_ble_status(const ipc_ble_status_t *st);

// Core 0: sample the last published state. out->valid==false until Core 1 publishes.
void ipc_get_ble_status(ipc_ble_status_t *out);

// Keyboard report FIFO — Core 0 writes, Core 1 reads. A ring (not single-slot) so a
// burst of fast typing isn't collapsed: the BLE link sends one report per connection
// interval (~7.5-15 ms) and Core 1 polls one IPC msg per ~5 ms, so a single slot would
// lose every intermediate full-state report (e.g. a press+release within one interval).
#define IPC_KBD_SLOTS 16

// Core 0 → Core 1: enqueue a full 8-byte keyboard report. Coalescing FIFO push (only
// notifies Core 1 when the ring was empty). On overflow drops the OLDEST entry so the
// latest key state always survives. Returns false only if the FIFO push failed.
bool ipc_put_kbd_report(const uint8_t report[8]);

// Core 1: dequeue the next keyboard report. Returns false when the ring is empty.
bool ipc_get_kbd_report(uint8_t report_out[8]);

// Core 0 → Core 1: write a full 7-byte mouse HID report and push IPC_SEND_MOUSE_REPORT.
// Single-slot, last-wins (same pattern as keyboard report).
bool ipc_put_mouse_report(const uint8_t report[7]);

// Core 1: read the mouse report written by Core 0 (called on IPC_SEND_MOUSE_REPORT).
// Returns false if the slot was already consumed (stale coalesced notify) — the
// caller must NOT treat report_out as a valid state in that case.
bool ipc_get_mouse_report(uint8_t report_out[7]);

// Core 0 → Core 1: write a 4-byte programmable button report and push IPC_SEND_PB_REPORT.
// Single-slot, last-wins (same pattern as keyboard/mouse reports).
bool ipc_put_pb_report(const uint8_t report[4]);

// Core 1: read the programmable button report (called on IPC_SEND_PB_REPORT).
// Returns false if the slot was already consumed (stale coalesced notify).
bool ipc_get_pb_report(uint8_t report_out[4]);

// Core 1 → Core 0: write an upstream BLE address and push IPC_SAVE_UPSTREAM_ADDR.
// Single-slot, last-wins. Called after upstream HID service connects.
bool ipc_put_upstream_addr(const uint8_t addr[6], uint8_t addr_type);

// Core 0: read the upstream address written by Core 1 (called on IPC_SAVE_UPSTREAM_ADDR).
void ipc_get_upstream_addr(uint8_t addr_out[6], uint8_t *type_out);

// Core 0 → Core 1: write a reconnect address so Core 1 can call gap_connect() directly.
// Single-slot, last-wins. Caller then sends IPC_RECONNECT_UPSTREAM.
void ipc_set_reconnect_addr(const uint8_t addr[6], uint8_t addr_type);

// Core 1: read the reconnect address written by Core 0 (called on IPC_RECONNECT_UPSTREAM).
void ipc_get_reconnect_addr(uint8_t addr_out[6], uint8_t *type_out);

// Core 0 → Core 1: write the target peer address for directed ADV.
// Caller then sends IPC_START_ADV_DIRECTED.
void ipc_set_directed_adv_addr(const uint8_t addr[6], uint8_t addr_type);

// Core 1: read the directed ADV target address (called on IPC_START_ADV_DIRECTED).
void ipc_get_directed_adv_addr(uint8_t addr_out[6], uint8_t *type_out);

// Core 1 → Core 0: write the downstream peer address on connection and push IPC_PERIPH_CONNECTED_ADDR.
// Single-slot, last-wins. Called from GAP_SUBEVENT_LE_CONNECTION_COMPLETE (slave role).
void ipc_put_periph_addr(const uint8_t addr[6], uint8_t addr_type);

// Core 0: read the downstream peer address (called on IPC_PERIPH_CONNECTED_ADDR).
void ipc_get_periph_addr(uint8_t addr_out[6], uint8_t *type_out);

// Delete-bonding address queue — Core 0 enqueues identity addresses whose SM bond
// must be purged (gap_delete_bonding), Core 1 drains them on IPC_DELETE_BONDING.
// A ring (not single-slot) because a factory reset enqueues several addrs in a
// burst; a single-slot last-wins would race Core 1's read. SPSC, spinlock 14.
// Capacity is SLOTS-1 entries; 8 holds the worst case (3 DS slots + 1 US) with margin.
#define IPC_DELBOND_SLOTS 8

// Core 0: enqueue an address to delete. Returns false if the ring is full.
// Caller then pushes IPC_DELETE_BONDING (one push covers all queued addrs).
bool ipc_put_delete_bonding_addr(const uint8_t addr[6], uint8_t addr_type);

// Core 1: dequeue the next address to delete. Returns false when the ring is empty.
bool ipc_get_delete_bonding_addr(uint8_t addr_out[6], uint8_t *type_out);

// ---------------------------------------------------------------------------
// Split remote-event queue (Phase 18) — Core 0 internal SPSC ring.
// The CM task (sole producer) enqueues matrix-event bytes received from the slave
// over BLE (IPC_SPLIT_RX_EVENT); the keyscan task (sole consumer) drains them and
// injects them into the action pipeline. Keeps keymap/action mutation in the
// keyscan task only. Snapshot: last-wins single slot (IPC_SPLIT_RX_SNAPSHOT).
void ipc_split_push_remote_event(uint8_t evt);          // CM task
bool ipc_split_pop_remote_event(uint8_t *evt_out);       // keyscan task
void ipc_split_set_remote_snapshot(uint32_t bitmap24);   // CM task
bool ipc_split_take_remote_snapshot(uint32_t *bitmap_out); // keyscan task (true if one was pending)

// ---------------------------------------------------------------------------
// RGB cross-half sync frame buffers (Phase 24) — single-slot, last-wins, spinlock.
// MASTER→SLAVE: Core 0 writes the frame, Core 1 sends it over rgb_sync (TX path);
// the SLAVE's Core 1 receives it and Core 0 reads it (RX path). State is idempotent
// so last-wins is correct (the latest state always wins; a periodic resync heals).
// ---------------------------------------------------------------------------

// Core 0 → Core 1 (MASTER): stage the RGB frame to send + push IPC_SEND_RGB_SYNC
// (coalescing). Returns false only if the FIFO push failed.
bool ipc_put_rgb_sync_tx(const uint8_t *frame, uint8_t len);

// Core 1 (MASTER): read the staged frame. Returns false if already consumed.
bool ipc_get_rgb_sync_tx(uint8_t *frame_out, uint8_t *len_out);

// Reactive-hit rings (Phase 24b) — Core 0 internal SPSC, lock-free (power-of-two).
// TX: keyscan (action_exec, MASTER) enqueues a LOCAL key hit → CM forwards it to
//     the slave as an IPC_SEND_RGB_HIT.
// RX: CM enqueues a hit received FROM the master → keyscan applies it (record_hit_xy).
// Both carry a CMK_RGB_HIT() packed byte. Drop on full (a missed splash self-heals).
void ipc_rgb_hit_tx_push(uint8_t packed);   // keyscan/action (producer)
bool ipc_rgb_hit_tx_pop(uint8_t *out);       // CM (consumer)
void ipc_rgb_hit_rx_push(uint8_t packed);   // CM (producer)
bool ipc_rgb_hit_rx_pop(uint8_t *out);       // keyscan (consumer)

// Core 1 → Core 0 (SLAVE): stash a received RGB frame + push IPC_RGB_SYNC_RX.
bool ipc_put_rgb_sync_rx(const uint8_t *frame, uint8_t len);

// Core 0 (SLAVE): read the received frame (called on IPC_RGB_SYNC_RX).
bool ipc_get_rgb_sync_rx(uint8_t *frame_out, uint8_t *len_out);

// ---------------------------------------------------------------------------
// Log ring buffer — Core 1 writes formatted strings, Core 0 drains to UART
// ---------------------------------------------------------------------------

#define IPC_LOG_SLOTS       8     // ring buffer depth
#define IPC_LOG_SLOT_SIZE   128   // max chars per message (including '\0')

// Core 1: format and enqueue a log message (non-blocking, drops if full).
// Never touches UART — safe to call from BTstack callbacks and IRQ context.
void core1_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

// Core 0: drain all pending log messages to UART. Call from CM task loop.
void ipc_log_drain(void);

#endif // CMK_IPC_CORE_H
