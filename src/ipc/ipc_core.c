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
// ipc_core.c — Inter-core communication implementation
//

#include "ipc_core.h"
#include "diag.h"

#include "pico/multicore.h"
#include "pico/platform.h"
#include "hardware/sync.h"          // hardware spinlocks
#include "hardware/structs/sio.h"   // direct FIFO write (atomic push, R3)

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Hardware spinlocks — claim-free range 24..31 (R3). The previous IDs were
// hazardous: 14/15 are PICO_SPINLOCK_ID_OS1/OS2 (free only while FreeRTOS runs
// single-core) and 16+ is the SDK "striped" range handed out round-robin to
// every SDK mutex/semaphore. We claim ours explicitly so any future collision
// asserts at boot instead of silently sharing a lock.
//   24 = report rings + address shared slots + BLE status
//   25 = Log ring buffer (separate to avoid contention)
// ---------------------------------------------------------------------------

#define IPC_SPINLOCK_ID     24
#define IPC_LOG_SPINLOCK_ID 25

static spin_lock_t *s_lock     = NULL;
static spin_lock_t *s_log_lock = NULL;

// ---------------------------------------------------------------------------
// HID ring buffer (in SRAM, visible to both cores)
// ---------------------------------------------------------------------------

static volatile ipc_hid_slot_t s_hid_ring[IPC_HID_SLOTS];
static volatile uint32_t       s_hid_write_idx = 0;  // Core 1 writes here
static volatile uint32_t       s_hid_read_idx  = 0;  // Core 0 reads here

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

void ipc_init(void) {
    spin_lock_claim(IPC_SPINLOCK_ID);
    spin_lock_claim(IPC_LOG_SPINLOCK_ID);
    s_lock     = spin_lock_init(IPC_SPINLOCK_ID);
    s_log_lock = spin_lock_init(IPC_LOG_SPINLOCK_ID);

    for (int i = 0; i < IPC_HID_SLOTS; i++) {
        s_hid_ring[i].valid = false;
        s_hid_ring[i].len   = 0;
    }
    s_hid_write_idx = 0;
    s_hid_read_idx  = 0;
}

// ---------------------------------------------------------------------------
// Core 1 → Core 0 FIFO (messages about BLE events)
// ---------------------------------------------------------------------------

bool ipc_push_to_core0(uint32_t msg) {
    // multicore_fifo_push_blocking() pushes INTO the fifo that the OTHER core
    // reads — from Core 1's perspective this sends to Core 0. Core 1 is single
    // threaded (BTstack run loop), so no atomicity guard is needed here.
    if (!multicore_fifo_wready()) {
        DIAG_INC(fifo_c1c0_drop);
        return false;
    }
    multicore_fifo_push_blocking(msg);
    return true;
}

bool ipc_pop_from_core1(uint32_t *msg_out) {
    // Called by Core 0 to read what Core 1 sent
    if (!multicore_fifo_rvalid()) {
        return false;
    }
    *msg_out = multicore_fifo_pop_blocking();
    return true;
}

// ---------------------------------------------------------------------------
// Core 0 → Core 1 FIFO (commands from FreeRTOS tasks to BTstack)
// ---------------------------------------------------------------------------

bool ipc_push_to_core1(uint32_t msg) {
    // Called by Core 0 tasks to send commands to Core 1. MULTIPLE tasks produce
    // here (CM task + keyscan split TX) → the wready check and the FIFO write
    // must be one atomic step (R3): with the old check-then-push_blocking, two
    // tasks could both pass the check on one free slot and the loser would BLOCK
    // inside multicore_fifo_push_blocking (keyscan stalled up to the 5 ms drain).
    uint32_t save = spin_lock_blocking(s_lock);
    if (!multicore_fifo_wready()) {
        spin_unlock(s_lock, save);
        DIAG_INC(fifo_c0c1_drop);
        return false;
    }
    sio_hw->fifo_wr = msg;   // guaranteed non-blocking after the wready check
    __sev();                 // wake Core 1 (same as multicore_fifo_push_blocking)
    spin_unlock(s_lock, save);
    return true;
}

bool ipc_pop_from_core0(uint32_t *msg_out) {
    // Called by Core 1 to read commands from Core 0
    if (!multicore_fifo_rvalid()) {
        return false;
    }
    *msg_out = multicore_fifo_pop_blocking();
    return true;
}

// ---------------------------------------------------------------------------
// Keyboard report ring — Core 0 writes, Core 1 reads (SPSC FIFO, spinlock 14)
// A ring (not single-slot) so fast typing isn't collapsed before Core 1 can ship
// each full-state report over BLE (one per connection interval). See ipc_core.h.
// ---------------------------------------------------------------------------

static volatile uint8_t  s_kbd_ring[IPC_KBD_SLOTS][8];
static volatile uint32_t s_kbd_wr = 0;  // Core 0 producer
static volatile uint32_t s_kbd_rd = 0;  // Core 1 consumer

bool ipc_put_kbd_report(const uint8_t report[8]) {
    uint32_t save = spin_lock_blocking(s_lock);
    bool     was_empty = (s_kbd_wr == s_kbd_rd);
    uint32_t next      = (s_kbd_wr + 1) % IPC_KBD_SLOTS;
    if (next == s_kbd_rd) {
        // Full — drop the OLDEST so the latest key state always survives.
        s_kbd_rd = (s_kbd_rd + 1) % IPC_KBD_SLOTS;
        DIAG_INC(kbd_ring_overflow);
    }
    for (int i = 0; i < 8; i++) s_kbd_ring[s_kbd_wr][i] = report[i];
    s_kbd_wr = next;
    spin_unlock(s_lock, save);
    // Coalescing notify: only poke Core 1 when the ring went from empty → non-empty.
    // Core 1 drains the whole ring per IPC_SEND_KBD_REPORT, so extra pushes are wasted
    // FIFO traffic (and multicore_fifo_push_blocking can stall) — avoid them.
    if (was_empty) {
        if (!ipc_push_to_core1(IPC_ENCODE(IPC_SEND_KBD_REPORT, 0))) {
            DIAG_INC(kbd_notify_fail);
            return false;
        }
    }
    return true;
}

bool ipc_get_kbd_report(uint8_t report_out[8]) {
    uint32_t save = spin_lock_blocking(s_lock);
    if (s_kbd_rd == s_kbd_wr) {        // empty
        spin_unlock(s_lock, save);
        return false;
    }
    for (int i = 0; i < 8; i++) report_out[i] = (uint8_t)s_kbd_ring[s_kbd_rd][i];
    s_kbd_rd = (s_kbd_rd + 1) % IPC_KBD_SLOTS;
    spin_unlock(s_lock, save);
    return true;
}

// ---------------------------------------------------------------------------
// Mouse report slot — Core 0 writes, Core 1 reads (single-slot, last-wins)
// ---------------------------------------------------------------------------

static volatile uint8_t s_mouse_report[7];
static volatile bool    s_mouse_report_valid = false;

bool ipc_put_mouse_report(const uint8_t report[7]) {
    uint32_t save = spin_lock_blocking(s_lock);
    bool was_valid = s_mouse_report_valid;
    for (int i = 0; i < 7; i++) s_mouse_report[i] = report[i];
    s_mouse_report_valid = true;
    spin_unlock(s_lock, save);
    // Coalescing notify (R3): one FIFO message per invalid→valid transition.
    // The slot is last-wins, so back-to-back puts only need a single wake-up;
    // Core 1 also polls the slot every tick, so even a lost notify just adds
    // ≤5 ms of latency.
    if (!was_valid) return ipc_push_to_core1(IPC_ENCODE(IPC_SEND_MOUSE_REPORT, 0));
    return true;
}

bool ipc_get_mouse_report(uint8_t report_out[7]) {
    uint32_t save = spin_lock_blocking(s_lock);
    bool valid = s_mouse_report_valid;
    if (valid) {
        for (int i = 0; i < 7; i++) report_out[i] = (uint8_t)s_mouse_report[i];
        s_mouse_report_valid = false;
    }
    spin_unlock(s_lock, save);
    return valid;
}

// ---------------------------------------------------------------------------
// Programmable button report slot — Core 0 writes, Core 1 reads (single-slot, last-wins)
// ---------------------------------------------------------------------------

static volatile uint8_t s_pb_report[4];
static volatile bool    s_pb_report_valid = false;

bool ipc_put_pb_report(const uint8_t report[4]) {
    uint32_t save = spin_lock_blocking(s_lock);
    bool was_valid = s_pb_report_valid;
    for (int i = 0; i < 4; i++) s_pb_report[i] = report[i];
    s_pb_report_valid = true;
    spin_unlock(s_lock, save);
    // Coalescing notify (R3) — see mouse above.
    if (!was_valid) return ipc_push_to_core1(IPC_ENCODE(IPC_SEND_PB_REPORT, 0));
    return true;
}

bool ipc_get_pb_report(uint8_t report_out[4]) {
    uint32_t save = spin_lock_blocking(s_lock);
    bool valid = s_pb_report_valid;
    if (valid) {
        for (int i = 0; i < 4; i++) report_out[i] = (uint8_t)s_pb_report[i];
        s_pb_report_valid = false;
    }
    spin_unlock(s_lock, save);
    return valid;
}

// ---------------------------------------------------------------------------
// Consumer report slot (bug #6) — Core 0 writes, Core 1 reads (single-slot, last-wins)
// ---------------------------------------------------------------------------

static volatile uint8_t s_consumer_report[2];
static volatile bool    s_consumer_report_valid = false;

bool ipc_put_consumer_report(const uint8_t report[2]) {
    uint32_t save = spin_lock_blocking(s_lock);
    bool was_valid = s_consumer_report_valid;
    s_consumer_report[0] = report[0];
    s_consumer_report[1] = report[1];
    s_consumer_report_valid = true;
    spin_unlock(s_lock, save);
    // Coalescing notify (R3) — see mouse above.
    if (!was_valid) return ipc_push_to_core1(IPC_ENCODE(IPC_SEND_CONSUMER_REPORT, 0));
    return true;
}

bool ipc_get_consumer_report(uint8_t report_out[2]) {
    uint32_t save = spin_lock_blocking(s_lock);
    bool valid = s_consumer_report_valid;
    if (valid) {
        report_out[0] = (uint8_t)s_consumer_report[0];
        report_out[1] = (uint8_t)s_consumer_report[1];
        s_consumer_report_valid = false;
    }
    spin_unlock(s_lock, save);
    return valid;
}

// ---------------------------------------------------------------------------
// RGB cross-half sync frame buffers (Phase 24) — single-slot, last-wins.
// TX = Core 0 → Core 1 (MASTER sends); RX = Core 1 → Core 0 (SLAVE receives).
// ---------------------------------------------------------------------------

static volatile uint8_t s_rgb_tx_frame[CMK_RGB_SYNC_FRAME_MAX];
static volatile uint8_t s_rgb_tx_len   = 0;
static volatile bool    s_rgb_tx_valid = false;

bool ipc_put_rgb_sync_tx(const uint8_t *frame, uint8_t len) {
    if (len > CMK_RGB_SYNC_FRAME_MAX) len = CMK_RGB_SYNC_FRAME_MAX;
    uint32_t save = spin_lock_blocking(s_lock);
    bool was_valid = s_rgb_tx_valid;
    for (uint8_t i = 0; i < len; i++) s_rgb_tx_frame[i] = frame[i];
    s_rgb_tx_len   = len;
    s_rgb_tx_valid = true;
    spin_unlock(s_lock, save);
    // Coalescing notify (R3): only push when the slot was empty.
    if (!was_valid) return ipc_push_to_core1(IPC_ENCODE(IPC_SEND_RGB_SYNC, 0));
    return true;
}

bool ipc_get_rgb_sync_tx(uint8_t *frame_out, uint8_t *len_out) {
    uint32_t save = spin_lock_blocking(s_lock);
    bool valid = s_rgb_tx_valid;
    if (valid) {
        uint8_t len = s_rgb_tx_len;
        for (uint8_t i = 0; i < len; i++) frame_out[i] = (uint8_t)s_rgb_tx_frame[i];
        *len_out = len;
        s_rgb_tx_valid = false;
    } else {
        *len_out = 0;
    }
    spin_unlock(s_lock, save);
    return valid;
}

static volatile uint8_t s_rgb_rx_frame[CMK_RGB_SYNC_FRAME_MAX];
static volatile uint8_t s_rgb_rx_len   = 0;
static volatile bool    s_rgb_rx_valid = false;

bool ipc_put_rgb_sync_rx(const uint8_t *frame, uint8_t len) {
    if (len > CMK_RGB_SYNC_FRAME_MAX) len = CMK_RGB_SYNC_FRAME_MAX;
    uint32_t save = spin_lock_blocking(s_lock);
    bool was_valid = s_rgb_rx_valid;
    for (uint8_t i = 0; i < len; i++) s_rgb_rx_frame[i] = frame[i];
    s_rgb_rx_len   = len;
    s_rgb_rx_valid = true;
    spin_unlock(s_lock, save);
    if (!was_valid) return ipc_push_to_core0(IPC_ENCODE(IPC_RGB_SYNC_RX, 0));
    return true;
}

bool ipc_get_rgb_sync_rx(uint8_t *frame_out, uint8_t *len_out) {
    uint32_t save = spin_lock_blocking(s_lock);
    bool valid = s_rgb_rx_valid;
    if (valid) {
        uint8_t len = s_rgb_rx_len;
        for (uint8_t i = 0; i < len; i++) frame_out[i] = (uint8_t)s_rgb_rx_frame[i];
        *len_out = len;
        s_rgb_rx_valid = false;
    } else {
        *len_out = 0;
    }
    spin_unlock(s_lock, save);
    return valid;
}

// ---------------------------------------------------------------------------
// Upstream BLE address slot — Core 1 writes, Core 0 reads (single-slot, last-wins)
// ---------------------------------------------------------------------------

static volatile uint8_t s_upstream_addr[6]    = {0};
static volatile uint8_t s_upstream_addr_type  = 0;

bool ipc_put_upstream_addr(const uint8_t addr[6], uint8_t addr_type) {
    uint32_t save = spin_lock_blocking(s_lock);
    for (int i = 0; i < 6; i++) s_upstream_addr[i] = addr[i];
    s_upstream_addr_type = addr_type;
    spin_unlock(s_lock, save);
    return ipc_push_to_core0(IPC_ENCODE(IPC_SAVE_UPSTREAM_ADDR, 0));
}

void ipc_get_upstream_addr(uint8_t addr_out[6], uint8_t *type_out) {
    uint32_t save = spin_lock_blocking(s_lock);
    if (addr_out) {
        for (int i = 0; i < 6; i++) addr_out[i] = (uint8_t)s_upstream_addr[i];
    }
    if (type_out) *type_out = (uint8_t)s_upstream_addr_type;
    spin_unlock(s_lock, save);
}

// ---------------------------------------------------------------------------
// Reconnect address slot — Core 0 writes, Core 1 reads (single-slot, last-wins)
// ---------------------------------------------------------------------------

static volatile uint8_t s_reconnect_addr[6]   = {0};
static volatile uint8_t s_reconnect_addr_type = 0;

void ipc_set_reconnect_addr(const uint8_t addr[6], uint8_t addr_type) {
    uint32_t save = spin_lock_blocking(s_lock);
    for (int i = 0; i < 6; i++) s_reconnect_addr[i] = addr[i];
    s_reconnect_addr_type = addr_type;
    spin_unlock(s_lock, save);
}

void ipc_get_reconnect_addr(uint8_t addr_out[6], uint8_t *type_out) {
    uint32_t save = spin_lock_blocking(s_lock);
    if (addr_out) {
        for (int i = 0; i < 6; i++) addr_out[i] = (uint8_t)s_reconnect_addr[i];
    }
    if (type_out) *type_out = (uint8_t)s_reconnect_addr_type;
    spin_unlock(s_lock, save);
}

// ---------------------------------------------------------------------------
// Directed ADV address slot — Core 0 writes, Core 1 reads (single-slot, last-wins)
// ---------------------------------------------------------------------------

static volatile uint8_t s_directed_adv_addr[6]   = {0};
static volatile uint8_t s_directed_adv_addr_type = 0;

void ipc_set_directed_adv_addr(const uint8_t addr[6], uint8_t addr_type) {
    uint32_t save = spin_lock_blocking(s_lock);
    for (int i = 0; i < 6; i++) s_directed_adv_addr[i] = addr[i];
    s_directed_adv_addr_type = addr_type;
    spin_unlock(s_lock, save);
}

void ipc_get_directed_adv_addr(uint8_t addr_out[6], uint8_t *type_out) {
    uint32_t save = spin_lock_blocking(s_lock);
    if (addr_out) {
        for (int i = 0; i < 6; i++) addr_out[i] = (uint8_t)s_directed_adv_addr[i];
    }
    if (type_out) *type_out = (uint8_t)s_directed_adv_addr_type;
    spin_unlock(s_lock, save);
}

// ---------------------------------------------------------------------------
// Peripheral (downstream) peer address — Core 1 writes, Core 0 reads (single-slot, last-wins)
// ---------------------------------------------------------------------------

static volatile uint8_t s_periph_addr[6]   = {0};
static volatile uint8_t s_periph_addr_type = 0;

void ipc_put_periph_addr(const uint8_t addr[6], uint8_t addr_type) {
    uint32_t save = spin_lock_blocking(s_lock);
    for (int i = 0; i < 6; i++) s_periph_addr[i] = addr[i];
    s_periph_addr_type = addr_type;
    spin_unlock(s_lock, save);
    ipc_push_to_core0(IPC_ENCODE(IPC_PERIPH_CONNECTED_ADDR, 0));
}

void ipc_get_periph_addr(uint8_t addr_out[6], uint8_t *type_out) {
    uint32_t save = spin_lock_blocking(s_lock);
    if (addr_out) {
        for (int i = 0; i < 6; i++) addr_out[i] = (uint8_t)s_periph_addr[i];
    }
    if (type_out) *type_out = (uint8_t)s_periph_addr_type;
    spin_unlock(s_lock, save);
}

// ---------------------------------------------------------------------------
// Delete-bonding address queue — Core 0 writes, Core 1 reads (SPSC ring, spinlock 14)
// ---------------------------------------------------------------------------

static volatile uint8_t  s_delbond_addr[IPC_DELBOND_SLOTS][6] = {{0}};
static volatile uint8_t  s_delbond_type[IPC_DELBOND_SLOTS]    = {0};
static volatile uint32_t s_delbond_wr = 0;  // Core 0 producer
static volatile uint32_t s_delbond_rd = 0;  // Core 1 consumer

bool ipc_put_delete_bonding_addr(const uint8_t addr[6], uint8_t addr_type) {
    uint32_t save = spin_lock_blocking(s_lock);
    uint32_t next = (s_delbond_wr + 1) % IPC_DELBOND_SLOTS;
    if (next == s_delbond_rd) {  // ring full — drop
        spin_unlock(s_lock, save);
        return false;
    }
    for (int i = 0; i < 6; i++) s_delbond_addr[s_delbond_wr][i] = addr[i];
    s_delbond_type[s_delbond_wr] = addr_type;
    s_delbond_wr = next;
    spin_unlock(s_lock, save);
    return true;
}

bool ipc_get_delete_bonding_addr(uint8_t addr_out[6], uint8_t *type_out) {
    uint32_t save = spin_lock_blocking(s_lock);
    if (s_delbond_rd == s_delbond_wr) {  // empty
        spin_unlock(s_lock, save);
        return false;
    }
    if (addr_out) {
        for (int i = 0; i < 6; i++) addr_out[i] = (uint8_t)s_delbond_addr[s_delbond_rd][i];
    }
    if (type_out) *type_out = (uint8_t)s_delbond_type[s_delbond_rd];
    s_delbond_rd = (s_delbond_rd + 1) % IPC_DELBOND_SLOTS;
    spin_unlock(s_lock, save);
    return true;
}

// ---------------------------------------------------------------------------
// HID ring buffer — Core 1 writes, Core 0 reads
// ---------------------------------------------------------------------------

bool ipc_put_hid_report(const uint8_t *data, uint8_t len) {
    if (len > IPC_HID_SLOT_SIZE) {
        len = IPC_HID_SLOT_SIZE;
    }

    uint32_t save = spin_lock_blocking(s_lock);

    uint32_t next_idx = (s_hid_write_idx + 1) % IPC_HID_SLOTS;
    if (next_idx == s_hid_read_idx) {
        // Ring buffer full — drop report
        spin_unlock(s_lock, save);
        DIAG_INC(hid_ring_drop);
        return false;
    }

    uint32_t slot = s_hid_write_idx;
    memcpy((void *)s_hid_ring[slot].data, data, len);
    s_hid_ring[slot].len   = len;
    s_hid_ring[slot].valid = true;
    s_hid_write_idx        = next_idx;

    spin_unlock(s_lock, save);

    // Notify Core 0
    ipc_push_to_core0(IPC_ENCODE(IPC_HID_REPORT, slot));
    return true;
}

void ipc_get_hid_report(uint32_t slot_idx, uint8_t *data_out, uint8_t *len_out) {
    if (slot_idx >= IPC_HID_SLOTS) {
        *len_out = 0;
        return;
    }

    uint32_t save = spin_lock_blocking(s_lock);

    uint8_t len = s_hid_ring[slot_idx].len;
    memcpy(data_out, (const void *)s_hid_ring[slot_idx].data, len);
    *len_out = len;
    s_hid_ring[slot_idx].valid = false;
    s_hid_read_idx = (slot_idx + 1) % IPC_HID_SLOTS;

    spin_unlock(s_lock, save);
}

bool ipc_pop_hid_report(uint8_t *data_out, uint8_t *len_out) {
    uint32_t save = spin_lock_blocking(s_lock);
    uint32_t idx  = s_hid_read_idx;
    if (!s_hid_ring[idx].valid) {
        spin_unlock(s_lock, save);
        return false;
    }
    uint8_t len = s_hid_ring[idx].len;
    memcpy(data_out, (const void *)s_hid_ring[idx].data, len);
    *len_out = len;
    s_hid_ring[idx].valid = false;
    s_hid_read_idx = (idx + 1) % IPC_HID_SLOTS;
    spin_unlock(s_lock, save);
    return true;
}

// ---------------------------------------------------------------------------
// BLE status slot (R3) — Core 1 publishes, Core 0 samples (level-based)
// ---------------------------------------------------------------------------

static volatile ipc_ble_status_t s_ble_status;   // .valid starts false (BSS)

void ipc_publish_ble_status(const ipc_ble_status_t *st) {
    uint32_t save = spin_lock_blocking(s_lock);
    s_ble_status.periph      = st->periph;
    s_ble_status.central     = st->central;
    s_ble_status.active_slot = st->active_slot;
    s_ble_status.split_role  = st->split_role;
    s_ble_status.peer_batt   = st->peer_batt;
    s_ble_status.mouse_batt  = st->mouse_batt;
    s_ble_status.valid       = true;
    spin_unlock(s_lock, save);
}

void ipc_get_ble_status(ipc_ble_status_t *out) {
    uint32_t save = spin_lock_blocking(s_lock);
    out->periph      = s_ble_status.periph;
    out->central     = s_ble_status.central;
    out->active_slot = s_ble_status.active_slot;
    out->split_role  = s_ble_status.split_role;
    out->peer_batt   = s_ble_status.peer_batt;
    out->mouse_batt  = s_ble_status.mouse_batt;
    out->valid       = s_ble_status.valid;
    spin_unlock(s_lock, save);
}

// ---------------------------------------------------------------------------
// Log ring buffer — Core 1 writes, Core 0 drains to UART
// Uses spinlock 15 (separate from HID/addr spinlock 14)
// ---------------------------------------------------------------------------

typedef struct {
    char          text[IPC_LOG_SLOT_SIZE];
    volatile bool ready;   // set by Core 1 AFTER the text is fully written
} ipc_log_slot_t;

static volatile ipc_log_slot_t s_log_ring[IPC_LOG_SLOTS];
static volatile uint32_t       s_log_wr      = 0;  // Core 1 writer
static volatile uint32_t       s_log_rd      = 0;  // Core 0 reader
static volatile uint32_t       s_log_dropped = 0;

void core1_log(const char *fmt, ...) {
    uint32_t save = spin_lock_blocking(s_log_lock);
    uint32_t next = (s_log_wr + 1) % IPC_LOG_SLOTS;
    if (next == s_log_rd) {
        s_log_dropped++;
        spin_unlock(s_log_lock, save);
        return;
    }
    uint32_t slot = s_log_wr;
    s_log_ring[slot].ready = false;   // not printable until the text is written
    s_log_wr = next;
    spin_unlock(s_log_lock, save);

    // vsnprintf outside spinlock — the slot is reserved (s_log_wr advanced past
    // it) but Core 0 must NOT print it yet: rd != wr alone is not enough, the
    // drain also waits for the per-slot ready flag set after the write below.
    va_list ap;
    va_start(ap, fmt);
    vsnprintf((char *)s_log_ring[slot].text, IPC_LOG_SLOT_SIZE, fmt, ap);
    va_end(ap);
    __dmb();                          // text visible before the ready flag
    s_log_ring[slot].ready = true;
}

void ipc_log_drain(void) {
    // Report dropped messages
    uint32_t save = spin_lock_blocking(s_log_lock);
    uint32_t dropped = s_log_dropped;
    s_log_dropped = 0;
    uint32_t rd = s_log_rd;
    uint32_t wr = s_log_wr;
    spin_unlock(s_log_lock, save);

    if (dropped > 0) {
        printf("[C1-LOG] %lu messaggi persi\n", (unsigned long)dropped);
    }

    // Drain all available slots. Stop at the first slot whose text is still
    // being written (ready flag clear) — it will be drained on the next pass.
    while (rd != wr) {
        if (!s_log_ring[rd].ready) break;
        printf("%s", (const char *)s_log_ring[rd].text);
        s_log_ring[rd].ready = false;

        save = spin_lock_blocking(s_log_lock);
        s_log_rd = (rd + 1) % IPC_LOG_SLOTS;
        rd = s_log_rd;
        wr = s_log_wr;
        spin_unlock(s_log_lock, save);
    }
}

// ---------------------------------------------------------------------------
// Split remote-event queue (Phase 18) — Core 0 internal, lock-free SPSC ring.
// Single producer (CM task), single consumer (keyscan task), power-of-two size →
// no lock needed (uint8_t index writes are atomic on this core). Drops on full
// (a missed event is healed by the next matrix_snapshot resync).
// ---------------------------------------------------------------------------

#define SPLIT_RX_RING_SIZE  64   // power of two
static volatile uint8_t  s_split_rx_buf[SPLIT_RX_RING_SIZE];
static volatile uint16_t s_split_rx_head = 0;   // producer (CM)
static volatile uint16_t s_split_rx_tail = 0;   // consumer (keyscan)

static volatile uint32_t s_split_rx_snapshot = 0;
static volatile bool     s_split_rx_snapshot_pending = false;

void ipc_split_push_remote_event(uint8_t evt) {
    uint16_t head = s_split_rx_head;
    uint16_t next = (uint16_t)((head + 1) & (SPLIT_RX_RING_SIZE - 1));
    if (next == s_split_rx_tail) {         // full → drop (snapshot will resync)
        DIAG_INC(split_rx_drop);
        return;
    }
    s_split_rx_buf[head] = evt;
    s_split_rx_head = next;
}

bool ipc_split_pop_remote_event(uint8_t *evt_out) {
    uint16_t tail = s_split_rx_tail;
    if (tail == s_split_rx_head) return false;   // empty
    *evt_out = s_split_rx_buf[tail];
    s_split_rx_tail = (uint16_t)((tail + 1) & (SPLIT_RX_RING_SIZE - 1));
    return true;
}

void ipc_split_set_remote_snapshot(uint32_t bitmap24) {
    s_split_rx_snapshot = bitmap24;
    s_split_rx_snapshot_pending = true;
}

// ---------------------------------------------------------------------------
// Reactive-hit rings (Phase 24b) — Core 0 internal, lock-free SPSC, power-of-two.
// TX: keyscan/action producer → CM consumer (forward to slave).
// RX: CM producer → keyscan consumer (apply on the slave). Drop on full.
// ---------------------------------------------------------------------------

#define RGB_HIT_RING_SIZE  32   // power of two

static volatile uint8_t  s_rgb_hit_tx_buf[RGB_HIT_RING_SIZE];
static volatile uint16_t s_rgb_hit_tx_head = 0;   // producer (keyscan)
static volatile uint16_t s_rgb_hit_tx_tail = 0;   // consumer (CM)

void ipc_rgb_hit_tx_push(uint8_t packed) {
    uint16_t head = s_rgb_hit_tx_head;
    uint16_t next = (uint16_t)((head + 1) & (RGB_HIT_RING_SIZE - 1));
    if (next == s_rgb_hit_tx_tail) return;   // full → drop (splash self-heals)
    s_rgb_hit_tx_buf[head] = packed;
    s_rgb_hit_tx_head = next;
}

bool ipc_rgb_hit_tx_pop(uint8_t *out) {
    uint16_t tail = s_rgb_hit_tx_tail;
    if (tail == s_rgb_hit_tx_head) return false;
    *out = s_rgb_hit_tx_buf[tail];
    s_rgb_hit_tx_tail = (uint16_t)((tail + 1) & (RGB_HIT_RING_SIZE - 1));
    return true;
}

static volatile uint8_t  s_rgb_hit_rx_buf[RGB_HIT_RING_SIZE];
static volatile uint16_t s_rgb_hit_rx_head = 0;   // producer (CM)
static volatile uint16_t s_rgb_hit_rx_tail = 0;   // consumer (keyscan)

void ipc_rgb_hit_rx_push(uint8_t packed) {
    uint16_t head = s_rgb_hit_rx_head;
    uint16_t next = (uint16_t)((head + 1) & (RGB_HIT_RING_SIZE - 1));
    if (next == s_rgb_hit_rx_tail) return;   // full → drop
    s_rgb_hit_rx_buf[head] = packed;
    s_rgb_hit_rx_head = next;
}

bool ipc_rgb_hit_rx_pop(uint8_t *out) {
    uint16_t tail = s_rgb_hit_rx_tail;
    if (tail == s_rgb_hit_rx_head) return false;
    *out = s_rgb_hit_rx_buf[tail];
    s_rgb_hit_rx_tail = (uint16_t)((tail + 1) & (RGB_HIT_RING_SIZE - 1));
    return true;
}

bool ipc_split_take_remote_snapshot(uint32_t *bitmap_out) {
    if (!s_split_rx_snapshot_pending) return false;
    *bitmap_out = s_split_rx_snapshot;
    s_split_rx_snapshot_pending = false;
    return true;
}

// Power state shares the existing report/status lock. Never hold it across BLE,
// stdio or FreeRTOS calls. Cancellation and commit have one linearization point.
static power_state_t s_power;
void ipc_power_init(uint64_t now) {
    uint32_t irq=spin_lock_blocking(s_lock);
    memset(&s_power,0,sizeof(s_power)); s_power.boot_us=now;
    spin_unlock(s_lock,irq);
}
void ipc_power_local(const power_local_t *local) {
    uint32_t irq=spin_lock_blocking(s_lock); s_power.local=*local;
    spin_unlock(s_lock,irq);
}
void ipc_power_radio(const power_radio_t *radio) {
    uint32_t irq=spin_lock_blocking(s_lock); s_power.radio=*radio;
    spin_unlock(s_lock,irq);
}
void ipc_power_get(power_state_t *out) {
    uint32_t irq=spin_lock_blocking(s_lock); *out=s_power;
    spin_unlock(s_lock,irq);
}
uint16_t ipc_power_request(uint64_t now, uint32_t deep_ms) {
    uint32_t irq=spin_lock_blocking(s_lock);
    uint16_t token=power_request(&s_power,now,deep_ms);
    spin_unlock(s_lock,irq); return token;
}
void ipc_power_cancel(void) {
    uint32_t irq=spin_lock_blocking(s_lock); power_cancel(&s_power);
    spin_unlock(s_lock,irq);
}
bool ipc_power_commit(uint16_t token, uint64_t now, uint32_t deep_ms) {
    uint32_t irq=spin_lock_blocking(s_lock);
    bool ok=power_commit(&s_power,token,now,deep_ms);
    spin_unlock(s_lock,irq); return ok;
}
void ipc_power_off(void) {
    uint32_t irq=spin_lock_blocking(s_lock);
    if (s_power.phase==POWER_COMMITTED) s_power.phase=POWER_OFF;
    spin_unlock(s_lock,irq);
}

void ipc_power_usb(bool active) {
    uint32_t irq=spin_lock_blocking(s_lock); s_power.local.usb=active;
    if (active) power_cancel(&s_power);
    spin_unlock(s_lock,irq);
}

#if DEBUG_MOUSE_TRACE
static uint32_t s_trace[IPC_TRACE_CAPACITY][7];
static uint32_t s_trace_end;
void ipc_trace_put(const uint32_t words[7]) {
    uint32_t irq=spin_lock_blocking(s_lock);
    memcpy(s_trace[s_trace_end % IPC_TRACE_CAPACITY],words,7*sizeof(uint32_t));
    ++s_trace_end;
    spin_unlock(s_lock,irq);
}
uint32_t ipc_trace_end(void) {
    uint32_t irq=spin_lock_blocking(s_lock);
    uint32_t end=s_trace_end;
    spin_unlock(s_lock,irq); return end;
}
bool ipc_trace_get(uint32_t sequence,uint32_t words[7]) {
    uint32_t irq=spin_lock_blocking(s_lock);
    uint32_t age=s_trace_end-sequence;
    bool valid=age && age<=IPC_TRACE_CAPACITY && sequence<s_trace_end;
    if(valid) memcpy(words,s_trace[sequence % IPC_TRACE_CAPACITY],7*sizeof(uint32_t));
    spin_unlock(s_lock,irq); return valid;
}
#endif // DEBUG_MOUSE_TRACE
