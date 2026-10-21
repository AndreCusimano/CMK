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
// ble_relay.c — merge/relay/output to the downstream host (Core 1 only, R7 split)
//
// Keyboard merge (upstream + local → 6KRO), mouse relay (decoded via
// g_ble_upstream_map), programmable buttons, the HIDS CAN_SEND_NOW priority
// chain, IPC report slot/ring draining, and USB forwarding (Phase 14 ext).
//
// Relay (Core 1 internal, zero latency):
//   upstream HID report → s_relay_pending → CAN_SEND_NOW → downstream peripheral
//

#include "ble_manager.h"
#include "ble_internal.h"
#include "ble_link_policy.h"
#include "ble_power.h"
#include "ipc/ipc_core.h"
#include "diag.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"
#include "hid/hid_parser.h"

#ifdef USB_ENABLE
#include "usb/usb_hid.h"
#endif
#include "output_select.h"

#include "btstack.h"
#include "ble/gatt-service/hids_device.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include <string.h>

// ---------------------------------------------------------------------------
// Module-private state
// ---------------------------------------------------------------------------

static uint8_t s_protocol_mode = 1;

// Relay buffer — raw fallback only (unknown upstream report IDs). Decoded
// mouse reports go through the coalescing queue below instead.
static uint8_t s_relay_report[IPC_HID_SLOT_SIZE];
static uint8_t s_relay_len       = 0;
static uint8_t s_relay_report_id = HID_REPORT_ID_KEYBOARD;
static bool    s_relay_pending   = false;

// Mouse relay SEND queue — coalescing FIFO of decoded upstream mouse reports.
// The BLE link ships ~1 report per connection interval; a 125 Hz mouse arrives
// faster. The old single-slot last-wins relay DISCARDED every overwritten
// report — deltas lost (slow, choppy cursor) and fast button transitions
// collapsed. Here motion with unchanged buttons is summed (saturating) into
// the newest entry, while a button change always starts a new entry, so no
// click is ever collapsed and no movement is ever lost.
#define RELAY_MOUSE_SLOTS 8
static hid_mouse_report_t s_relay_mouse_q[RELAY_MOUSE_SLOTS];
static uint8_t s_relay_mouse_rd    = 0;
static uint8_t s_relay_mouse_count = 0;
// Diagnostics only. No per-report logging or extra HCI commands.
static uint64_t s_mouse_queued_at[RELAY_MOUSE_SLOTS];
static uint32_t s_mouse_rx, s_mouse_tx, s_mouse_errors, s_mouse_wait_max;
static uint32_t s_mouse_depth_max, s_mouse_last_status;
static uint64_t s_mouse_window_at;
static uint32_t s_tx_requests,s_tx_callbacks,s_tx_duplicates,s_tx_errors,s_tx_wait_max;
static uint32_t s_tx_polls,s_tx_no_slots,s_tx_reserved,s_tx_sendable,s_tx_last_error;
static uint32_t s_tx_completed_us,s_tx_completed_ds,s_tx_inflight_us,s_tx_inflight_ds,s_tx_poll_gap;
static uint64_t s_tx_requested_at,s_tx_last_poll;
static bool s_tx_waiting;
// Shape of the controller's credit return on the DS link. The packet total
// (s_tx_completed_ds) cannot tell a link that transmits steadily from one whose
// credits come back in bursts: both yield the same packets/s. These separate
// them — how MANY Number-Of-Completed-Packets events arrive, how many packets
// each one retires, and how far apart they are in connection intervals.
static uint32_t s_nocp_events,s_nocp_max_batch,s_nocp_gap[5];
static uint64_t s_nocp_last_at;
// In-flight depth profile on the DS link (BTstack's own num_packets_sent, i.e.
// packets handed to the controller and not yet completed), sampled every poll —
// a few samples per connection interval. The per-window MAX alone cannot tell a
// queue permanently pinned at 8 from one that fills and drains in bursts, so it
// cannot say whether the stall arrives in one step or builds up. This does.
// Buckets: 0, 1-2, 3-5, 6-7, 8.
static uint32_t s_depth_hist[5],s_depth_samples,s_depth_min;
// Right-side latency of each relayed mouse report, split at the controller
// boundary: queue (first coalesced contribution → handed to HCI) and air
// (handed to HCI → controller reports it completed). ACL packets of one
// connection complete in FIFO order, so a report is done once the link's
// cumulative completed count reaches the position it took at send time.
#define MOUSE_FLIGHT_SLOTS 16
typedef struct { uint32_t position; uint64_t queued_at,sent_at; } mouse_flight_t;
static mouse_flight_t s_flight[MOUSE_FLIGHT_SLOTS];
static uint8_t s_flight_rd,s_flight_count;
static uint32_t s_ds_completed_total;   // monotonic per DS connection
static uint32_t s_lat_count,s_lat_air_max,s_lat_total_max,s_mouse_capped;
static uint64_t s_lat_air_sum,s_lat_total_sum;
// Phase of each DS credit return inside the upstream mouse's connection grid,
// measured from the latest mouse report (one per US connection event while the
// mouse moves). Both links run on this controller: if the DS anchor has latched
// onto a fixed position of the US grid and the scheduler drops the DS event
// there, the lag windows concentrate in one band of bins that drifts slowly.
// 8 bins over one US interval.
static uint64_t s_us_rx_at;
static uint32_t s_phase_bins[8],s_phase_samples;

static void relay_request_send(void);

// Backpressure: may another mouse report enter the controller? See
// RELAY_MOUSE_INFLIGHT_MAX / RELAY_MOUSE_SLOW_MS in cmk_config.h.
static bool relay_mouse_can_ship(void) {
#if RELAY_MOUSE_INFLIGHT_MAX > 0
    hci_connection_t *ds=hci_connection_for_handle(g_ble_periph_handle);
    if(!ds || ds->num_packets_sent<RELAY_MOUSE_INFLIGHT_MAX) return true;
#if RELAY_MOUSE_SLOW_MS > 0
    // Link keeping up (oldest tracked mouse report still young) → no cap.
    // Turning false→true only happens as packets complete, and the DS credit
    // return re-arms the send, so no timer is needed.
    if(!s_flight_count) return true;
    return time_us_64()-s_flight[s_flight_rd].sent_at < RELAY_MOUSE_SLOW_MS*1000ull;
#else
    return false;
#endif
#else
    return true;
#endif
}
static void mouse_flight_track(uint64_t queued_at) {
    hci_connection_t *ds=hci_connection_for_handle(g_ble_periph_handle);
    if(!ds || s_flight_count==MOUSE_FLIGHT_SLOTS) return;
    mouse_flight_t *f=&s_flight[(s_flight_rd+s_flight_count)%MOUSE_FLIGHT_SLOTS];
    // hci_send_acl_packet_fragments() already counted this packet in flight.
    f->position=s_ds_completed_total+ds->num_packets_sent;
    f->queued_at=queued_at; f->sent_at=time_us_64();
    ++s_flight_count;
}
static void mouse_flight_complete(uint64_t now) {
    while(s_flight_count && s_flight[s_flight_rd].position<=s_ds_completed_total) {
        const mouse_flight_t *f=&s_flight[s_flight_rd];
        uint32_t air=(uint32_t)(now-f->sent_at), total=(uint32_t)(now-f->queued_at);
        if(air>s_lat_air_max) s_lat_air_max=air;
        if(total>s_lat_total_max) s_lat_total_max=total;
        s_lat_air_sum+=air; s_lat_total_sum+=total; ++s_lat_count;
        s_flight_rd=(uint8_t)((s_flight_rd+1u)%MOUSE_FLIGHT_SLOTS); --s_flight_count;
    }
}
static void relay_trace_callback(void) {
    ++s_tx_callbacks;
    if(s_tx_waiting) {
        uint32_t elapsed=(uint32_t)(time_us_64()-s_tx_requested_at);
        if(elapsed>s_tx_wait_max) s_tx_wait_max=elapsed;
        s_tx_waiting=false;
    }
}
static void relay_request_send(void) {
    ++s_tx_requests;
    bool first=!s_tx_waiting;
    if(first) { s_tx_requested_at=time_us_64(); s_tx_waiting=true; }
    // BTstack may invoke the callback synchronously. Set timing BEFORE calling.
    uint8_t rc=hids_device_request_can_send_now_event(g_ble_periph_handle);
    if(rc==ERROR_CODE_COMMAND_DISALLOWED) ++s_tx_duplicates;
    else if(rc!=ERROR_CODE_SUCCESS) {
        ++s_tx_errors; s_tx_last_error=rc;
        if(first) s_tx_waiting=false;
    }
}
void ble_relay_trace_completed(hci_con_handle_t handle,uint16_t packets) {
    if(handle==g_ble_central_handle) s_tx_completed_us+=packets;
    if(handle!=g_ble_periph_handle) return;
    s_tx_completed_ds+=packets;
    ++s_nocp_events;
    if(packets>s_nocp_max_batch) s_nocp_max_batch=packets;
    uint64_t now=time_us_64();
    s_ds_completed_total+=packets;
    mouse_flight_complete(now);
    uint32_t us=ble_manager_us_interval_us();
    if(us && s_us_rx_at && now-s_us_rx_at<4ull*us) {
        ++s_phase_bins[(uint32_t)((now-s_us_rx_at)%us)*8u/us];
        ++s_phase_samples;
    }
    // A mouse report held back by the in-flight cap ships on this credit.
    if(s_relay_mouse_count && g_ble_periph_subscribed && relay_mouse_can_ship())
        relay_request_send();
    uint32_t interval=ble_manager_ds_interval_us();
    // Bucket the gap since the previous DS credit return, rounded to the nearest
    // connection interval: 1,2,3,4,5+. Skip the first sample and any window in
    // which the interval is unknown (link down) — no bucket can be attributed.
    if(s_nocp_last_at && interval) {
        uint32_t n=(uint32_t)((now-s_nocp_last_at+interval/2)/interval);
        if(n<1) n=1;
        if(n>5) n=5;
        ++s_nocp_gap[n-1];
    }
    s_nocp_last_at=now;
}

#if DEBUG_MOUSE_TRACE
// 2 s relay/link sample for the RAM recorder (compiled out with it).
static void ble_relay_trace_poll(void) {
    uint64_t now=time_us_64();
    if(s_tx_last_poll && now-s_tx_last_poll>s_tx_poll_gap) s_tx_poll_gap=(uint32_t)(now-s_tx_last_poll);
    s_tx_last_poll=now;
    hci_connection_t *us=hci_connection_for_handle(g_ble_central_handle);
    hci_connection_t *ds=hci_connection_for_handle(g_ble_periph_handle);
    if(us && us->num_packets_sent>s_tx_inflight_us) s_tx_inflight_us=us->num_packets_sent;
    if(ds) {
        uint32_t d=ds->num_packets_sent;
        if(d>s_tx_inflight_ds) s_tx_inflight_ds=d;
        if(!s_depth_samples || d<s_depth_min) s_depth_min=d;
        ++s_depth_samples;
        ++s_depth_hist[d==0?0:d<=2?1:d<=5?2:d<=7?3:4];
    }
    if(s_relay_mouse_count && ds) {
        ++s_tx_polls;
        if(hci_number_free_acl_slots_for_handle(g_ble_periph_handle)<=0) ++s_tx_no_slots;
        if(hci_is_packet_buffer_reserved()) ++s_tx_reserved;
        if(hci_can_send_acl_packet_now(g_ble_periph_handle)) ++s_tx_sendable;
    }
    if (!s_mouse_window_at) { s_mouse_window_at=now; return; }
    if (now-s_mouse_window_at<2000000) return;
    uint32_t age=s_relay_mouse_count ? (uint32_t)((now-s_mouse_queued_at[s_relay_mouse_rd])/1000) : 0;
    uint32_t w[7]={1,(uint32_t)(now/1000),s_mouse_rx,s_mouse_tx,s_mouse_errors,
                  s_mouse_wait_max,(s_mouse_depth_max<<24)|(s_relay_mouse_count<<16)|(s_mouse_last_status<<8)};
    ipc_trace_put(w);
    uint32_t extra[7]={2,(uint32_t)(now/1000),age,(uint32_t)((now-s_mouse_window_at)/1000),
        g_ble_central_handle,g_ble_periph_handle,output_ble_enabled()};
    ipc_trace_put(extra);
    ble_manager_trace_links();
    uint32_t requests[7]={8,(uint32_t)(now/1000),s_tx_requests,s_tx_callbacks,s_tx_duplicates,s_tx_errors,s_tx_wait_max};
    uint32_t blocked[7]={9,(uint32_t)(now/1000),s_tx_waiting?(uint32_t)(now-s_tx_requested_at):0,
                        s_tx_polls,s_tx_no_slots,s_tx_reserved,s_tx_sendable};
    uint32_t credits[7]={10,(uint32_t)(now/1000),s_tx_completed_us,s_tx_completed_ds,
                        (s_tx_inflight_us<<16)|s_tx_inflight_ds,s_tx_poll_gap,s_tx_last_error};
    uint32_t g3=s_nocp_gap[2]>255?255:s_nocp_gap[2];
    uint32_t g4=s_nocp_gap[3]>255?255:s_nocp_gap[3];
    uint32_t g5=s_nocp_gap[4]>255?255:s_nocp_gap[4];
    uint32_t nocp[7]={13,(uint32_t)(now/1000),s_nocp_events,s_nocp_max_batch,
                     (s_nocp_gap[0]&0xffffu)|(s_nocp_gap[1]<<16),
                     g3|(g4<<8)|(g5<<16),ble_manager_ds_interval_us()};
    uint32_t depth[7]={14,(uint32_t)(now/1000),
                      (s_depth_hist[0]&0xffffu)|(s_depth_hist[1]<<16),
                      (s_depth_hist[2]&0xffffu)|(s_depth_hist[3]<<16),
                      s_depth_hist[4],
                      ((s_depth_samples?s_depth_min:0)<<8)|(s_tx_inflight_ds&0xffu),
                      s_depth_samples};
    uint32_t bins[2]={0,0};
    for(unsigned i=0;i<8;++i) bins[i/4]|=(s_phase_bins[i]>255?255:s_phase_bins[i])<<(8*(i%4));
    uint32_t phase[7]={15,(uint32_t)(now/1000),bins[0],bins[1],s_phase_samples,
                      ble_manager_us_interval_us(),ble_manager_ds_interval_us()};
    uint32_t latency[7]={16,(uint32_t)(now/1000),s_lat_air_max,
                        s_lat_count?(uint32_t)(s_lat_air_sum/s_lat_count):0,s_lat_total_max,
                        s_lat_count?(uint32_t)(s_lat_total_sum/s_lat_count):0,
                        ((s_lat_count>0xffffu?0xffffu:s_lat_count)<<16)|(s_mouse_capped>0xffffu?0xffffu:s_mouse_capped)};
    ipc_trace_put(requests); ipc_trace_put(blocked); ipc_trace_put(credits);
    ipc_trace_put(nocp); ipc_trace_put(depth); ipc_trace_put(phase); ipc_trace_put(latency);
    for(unsigned i=0;i<8;++i) s_phase_bins[i]=0;
    s_phase_samples=0;
    s_lat_count=s_lat_air_max=s_lat_total_max=s_mouse_capped=0;
    s_lat_air_sum=s_lat_total_sum=0;
    s_nocp_events=s_nocp_max_batch=0;
    for(unsigned i=0;i<5;++i) s_nocp_gap[i]=0;
    s_depth_samples=0;
    for(unsigned i=0;i<5;++i) s_depth_hist[i]=0;
    s_tx_requests=s_tx_callbacks=s_tx_duplicates=s_tx_errors=s_tx_wait_max=0;
    s_tx_polls=s_tx_no_slots=s_tx_reserved=s_tx_sendable=0;
    s_tx_completed_us=s_tx_completed_ds=s_tx_inflight_us=s_tx_inflight_ds=s_tx_poll_gap=s_tx_last_error=0;
    s_mouse_rx=s_mouse_tx=s_mouse_errors=s_mouse_wait_max=0;
    s_mouse_depth_max=s_relay_mouse_count; s_mouse_last_status=0; s_mouse_window_at=now;
}
#endif // DEBUG_MOUSE_TRACE


// HID report log buffer — decouples HID processing from UART output.
// Capture writes here (overwriting old), timer prints periodically.
// Only active when DEBUG_UPSTREAM_K or DEBUG_UPSTREAM_M is enabled.
#define LOG_HID_INTERVAL_MS  100
#if DEBUG_UPSTREAM_K || DEBUG_UPSTREAM_M
static uint8_t  s_log_report[IPC_HID_SLOT_SIZE];
static uint8_t  s_log_len        = 0;
static uint8_t  s_log_report_id  = 0;   // report ID of last captured upstream report
static uint32_t s_log_count      = 0;   // reports arrived since last print
static uint32_t s_log_total      = 0;   // running total
static bool     s_log_has_data   = false;
#endif
static btstack_timer_source_t s_log_timer;

// ---------------------------------------------------------------------------
// Keyboard merge state — upstream + local combined into one HID report
//
// Two source buffers (upstream and local) are kept separate so that when
// upstream disconnects, its contribution is zeroed and the merged report
// is immediately rebuilt with only local keys.
//
// s_upstream_slot_mask: bitmask (bits 0-5) tracking which of the 6 keycode
// slots in the MERGED output were filled by upstream. Updated every build.
// ---------------------------------------------------------------------------
static uint8_t s_upstream_kbd[8]    = {0}; // last upstream keyboard [mod,0,kc0..kc5]
static uint8_t s_local_kbd[8]       = {0}; // last local keyboard    [mod,0,kc0..kc5]
static uint8_t s_kbd_merged[8]      = {0}; // last merged report sent downstream
static uint8_t s_upstream_slot_mask = 0;   // bits 0-5: merged slots filled by upstream
static bool    s_kbd_pending        = false;  // mirror of "send ring non-empty"

// Keyboard SEND ring (Core 1 single-thread → no lock). The BLE link ships one report
// per CAN_SEND_NOW (one connection interval); fast typing arrives faster, so we queue
// each distinct merged state and drain one per CAN_SEND_NOW instead of last-wins
// collapsing them (which dropped fast keystrokes). Fed by local (IPC) + upstream relay.
#define KBD_SEND_SLOTS 16
static uint8_t s_kbd_send_ring[KBD_SEND_SLOTS][8];
static uint8_t s_kbd_send_wr = 0;
static uint8_t s_kbd_send_rd = 0;
static uint8_t s_kbd_last_enq[8] = {0};   // dedup consecutive identical merged states

// Local mouse report from Core 0 Mouse Keys (Phase 9)
static uint8_t s_local_mouse[7]    = {0};
static bool    s_local_mouse_pending = false;

// Programmable Button report from Core 0 (Phase 12)
static uint8_t s_local_pb[4]       = {0};
static bool    s_local_pb_pending  = false;

// Consumer Control report from Core 0 (bug #6 — media/volume keys)
static uint8_t s_local_consumer[2]      = {0};
static bool    s_local_consumer_pending = false;

static void hids_device_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz);
static void log_hid_timer_handler(btstack_timer_source_t *ts);

// ---------------------------------------------------------------------------
// Init — HIDS device packet handler + HID log timer
// ---------------------------------------------------------------------------

void ble_relay_init(void) {
    hids_device_register_packet_handler(hids_device_handler);

    // Start HID log timer — prints buffered reports every LOG_HID_INTERVAL_MS
    s_log_timer.process = &log_hid_timer_handler;
    btstack_run_loop_set_timer(&s_log_timer, LOG_HID_INTERVAL_MS);
    btstack_run_loop_add_timer(&s_log_timer);
}

// ---------------------------------------------------------------------------
// Keyboard merge
// ---------------------------------------------------------------------------

// Build the merged keyboard report from upstream + local sources and request
// a CAN_SEND_NOW event to flush it downstream.
//
// Merge rules:
//   modifier (byte 0) : OR of upstream and local modifier bytes
//   keycodes (bytes 2-7): upstream keycodes first, then local (6KRO total)
//
// s_upstream_slot_mask is updated to reflect which output slots came from
// upstream, so the caller can audit the contribution after each build.
// Build s_kbd_merged from upstream + local sources (no send/queue side effects).
static void kbd_build_merged(void) {
    memset(s_kbd_merged, 0, sizeof(s_kbd_merged));
    s_upstream_slot_mask = 0;

    // Merge modifier bits from both sources
    s_kbd_merged[0] = s_upstream_kbd[0] | s_local_kbd[0];

    // Fill keycode slots: upstream first, then local (6KRO)
    uint8_t slot = 2;
    for (int i = 2; i < 8 && slot < 8; i++) {
        if (s_upstream_kbd[i]) {
            s_upstream_slot_mask |= (uint8_t)(1u << (slot - 2));
            s_kbd_merged[slot++] = s_upstream_kbd[i];
        }
    }
    for (int i = 2; i < 8 && slot < 8; i++) {
        if (s_local_kbd[i]) {
            s_kbd_merged[slot++] = s_local_kbd[i];
        }
    }
}

// Build the current merged state and enqueue it on the SEND ring (deduping consecutive
// identical states), then ask BTstack to send. Used by both the local-report path and
// the upstream-relay path so their reports stay ordered and none are collapsed.
static void kbd_merge_and_request(void) {
    kbd_build_merged();

    if (memcmp(s_kbd_merged, s_kbd_last_enq, 8) != 0) {
        uint8_t next = (uint8_t)((s_kbd_send_wr + 1) % KBD_SEND_SLOTS);
        if (next == s_kbd_send_rd) {
            // Full — drop the OLDEST so the latest key state always survives.
            s_kbd_send_rd = (uint8_t)((s_kbd_send_rd + 1) % KBD_SEND_SLOTS);
        }
        memcpy(s_kbd_send_ring[s_kbd_send_wr], s_kbd_merged, 8);
        s_kbd_send_wr = next;
        memcpy(s_kbd_last_enq, s_kbd_merged, 8);
    }

    s_kbd_pending = (s_kbd_send_wr != s_kbd_send_rd);
    if (s_kbd_pending && g_ble_periph_handle != HCI_CON_HANDLE_INVALID && g_ble_periph_subscribed) {
        relay_request_send();
    }
}

// ---------------------------------------------------------------------------
// Mouse relay queue
// ---------------------------------------------------------------------------

static int16_t clamp_i16(int32_t v) {
    return (int16_t)(v > INT16_MAX ? INT16_MAX : (v < INT16_MIN ? INT16_MIN : v));
}
static int8_t clamp_i8(int32_t v) {
    return (int8_t)(v > INT8_MAX ? INT8_MAX : (v < INT8_MIN ? INT8_MIN : v));
}

// Sum src's deltas into dst (saturating); dst keeps its own button state.
static void relay_mouse_fold_into(hid_mouse_report_t *dst, const hid_mouse_report_t *src) {
    dst->x     = clamp_i16((int32_t)dst->x     + src->x);
    dst->y     = clamp_i16((int32_t)dst->y     + src->y);
    dst->wheel = clamp_i8 ((int32_t)dst->wheel + src->wheel);
    dst->pan   = clamp_i8 ((int32_t)dst->pan   + src->pan);
}

static void relay_mouse_enqueue(const hid_mouse_report_t *m) {
    if (s_relay_mouse_count > 0) {
        hid_mouse_report_t *tail =
            &s_relay_mouse_q[(s_relay_mouse_rd + s_relay_mouse_count - 1u) % RELAY_MOUSE_SLOTS];
        if (tail->buttons == m->buttons) {
            relay_mouse_fold_into(tail, m);   // same button state → coalesce motion
            return;
        }
    }
    if (s_relay_mouse_count == RELAY_MOUSE_SLOTS) {
        // Full — fold the OLDEST into its successor: its motion survives, only
        // its (already superseded) button state is dropped.
        hid_mouse_report_t *oldest = &s_relay_mouse_q[s_relay_mouse_rd];
        hid_mouse_report_t *next   = &s_relay_mouse_q[(s_relay_mouse_rd + 1u) % RELAY_MOUSE_SLOTS];
        relay_mouse_fold_into(next, oldest);
        s_mouse_queued_at[(s_relay_mouse_rd + 1u) % RELAY_MOUSE_SLOTS] = s_mouse_queued_at[s_relay_mouse_rd];
        s_relay_mouse_rd = (uint8_t)((s_relay_mouse_rd + 1u) % RELAY_MOUSE_SLOTS);
        s_relay_mouse_count--;
        DIAG_INC(relay_mouse_fold);
    }
    s_mouse_queued_at[(s_relay_mouse_rd + s_relay_mouse_count) % RELAY_MOUSE_SLOTS] = time_us_64();
    s_relay_mouse_q[(s_relay_mouse_rd + s_relay_mouse_count) % RELAY_MOUSE_SLOTS] = *m;
    s_relay_mouse_count++;
    if(s_relay_mouse_count>s_mouse_depth_max) s_mouse_depth_max=s_relay_mouse_count;
}

#ifdef USB_ENABLE
// Forward an upstream HID report to Core 0 for USB output (Phase 14 ext).
// The report is tagged with its Report ID as byte[0]; Core 0 strips it and
// calls the matching usb_hid_send_*(). Used only while output == USB.
static void forward_upstream_to_usb(uint8_t report_id, const uint8_t *data, uint16_t len) {
    uint8_t buf[1 + 8];
    if (len > 8) len = 8;
    buf[0] = report_id;
    memcpy(buf + 1, data, len);
    ipc_put_hid_report(buf, (uint8_t)(1 + len));
}
#endif

// ---------------------------------------------------------------------------
// Helpers — send a report to the downstream peripheral
// ---------------------------------------------------------------------------

// Send a report using its Report ID.
// Boot protocol only supports keyboard; mouse reports are skipped in boot mode.
// NOTE: hids_device_send_input_report_for_id -> att_server_notify handles CCC check
//       internally, so it is safe to call even if the specific report is not subscribed.
static uint8_t send_report_to_downstream_for_id(uint8_t report_id,
                                                 const uint8_t *data, uint8_t len) {
    uint8_t status;
    if (s_protocol_mode == 0) {
        // Boot protocol: only keyboard supported
        if (report_id == HID_REPORT_ID_KEYBOARD) {
            status = hids_device_send_boot_keyboard_input_report(g_ble_periph_handle, data, len);
        } else {
            return ERROR_CODE_COMMAND_DISALLOWED;
        }
    } else {
        status = hids_device_send_input_report_for_id(g_ble_periph_handle, report_id, data, len);
    }
    if(report_id==HID_REPORT_ID_MOUSE) {
        if(status==ERROR_CODE_SUCCESS) ++s_mouse_tx;
        else { ++s_mouse_errors; s_mouse_last_status=status; }
    }
    return status;
}

// ---------------------------------------------------------------------------
// HID log timer — prints buffered report data periodically
// ---------------------------------------------------------------------------

static void log_hid_timer_handler(btstack_timer_source_t *ts) {
#if DEBUG_UPSTREAM_K || DEBUG_UPSTREAM_M
    if (s_log_has_data) {
        bool is_kbd = (s_log_report_id == HID_REPORT_ID_KEYBOARD);
#if DEBUG_UPSTREAM_K
        if (is_kbd) {
            core1_log("[UP-K] #%lu (%u B, %lu new): mod=%02x keys=%02x %02x %02x %02x %02x %02x\n",
                      s_log_total, s_log_len, s_log_count,
                      s_log_report[0],
                      s_log_report[2], s_log_report[3], s_log_report[4],
                      s_log_report[5], s_log_report[6], s_log_report[7]);
        }
#endif
#if DEBUG_UPSTREAM_M
        if (!is_kbd) {
            core1_log("[UP-M] #%lu (%u B, %lu new): %02x %02x %02x %02x %02x %02x %02x %02x\n",
                      (unsigned long)s_log_total, s_log_len, (unsigned long)s_log_count,
                      s_log_report[0], s_log_len > 1 ? s_log_report[1] : 0,
                      s_log_len > 2 ? s_log_report[2] : 0, s_log_len > 3 ? s_log_report[3] : 0,
                      s_log_len > 4 ? s_log_report[4] : 0, s_log_len > 5 ? s_log_report[5] : 0,
                      s_log_len > 6 ? s_log_report[6] : 0, s_log_len > 7 ? s_log_report[7] : 0);
        }
#endif
        s_log_has_data = false;
        s_log_count    = 0;
    }
#else
    (void)ts;
#endif
    btstack_run_loop_set_timer(ts, LOG_HID_INTERVAL_MS);
    btstack_run_loop_add_timer(ts);
}

// ---------------------------------------------------------------------------
// HIDS Device handler (peripheral — downstream)
// ---------------------------------------------------------------------------

static void hids_device_handler(uint8_t ptype, uint16_t ch, uint8_t *pkt, uint16_t sz) {
    UNUSED(ch); UNUSED(sz);
    if (ptype != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(pkt) != HCI_EVENT_HIDS_META) return;

    switch (hci_event_hids_meta_get_subevent_code(pkt)) {

    case HIDS_SUBEVENT_INPUT_REPORT_ENABLE: {
        g_ble_periph_handle = hids_subevent_input_report_enable_get_con_handle(pkt);
        uint8_t rep_id      = hids_subevent_input_report_enable_get_report_id(pkt);
        bool    enabled     = (hids_subevent_input_report_enable_get_enable(pkt) != 0);
        // OR-latch: set on any subscribe; only cleared on disconnect
        if (enabled) {
            g_ble_periph_subscribed = true;
            // DS subscribed — if US reconnect was deferred, trigger it now
            ble_central_resume_deferred_reconnect();
            // Ask the host for BLE_LINK_INTERVAL (slave → L2CAP request ladder
            // with fallbacks, once per connection — ble_link_policy.c).
            ble_link_policy_on_ds_subscribed(g_ble_periph_handle);
        }
        C1_DBG_STATUS("[PERIPH] Report ID %u subscribed: %s\n", rep_id, enabled ? "YES" : "NO");
#ifdef PIN_LED_CONN
        gpio_put(PIN_LED_CONN, g_ble_periph_subscribed ? 1 : 0);
#endif
        break;
    }

    case HIDS_SUBEVENT_BOOT_KEYBOARD_INPUT_REPORT_ENABLE:
        g_ble_periph_handle     = hids_subevent_boot_keyboard_input_report_enable_get_con_handle(pkt);
        g_ble_periph_subscribed = (hids_subevent_boot_keyboard_input_report_enable_get_enable(pkt) != 0);
        C1_DBG_STATUS("[PERIPH] Downstream subscribed (boot): %s\n", g_ble_periph_subscribed ? "YES" : "NO");
#ifdef PIN_LED_CONN
        gpio_put(PIN_LED_CONN, g_ble_periph_subscribed ? 1 : 0);
#endif
        break;

    case HIDS_SUBEVENT_PROTOCOL_MODE:
        s_protocol_mode = hids_subevent_protocol_mode_get_protocol_mode(pkt);
        C1_DBG_STATUS("[PERIPH] Protocol mode: %s\n", s_protocol_mode ? "Report" : "Boot");
        break;

    case HIDS_SUBEVENT_CAN_SEND_NOW:
        relay_trace_callback();
        if (s_relay_mouse_count > 0 && relay_mouse_can_ship()) {
            // 1st priority: mouse relay from upstream (decoded, normalised).
            // Pop ONE queued state (FIFO) and re-arm while entries remain —
            // button transitions are distinct entries, motion is pre-coalesced.
            // Past the in-flight cap the entry stays queued (and keeps
            // coalescing); the DS credit return re-arms the request.
            uint8_t buf[7];
            uint64_t queued_at=s_mouse_queued_at[s_relay_mouse_rd];
            uint32_t wait=(uint32_t)((time_us_64()-queued_at)/1000);
            if(wait>s_mouse_wait_max) s_mouse_wait_max=wait;
            hid_pack_mouse(&s_relay_mouse_q[s_relay_mouse_rd], buf);
            s_relay_mouse_rd = (uint8_t)((s_relay_mouse_rd + 1u) % RELAY_MOUSE_SLOTS);
            s_relay_mouse_count--;
            if (send_report_to_downstream_for_id(HID_REPORT_ID_MOUSE, buf, sizeof(buf)) == ERROR_CODE_SUCCESS)
                mouse_flight_track(queued_at);
            C1_DBG_DN_M("[DN-M] sent: %02x %02x %02x %02x %02x %02x %02x (q=%u)\n",
                     buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
                     s_relay_mouse_count);
            if ((s_relay_mouse_count && relay_mouse_can_ship()) || s_relay_pending ||
                s_local_mouse_pending || s_kbd_pending || s_local_pb_pending ||
                s_local_consumer_pending) {
                relay_request_send();
            }
        } else if (s_relay_pending) {
            // Raw relay fallback (unknown upstream report IDs)
            s_relay_pending = false;
            send_report_to_downstream_for_id(s_relay_report_id, s_relay_report, s_relay_len);
            C1_DBG_DN_M("[DN-M] sent %u B: %02x %02x %02x %02x %02x %02x %02x\n",
                     s_relay_len,
                     s_relay_report[0], s_relay_report[1], s_relay_report[2],
                     s_relay_report[3], s_relay_report[4], s_relay_report[5],
                     s_relay_len > 6 ? s_relay_report[6] : 0x00);
            if (s_local_mouse_pending || s_kbd_pending || s_local_pb_pending ||
                s_local_consumer_pending) {
                relay_request_send();
            }
        } else if (s_local_mouse_pending) {
            // 2nd priority: local mouse from Core 0 Mouse Keys (Phase 9)
            s_local_mouse_pending = false;
            send_report_to_downstream_for_id(HID_REPORT_ID_MOUSE,
                                              s_local_mouse, sizeof(s_local_mouse));
            C1_DBG_DN_M("[DN-M] local mouse: btn=%02x x=%d y=%d wh=%d pan=%d\n",
                     s_local_mouse[0],
                     (int16_t)(s_local_mouse[1] | (s_local_mouse[2] << 8)),
                     (int16_t)(s_local_mouse[3] | (s_local_mouse[4] << 8)),
                     (int8_t)s_local_mouse[5], (int8_t)s_local_mouse[6]);
            if (s_kbd_pending || s_local_pb_pending || s_local_consumer_pending) {
                relay_request_send();
            }
        } else if (s_kbd_pending) {
            // 3rd priority: merged keyboard — pop ONE queued state (FIFO, not last-wins)
            // so fast typing isn't collapsed. Re-arm while the ring still has entries.
            uint8_t out[8];
            memcpy(out, s_kbd_send_ring[s_kbd_send_rd], 8);
            s_kbd_send_rd = (uint8_t)((s_kbd_send_rd + 1) % KBD_SEND_SLOTS);
            s_kbd_pending = (s_kbd_send_rd != s_kbd_send_wr);
            send_report_to_downstream_for_id(HID_REPORT_ID_KEYBOARD, out, sizeof(out));
            C1_DBG_DN_K("[DN-K] sent keys=%02x %02x %02x %02x %02x %02x %02x %02x\n",
                     out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7]);
            if (s_kbd_pending || s_local_pb_pending || s_local_consumer_pending) {
                relay_request_send();
            }
        } else if (s_local_pb_pending) {
            // 4th priority: programmable button report (Phase 12)
            s_local_pb_pending = false;
            send_report_to_downstream_for_id(HID_REPORT_ID_PROG_BUTTON,
                                              s_local_pb, sizeof(s_local_pb));
            if (s_local_consumer_pending) {
                relay_request_send();
            }
        } else if (s_local_consumer_pending) {
            // 5th priority: consumer control report (bug #6 — media/volume keys)
            s_local_consumer_pending = false;
            send_report_to_downstream_for_id(HID_REPORT_ID_CONSUMER,
                                              s_local_consumer, sizeof(s_local_consumer));
        }
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Upstream HID report dispatch (from hids_host_handler in ble_central.c)
// ---------------------------------------------------------------------------

// `data`/`len` arrive with the Report ID prefix already stripped.
void ble_relay_on_upstream_report(uint8_t report_id, const uint8_t *data, uint16_t len) {
    // Strict split-mode matching: the upstream is a CMK sibling whose split_mode
    // disagrees with ours → the halves don't cooperate. Drop its HID entirely
    // (no BLE relay, no USB forward) until the states agree. 3rd-party upstreams
    // (no CMK-Split service) are never gated — the mouse relay stays untouched.
    if (ble_split_peer_is_cmk() && !ble_split_states_match()) {
        DIAG_INC(relay_mismatch_drop);
        return;
    }

    // Deferred upstream log — captured here (hot path), printed by log timer
#if DEBUG_UPSTREAM_K || DEBUG_UPSTREAM_M
    {
        uint8_t copy = (len < sizeof(s_log_report))
                       ? (uint8_t)len : (uint8_t)sizeof(s_log_report);
        memcpy(s_log_report, data, copy);
        s_log_len       = copy;
        s_log_report_id = report_id;
        s_log_count++;
        s_log_total++;
        s_log_has_data  = true;
    }
#endif

    // Classify the upstream report. With a valid descriptor map, a report is a
    // mouse if it carries X/Y usages — regardless of the upstream's report ID
    // numbering (3rd-party mice often use ID 1, which our static check used to
    // misroute into the keyboard merge). Without a map, fall back to assuming
    // the upstream uses our own ID assignment.
    bool is_mouse = g_ble_upstream_map_valid
                  ? hid_report_is_mouse(&g_ble_upstream_map, report_id)
                  : (report_id == HID_REPORT_ID_MOUSE);
    bool is_kbd   = !is_mouse && (report_id == HID_REPORT_ID_KEYBOARD);
    // Consumer (ID 4) is OUR report layout (bug #6). A 3rd-party upstream that
    // happens to number a report 4 means something else entirely: relaying it
    // into our consumer characteristic would fire random media usages, so it
    // stays on the raw-relay path only for a CMK sibling.
    bool is_consumer = !is_mouse && (report_id == HID_REPORT_ID_CONSUMER) &&
                       len == 2 && ble_split_peer_is_cmk();
    if (!is_mouse && !is_kbd && report_id == HID_REPORT_ID_CONSUMER && !is_consumer) {
        return;
    }

    // Record accepted input before output availability/queueing. Sibling input
    // is observed locally but is NOT re-originated on the control channel.
    if (!len) return;
    bool origin=!ble_split_peer_is_cmk();
    if (is_mouse && g_ble_upstream_map_valid) {
        hid_mouse_report_t decoded;
        if (!hid_decode_mouse(&g_ble_upstream_map,report_id,data,len,&decoded)) return;
        ++s_mouse_rx;
        ble_power_hid(origin,1,decoded.buttons!=0);
    } else if (is_kbd && len<=8) {
        bool held=false;
        for (uint16_t i=0;i<len;i++) if (i!=1 && data[i]) held=true;
        ble_power_hid(origin,0,held);
    } else {
        bool held=false;
        if (report_id==HID_REPORT_ID_PROG_BUTTON && len==4)
            held=(data[0]|data[1]|data[2]|data[3])!=0;
        else if (is_consumer)
            held=(data[0]|data[1])!=0;   // non-zero usage = key still held
        ble_power_hid(origin,2,held);
    }

#ifdef USB_ENABLE
    // Phase 14 ext: when USB is the selected output, route the upstream relay
    // to Core 0 (→ USB) so the BLE-chain keeps feeding USB. Mouse is forwarded
    // as-is (relative deltas). The upstream KEYBOARD contribution is forwarded
    // too, so Core 0 can merge it with the local report (merge on Core 0 for
    // USB, on Core 1 for BLE). Downstream BLE relay path left untouched.
    if (!output_ble_enabled()) {
        if (is_mouse && g_ble_upstream_map_valid) {
            hid_mouse_report_t mouse;
            if (hid_decode_mouse(&g_ble_upstream_map, report_id,
                                 data, len, &mouse)) {
                uint8_t buf[7];
                hid_pack_mouse(&mouse, buf);
                forward_upstream_to_usb(HID_REPORT_ID_MOUSE, buf, sizeof(buf));
            }
        } else if (is_kbd && len <= 8) {
            // Track upstream kbd state (for disconnect cleanup) and forward the
            // raw upstream contribution; Core 0 merges it with local keys.
            memcpy(s_upstream_kbd, data, len);
            if (len < 8) memset(s_upstream_kbd + len, 0, 8 - len);
            forward_upstream_to_usb(HID_REPORT_ID_KEYBOARD, s_upstream_kbd, 8);
        } else if (is_consumer) {
            forward_upstream_to_usb(HID_REPORT_ID_CONSUMER, data, 2);
        }
        return;
    }
#endif

    // Dispatch upstream report to downstream (Core 1 internal, zero latency)
    //
    // Mouse (X/Y usages in the upstream map): decode → normalise to our
    //   standard 7-byte format, sent as Report ID 2. Queued on the coalescing
    //   mouse queue (1st priority).
    //
    // Keyboard (Report ID 1): merge upstream keycodes with local keyscan state.
    //   OR modifiers, upstream keycodes first (6KRO combined).
    //   Queued via s_kbd_pending (2nd priority, same slot as local-only reports).
    //   When upstream disconnects, s_upstream_kbd is zeroed and a new merged
    //   report is sent automatically to release any upstream-held keys.
    //
    // Other report IDs: raw relay fallback.
    if (g_ble_periph_handle == HCI_CON_HANDLE_INVALID || !g_ble_periph_subscribed) {
        C1_DBG_STATUS("[RELAY] DROP id=%u len=%u (DS not ready: handle=%s sub=%d)\n",
                      report_id, len,
                      (g_ble_periph_handle == HCI_CON_HANDLE_INVALID) ? "INV" : "OK",
                      g_ble_periph_subscribed ? 1 : 0);
    }
    if (g_ble_periph_handle != HCI_CON_HANDLE_INVALID && g_ble_periph_subscribed) {
        if (is_mouse && g_ble_upstream_map_valid) {
            // Decode upstream format → standard 7-byte layout, enqueue on the
            // coalescing queue (always relayed downstream as Report ID 2,
            // whatever ID the upstream used).
            hid_mouse_report_t mouse;
            if (hid_decode_mouse(&g_ble_upstream_map, report_id,
                                 data, len, &mouse)) {
                s_us_rx_at = time_us_64();
                relay_mouse_enqueue(&mouse);
                if (relay_mouse_can_ship()) relay_request_send();
                else ++s_mouse_capped;
            }
        } else if (is_kbd && len <= 8) {
            // Merge upstream keyboard with local keyscan state
            memcpy(s_upstream_kbd, data, len);
            if (len < 8) memset(s_upstream_kbd + len, 0, 8 - len);
            kbd_merge_and_request();
        } else {
            // Fallback: raw relay for unknown/unhandled report IDs
            uint8_t copy_len = (len < sizeof(s_relay_report))
                               ? (uint8_t)len : (uint8_t)sizeof(s_relay_report);
            memcpy(s_relay_report, data, copy_len);
            s_relay_len       = copy_len;
            s_relay_report_id = report_id;
            s_relay_pending   = true;
            relay_request_send();
        }
    }
}

// ---------------------------------------------------------------------------
// Disconnect hooks
// ---------------------------------------------------------------------------

// R4: flush every per-connection send state. Stale queued reports
// must not leak into the next session — the first CAN_SEND_NOW after
// a reconnect (possibly to a DIFFERENT host) would type ghost keys.
void ble_relay_on_periph_disconnect(void) {
    s_tx_waiting=false;
    s_kbd_send_wr = s_kbd_send_rd = 0;
    memset(s_kbd_last_enq, 0, sizeof(s_kbd_last_enq));
    memset(s_kbd_merged,   0, sizeof(s_kbd_merged));
    s_kbd_pending         = false;
    s_relay_pending       = false;
    s_local_mouse_pending = false;
    s_local_pb_pending    = false;
    s_local_consumer_pending = false;
    s_relay_mouse_count   = 0;
    s_relay_mouse_rd      = 0;
    s_flight_count = s_flight_rd = 0;
    s_ds_completed_total  = 0;
}

// Release any keys that were held by upstream — zero upstream state and
// re-send the now local-only report. On BLE this rebuilds the merged
// downstream report; on USB it tells Core 0 the upstream contribution is
// gone so it re-merges and flushes to USB. Skip if upstream held nothing.
void ble_relay_on_central_disconnect(void) {
    ble_power_clear_upstream();
    static const uint8_t s_kbd_zero[8] = {0};
    if (memcmp(s_upstream_kbd, s_kbd_zero, 8) != 0) {
        memset(s_upstream_kbd, 0, sizeof(s_upstream_kbd));
#ifdef USB_ENABLE
        if (!output_ble_enabled()) {
            forward_upstream_to_usb(HID_REPORT_ID_KEYBOARD, s_upstream_kbd, 8);
        } else
#endif
        {
            kbd_merge_and_request();
        }
    }
}

// ---------------------------------------------------------------------------
// IPC slot polling + status publish (R3)
// ---------------------------------------------------------------------------

// Drain the WHOLE IPC kbd ring (a burst of fast typing can leave several
// queued). For each local state, update s_local_kbd and enqueue the merged
// report on the send ring so nothing is collapsed. On USB output Core 0 sends
// locally; here we just keep s_local_kbd current (merge baseline).
void kbd_drain_ipc_ring(void) {
    uint8_t r[8];
    while (ipc_get_kbd_report(r)) {
        memcpy(s_local_kbd, r, 8);
#ifdef USB_ENABLE
        if (!output_ble_enabled()) continue;
#endif
        kbd_merge_and_request();
    }
}

// Take the mouse slot if valid. A stale notify (slot already consumed) must NOT
// send an all-zero report — that would spuriously release held buttons mid-drag.
void mouse_slot_take(bool notified) {
    if (!ipc_get_mouse_report(s_local_mouse)) {
        if (notified) DIAG_INC(ipc_stale_skip);
        return;
    }
#ifdef USB_ENABLE
    if (!output_ble_enabled()) return;  // output == USB → Core 0 sends to USB
#endif
    s_local_mouse_pending = true;
    if (g_ble_periph_subscribed) {
        relay_request_send();
    }
}

void pb_slot_take(bool notified) {
    if (!ipc_get_pb_report(s_local_pb)) {
        if (notified) DIAG_INC(ipc_stale_skip);
        return;
    }
#ifdef USB_ENABLE
    if (!output_ble_enabled()) return;  // output == USB → Core 0 sends to USB
#endif
    s_local_pb_pending = true;
    if (g_ble_periph_subscribed) {
        relay_request_send();
    }
}

// Consumer report slot (bug #6) — same single-slot last-wins pattern as PB.
void consumer_slot_take(bool notified) {
    if (!ipc_get_consumer_report(s_local_consumer)) {
        if (notified) DIAG_INC(ipc_stale_skip);
        return;
    }
#ifdef USB_ENABLE
    if (!output_ble_enabled()) return;  // output == USB → Core 0 sends to USB
#endif
    s_local_consumer_pending = true;
    if (g_ble_periph_subscribed) {
        relay_request_send();
    }
}

// Called every IPC poll tick (5 ms) from core1_entry, INDEPENDENTLY of the FIFO:
//   - drains the report slots/rings, so a lost coalesced notify (full FIFO at
//     the exact empty→non-empty transition — the old "kbd output dead forever"
//     hazard) degrades to ≤5 ms of latency instead of a silent permanent stall;
//   - publishes the level-based BLE status snapshot that the CM task reconciles
//     against (a lost FIFO *event* no longer strands Core 0 in a ghost state).
void ble_manager_ipc_slots_poll(void) {
    kbd_drain_ipc_ring();
#if DEBUG_MOUSE_TRACE
    ble_relay_trace_poll();
#endif
    ble_link_policy_poll();
    mouse_slot_take(false);
    pb_slot_take(false);
    consumer_slot_take(false);

    ipc_ble_status_t st = {
        .periph      = (uint8_t)g_ble_periph_state,
        .central     = (uint8_t)g_ble_central_state,
        .active_slot = g_ble_active_slot,
        .split_role  = ble_split_role_value(),
        .peer_batt   = ble_batt_peer_value(),    // bug #3
        .mouse_batt  = ble_batt_mouse_value(),   // bug #3
        .valid       = true,
    };
    ipc_publish_ble_status(&st);
}

bool ble_relay_power_busy(void) {
    bool held=s_local_mouse[0]!=0;
    for (unsigned i=0;i<sizeof(s_local_pb);++i) held |= s_local_pb[i]!=0;
    for (unsigned i=0;i<sizeof(s_local_consumer);++i) held |= s_local_consumer[i]!=0;
    for (unsigned i=0;i<sizeof(s_local_kbd);++i) if (i!=1) held |= s_local_kbd[i]!=0;
    bool pending=s_local_mouse_pending || s_local_pb_pending || s_local_consumer_pending || s_kbd_pending || s_relay_pending || s_relay_mouse_count;
    // Queues staged while no host exists must not inhibit sleep forever.
    return held || (g_ble_periph_handle!=HCI_CON_HANDLE_INVALID && g_ble_periph_subscribed && pending);
}
