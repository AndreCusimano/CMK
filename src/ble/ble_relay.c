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

// One-shot per connection: ask the downstream host (master) for a tight
// connection interval — we are slave here, so the L2CAP request is the
// correct API. Hosts default to 30–50 ms, which caps the mouse relay rate.
static bool    s_conn_params_requested = false;

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
        hids_device_request_can_send_now_event(g_ble_periph_handle);
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
        s_relay_mouse_rd = (uint8_t)((s_relay_mouse_rd + 1u) % RELAY_MOUSE_SLOTS);
        s_relay_mouse_count--;
        DIAG_INC(relay_mouse_fold);
    }
    s_relay_mouse_q[(s_relay_mouse_rd + s_relay_mouse_count) % RELAY_MOUSE_SLOTS] = *m;
    s_relay_mouse_count++;
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
static void send_report_to_downstream_for_id(uint8_t report_id,
                                              const uint8_t *data, uint8_t len) {
    uint8_t status;
    if (s_protocol_mode == 0) {
        // Boot protocol: only keyboard supported
        if (report_id == HID_REPORT_ID_KEYBOARD) {
            status = hids_device_send_boot_keyboard_input_report(g_ble_periph_handle, data, len);
        } else {
            return;
        }
    } else {
        status = hids_device_send_input_report_for_id(g_ble_periph_handle, report_id, data, len);
    }
    UNUSED(status);
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
            // Ask the host for a 7.5–15 ms interval (slave → L2CAP request;
            // the host may grant a close value or refuse — best effort).
            if (!s_conn_params_requested) {
                s_conn_params_requested = true;
                gap_request_connection_parameter_update(g_ble_periph_handle, 6, 12, 0, 400);
            }
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
        if (s_relay_mouse_count > 0) {
            // 1st priority: mouse relay from upstream (decoded, normalised).
            // Pop ONE queued state (FIFO) and re-arm while entries remain —
            // button transitions are distinct entries, motion is pre-coalesced.
            uint8_t buf[7];
            hid_pack_mouse(&s_relay_mouse_q[s_relay_mouse_rd], buf);
            s_relay_mouse_rd = (uint8_t)((s_relay_mouse_rd + 1u) % RELAY_MOUSE_SLOTS);
            s_relay_mouse_count--;
            send_report_to_downstream_for_id(HID_REPORT_ID_MOUSE, buf, sizeof(buf));
            C1_DBG_DN_M("[DN-M] sent: %02x %02x %02x %02x %02x %02x %02x (q=%u)\n",
                     buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
                     s_relay_mouse_count);
            if (s_relay_mouse_count || s_relay_pending ||
                s_local_mouse_pending || s_kbd_pending || s_local_pb_pending) {
                hids_device_request_can_send_now_event(g_ble_periph_handle);
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
            if (s_local_mouse_pending || s_kbd_pending || s_local_pb_pending) {
                hids_device_request_can_send_now_event(g_ble_periph_handle);
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
            if (s_kbd_pending || s_local_pb_pending) {
                hids_device_request_can_send_now_event(g_ble_periph_handle);
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
            if (s_kbd_pending || s_local_pb_pending) {
                hids_device_request_can_send_now_event(g_ble_periph_handle);
            }
        } else if (s_local_pb_pending) {
            // 4th priority: programmable button report (Phase 12)
            s_local_pb_pending = false;
            send_report_to_downstream_for_id(HID_REPORT_ID_PROG_BUTTON,
                                              s_local_pb, sizeof(s_local_pb));
        }
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Upstream HID report dispatch (from hids_client_handler in ble_central.c)
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
                relay_mouse_enqueue(&mouse);
                hids_device_request_can_send_now_event(g_ble_periph_handle);
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
            hids_device_request_can_send_now_event(g_ble_periph_handle);
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
    s_kbd_send_wr = s_kbd_send_rd = 0;
    memset(s_kbd_last_enq, 0, sizeof(s_kbd_last_enq));
    memset(s_kbd_merged,   0, sizeof(s_kbd_merged));
    s_kbd_pending         = false;
    s_relay_pending       = false;
    s_local_mouse_pending = false;
    s_local_pb_pending    = false;
    s_relay_mouse_count   = 0;
    s_relay_mouse_rd      = 0;
    s_conn_params_requested = false;
}

// Release any keys that were held by upstream — zero upstream state and
// re-send the now local-only report. On BLE this rebuilds the merged
// downstream report; on USB it tells Core 0 the upstream contribution is
// gone so it re-merges and flushes to USB. Skip if upstream held nothing.
void ble_relay_on_central_disconnect(void) {
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
void mouse_slot_take(void) {
    if (!ipc_get_mouse_report(s_local_mouse)) {
        DIAG_INC(ipc_stale_skip);
        return;
    }
#ifdef USB_ENABLE
    if (!output_ble_enabled()) return;  // output == USB → Core 0 sends to USB
#endif
    s_local_mouse_pending = true;
    if (g_ble_periph_subscribed) {
        hids_device_request_can_send_now_event(g_ble_periph_handle);
    }
}

void pb_slot_take(void) {
    if (!ipc_get_pb_report(s_local_pb)) {
        DIAG_INC(ipc_stale_skip);
        return;
    }
#ifdef USB_ENABLE
    if (!output_ble_enabled()) return;  // output == USB → Core 0 sends to USB
#endif
    s_local_pb_pending = true;
    if (g_ble_periph_subscribed) {
        hids_device_request_can_send_now_event(g_ble_periph_handle);
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
    mouse_slot_take();
    pb_slot_take();

    ipc_ble_status_t st = {
        .periph      = (uint8_t)g_ble_periph_state,
        .central     = (uint8_t)g_ble_central_state,
        .active_slot = g_ble_active_slot,
        .split_role  = ble_split_role_value(),
        .valid       = true,
    };
    ipc_publish_ble_status(&st);
}
