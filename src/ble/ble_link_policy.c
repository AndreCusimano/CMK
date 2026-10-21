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
// ble_link_policy.c — keep every BLE link at BLE_LINK_INTERVAL (Core 1 only)
//
// Why one interval everywhere: the relaying half's CYW43439 schedules the
// upstream link (it is master) and the downstream link (it is slave, clock of
// the other half) together. When their events overlap it drops the downstream
// one; with mismatched intervals that loss is either spread over time or, for
// harmonic ratios, latched for minutes (docs/Mouse_Lag_Investigation.md §11-20).
// Measured best: every hop we control at 7.5 ms. The controller also picks
// harmonic intervals for its own master links, so the value must be IMPOSED on
// both sides, not requested once.
//
// Master links (upstream mouse, inter-half link on the master half): whatever
// the peer negotiates, re-issue an HCI update to BLE_LINK_INTERVAL keeping the
// peer's latency and timeout (latency only lets an idle mouse skip events and
// save battery; the master's schedule is unchanged). The accepted-range API
// (gap_set_connection_parameter_range) is NOT used to refuse peers: BTstack
// applies it to LL requests on slave links too, so pinning it would also refuse
// the host PC's own parameter requests.
//
// Slave link (downstream host): we can only ask. Ladder of L2CAP requests,
// each checked against the parameters actually in force: exact target, then
// 11.25 ms (Apple's HID floor), then the host's pick inside 7.5–15 ms.
//

#include "ble_link_policy.h"
#include "ble_internal.h"
#include "ipc/ipc_core.h"
#include "config/cmk_config.h"
#include "config/debug_config.h"

#define ENFORCE_DELAY_MS   500     // let the peer's own procedure finish first
#define ENFORCE_RETRY_MS   3000
#define ENFORCE_BACKOFF_MS 30000   // after ENFORCE_FAST_TRIES failures in a row
#define ENFORCE_FAST_TRIES 5
#define DS_STEP_WAIT_MS    3000    // per ladder step: response + update complete
#define DS_TIMEOUT_10MS    400

typedef struct {
    hci_con_handle_t handle;
    uint16_t interval, latency, timeout;
    uint32_t due_ms;               // 0 = nothing to do
    uint8_t  failures;
} master_link_t;

typedef struct {
    hci_con_handle_t handle;
    uint16_t interval;             // in force, 0 = unknown
    uint8_t  step;                 // next ladder step to send
    uint32_t due_ms;               // 0 = idle / finished
    bool     active;
    bool     started;              // ladder already run for this connection
} slave_link_t;

static const uint16_t s_ladder[][2] = {
    {BLE_LINK_INTERVAL, BLE_LINK_INTERVAL},
    {9, 9},                        // 11.25 ms: accepted by hosts that refuse 7.5
    {6, 12},                       // host's choice inside 7.5–15 ms
};
#define LADDER_STEPS (sizeof(s_ladder) / sizeof(s_ladder[0]))

static master_link_t s_master = {.handle = HCI_CON_HANDLE_INVALID};
static slave_link_t  s_slave  = {.handle = HCI_CON_HANDLE_INVALID};

static uint32_t now_ms(void) {
    uint32_t t = btstack_run_loop_get_time_ms();
    return t ? t : 1;              // 0 means "not scheduled"
}

// Record 18: kind, handle, min, max, value.
//   kind 1 master update issued  (value = latency<<8 | rc)
//   kind 2 slave request sent    (value = step<<8 | rc)
//   kind 3 slave request answer  (value = L2CAP result, 0 accepted)
//   kind 4 slave ladder finished (min = max = interval in force, value = step)
static void trace(uint32_t kind, hci_con_handle_t handle, uint16_t min, uint16_t max, uint32_t value) {
    uint32_t record[7] = {18, btstack_run_loop_get_time_ms(), kind, handle, min, max, value};
    ipc_trace_put(record);
}

uint16_t ble_link_policy_fit_latency(uint16_t interval, uint16_t latency, uint16_t timeout) {
    if (!interval) return 0;
    // timeout*10 ms > (1+latency) * interval*1.25 ms * 2  ⇔  4*timeout > (1+latency)*interval
    uint32_t max_plus_one = (4u * timeout - 1u) / interval;
    uint32_t max_latency  = max_plus_one ? max_plus_one - 1u : 0u;
    if (max_latency > 499u) max_latency = 499u;
    return (uint16_t)(latency < max_latency ? latency : max_latency);
}

void ble_link_policy_on_params(hci_con_handle_t handle, bool master,
                               uint16_t interval, uint16_t latency, uint16_t timeout) {
    if (master) {
        if (s_master.handle != handle) s_master.failures = 0;
        s_master.handle   = handle;
        s_master.interval = interval;
        s_master.latency  = latency;
        s_master.timeout  = timeout;
        if (interval == BLE_LINK_INTERVAL) {
            s_master.due_ms = 0;
            s_master.failures = 0;
        } else if (!s_master.due_ms) {
            s_master.due_ms = now_ms() + ENFORCE_DELAY_MS;
        }
        return;
    }
    if (s_slave.handle != handle) {
        s_slave = (slave_link_t){.handle = handle};
    }
    s_slave.interval = interval;
    if (s_slave.active && interval >= s_ladder[0][0] && interval <= s_ladder[0][1]) {
        s_slave.active = false; s_slave.due_ms = 0;
        trace(4, handle, interval, interval, s_slave.step);
    }
}

void ble_link_policy_on_disconnect(hci_con_handle_t handle) {
    if (s_master.handle == handle) s_master = (master_link_t){.handle = HCI_CON_HANDLE_INVALID};
    if (s_slave.handle  == handle) s_slave  = (slave_link_t){.handle = HCI_CON_HANDLE_INVALID};
}

void ble_link_policy_on_ds_subscribed(hci_con_handle_t handle) {
    // Once per connection: HIDS reports one subscribe per report ID.
    if (s_slave.handle == handle && s_slave.started) return;
    uint16_t interval = s_slave.handle == handle ? s_slave.interval : 0;
    s_slave = (slave_link_t){.handle = handle, .interval = interval, .active = true, .started = true,
                             .due_ms = now_ms()};
    if (interval == BLE_LINK_INTERVAL) {
        s_slave.active = false; s_slave.due_ms = 0;
        trace(4, handle, interval, interval, 0);
    }
}

void ble_link_policy_on_ds_response(hci_con_handle_t handle, uint16_t result) {
    if (!s_slave.active || s_slave.handle != handle) return;
    trace(3, handle, 0, 0, result);
    // Rejected: no update is coming, move to the next step right away.
    if (result != 0) s_slave.due_ms = now_ms();
}

static void master_poll(uint32_t now) {
    if (!s_master.due_ms || (int32_t)(now - s_master.due_ms) < 0) return;
    if (s_master.handle == HCI_CON_HANDLE_INVALID || s_master.interval == BLE_LINK_INTERVAL) {
        s_master.due_ms = 0; return;
    }
    uint16_t latency = ble_link_policy_fit_latency(BLE_LINK_INTERVAL, s_master.latency, s_master.timeout);
    uint8_t rc = gap_update_connection_parameters(s_master.handle, BLE_LINK_INTERVAL, BLE_LINK_INTERVAL,
                                                  latency, s_master.timeout);
    trace(1, s_master.handle, BLE_LINK_INTERVAL, BLE_LINK_INTERVAL, ((uint32_t)latency << 8) | rc);
    C1_DBG_STATUS("[LINK] master 0x%04x at %u units → request %u (latency %u) rc=0x%02x\n",
                  s_master.handle, s_master.interval, BLE_LINK_INTERVAL, latency, rc);
    // Success lands as an update event that clears due_ms; otherwise retry.
    if (s_master.failures < 255) ++s_master.failures;
    s_master.due_ms = now + (s_master.failures > ENFORCE_FAST_TRIES ? ENFORCE_BACKOFF_MS : ENFORCE_RETRY_MS);
}

static void slave_poll(uint32_t now) {
    if (!s_slave.active || !s_slave.due_ms || (int32_t)(now - s_slave.due_ms) < 0) return;
    // The previous step settled on something inside its own range: accept it,
    // unless it is the first step (exact target, handled in on_params).
    if (s_slave.step > 0 && s_slave.interval &&
        s_slave.interval >= s_ladder[s_slave.step - 1][0] && s_slave.interval <= s_ladder[s_slave.step - 1][1]) {
        s_slave.active = false; s_slave.due_ms = 0;
        trace(4, s_slave.handle, s_slave.interval, s_slave.interval, s_slave.step);
        return;
    }
    if (s_slave.step >= LADDER_STEPS) {
        s_slave.active = false; s_slave.due_ms = 0;
        trace(4, s_slave.handle, s_slave.interval, s_slave.interval, s_slave.step);
        return;
    }
    const uint16_t *range = s_ladder[s_slave.step];
    uint8_t rc = gap_request_connection_parameter_update(s_slave.handle, range[0], range[1], 0, DS_TIMEOUT_10MS);
    trace(2, s_slave.handle, range[0], range[1], ((uint32_t)s_slave.step << 8) | rc);
    C1_DBG_STATUS("[LINK] downstream 0x%04x at %u units → ask %u-%u rc=0x%02x\n",
                  s_slave.handle, s_slave.interval, range[0], range[1], rc);
    ++s_slave.step;
    s_slave.due_ms = now + DS_STEP_WAIT_MS;
}

void ble_link_policy_poll(void) {
    uint32_t now = now_ms();
    master_poll(now);
    slave_poll(now);
}
