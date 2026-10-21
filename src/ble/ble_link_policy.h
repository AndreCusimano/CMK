//
// ble_link_policy.h — connection interval policy for every BLE link (Core 1 only)
//

#ifndef CMK_BLE_LINK_POLICY_H
#define CMK_BLE_LINK_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#include "btstack.h"

// A link event (connection complete / connection update complete, success
// only) for `handle` with the parameters now in force. `master` = we are
// central on it.
void ble_link_policy_on_params(hci_con_handle_t handle, bool master,
                               uint16_t interval, uint16_t latency, uint16_t timeout);
void ble_link_policy_on_disconnect(hci_con_handle_t handle);
// The downstream host subscribed to our HID reports: start asking it for
// BLE_LINK_INTERVAL (peripheral side, L2CAP request ladder).
void ble_link_policy_on_ds_subscribed(hci_con_handle_t handle);
// L2CAP Connection Parameter Update Response for a request we sent.
void ble_link_policy_on_ds_response(hci_con_handle_t handle, uint16_t result);
// Every IPC poll tick (5 ms).
void ble_link_policy_poll(void);

// Pure helpers (host-tested).
// Largest latency ≤ `latency` that keeps the supervision timeout valid at
// `interval`: timeout(10 ms) > (1 + latency) * interval(1.25 ms) * 2.
uint16_t ble_link_policy_fit_latency(uint16_t interval, uint16_t latency, uint16_t timeout);

#endif // CMK_BLE_LINK_POLICY_H
