// SPDX-License-Identifier: GPL-2.0-only
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "power/power_policy.h"
// Core 1 / BTstack context only. Chip-local readers use ipc_power_get instead.
void ble_power_poll(void);
void ble_power_request_off(uint16_t token);
void ble_power_hid(bool origin, unsigned kind, bool held);
void ble_power_clear_upstream(void);
void ble_power_peer_keys(uint32_t bitmap);
void ble_power_link(uint16_t handle, bool present, bool ready);
bool ble_power_receive(uint16_t handle, const uint8_t *data, unsigned len);
void ble_power_frame(uint8_t out[POWER_WIRE_SIZE]);
