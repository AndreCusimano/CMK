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
// usb_hid.c — TinyUSB HID device (Phase 14a)
//
// FreeRTOS task wrapping tud_task(). Report send functions.
// LED indicator callback from USB host Output Report.
//

#include "config/cmk_config.h"

#ifdef USB_ENABLE

#include "usb_hid.h"
#include "usb_debug.h"
#include "usb_descriptors.h"
#include "tusb.h"
#include "led_indicators.h"
#include "diag.h"
#include "ipc/ipc_core.h"

#ifdef RAW_HID_ENABLE
#include "raw_hid.h"
#endif

#include "FreeRTOS.h"
#include "task.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

// Volatile: read by Core 1 (ble_manager), written by Core 0 (USB callbacks)
static volatile bool s_usb_active = false;

// Gate: tusb_init() must NOT run while Core 1 is bringing up CYW43/BTstack
// (the "BT FW download" phase). Calling tusb_init() concurrently with that
// bringup deterministically hangs Core 1. Set true once BLE radio is up
// (IPC_BLE_WORKING → connection_manager → usb_hid_start()).
static volatile bool s_usb_start = false;

void usb_hid_start(void) {
    s_usb_start = true;
}

// ---------------------------------------------------------------------------
// FreeRTOS task
// ---------------------------------------------------------------------------

static void usb_device_task(void *param) {
    (void)param;

    // Wait for Core 1 BLE bringup to finish before touching the USB controller.
    while (!s_usb_start) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    tusb_init();

    while (1) {
        DIAG_TASK_ALIVE(DIAG_TASK_USB);
        tud_task();
        usb_debug_poll();
        // tud_task() does NOT block here: the pico-sdk compiles TinyUSB with
        // CFG_TUSB_OS=OPT_OS_PICO (family.cmake overrides our tusb_config.h),
        // so without this delay the loop busy-spins at priority 2 and STARVES
        // every priority-1 task (ih_link, led_mgr) — the I2C1 link never left
        // S_SLAVE and the peer join always failed. 1 ms costs nothing: the USB
        // FS interrupt endpoint is host-polled at 1 ms anyway.
        // Proper future fix: set(TINYUSB_OPT_OS OPT_OS_FREERTOS) before
        // pico_sdk_init() and validate ISR-safety — see docs/fixes/Split.md.
        //
        // Low-power: when the cable is unplugged (running on battery) poll slower
        // so this task stops waking Core 0 every ms and tickless idle can sleep.
        // Gate on tud_connected() (NOT tud_mounted): keep the fast 1 ms cadence
        // through the whole connect→enumerate→mount window so enumeration is not
        // stalled. A connect event raises a USB IRQ that wakes the core anyway.
        vTaskDelay(pdMS_TO_TICKS(tud_connected() ? 1u : USB_IDLE_POLL_MS));
    }
}

void usb_hid_task_create(void) {
    xTaskCreate(usb_device_task, "USB", USB_TASK_STACK, NULL, USB_TASK_PRIORITY, NULL);
}

// ---------------------------------------------------------------------------
// TinyUSB device callbacks — mount/unmount
// ---------------------------------------------------------------------------

void tud_mount_cb(void) {
    s_usb_active = true;
    ipc_power_usb(true);
}

void tud_umount_cb(void) {
    s_usb_active = false;
    ipc_power_usb(false);
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    s_usb_active = false;
    ipc_power_usb(false);
}

void tud_resume_cb(void) {
    s_usb_active = true;
    ipc_power_usb(true);
}

// ---------------------------------------------------------------------------
// Report sending
// ---------------------------------------------------------------------------

bool usb_hid_send_kbd_report(const uint8_t report[8]) {
    if (!s_usb_active || !tud_hid_n_ready(ITF_NUM_HID_KBD)) {
        DIAG_INC(usb_kbd_drop);
        return false;
    }
    if (!tud_hid_n_report(ITF_NUM_HID_KBD, HID_REPORT_ID_KEYBOARD, report, 8)) {
        DIAG_INC(usb_kbd_drop);
        return false;
    }
    return true;
}

void usb_hid_send_mouse_report(const uint8_t report[7]) {
    if (!s_usb_active || !tud_hid_n_ready(ITF_NUM_HID_KBD)) {
        DIAG_INC(usb_mouse_drop);
        return;
    }
    if (!tud_hid_n_report(ITF_NUM_HID_KBD, HID_REPORT_ID_MOUSE, report, 7)) {
        DIAG_INC(usb_mouse_drop);
    }
}

void usb_hid_send_pb_report(const uint8_t report[4]) {
    if (!s_usb_active || !tud_hid_n_ready(ITF_NUM_HID_KBD)) {
        DIAG_INC(usb_pb_drop);
        return;
    }
    if (!tud_hid_n_report(ITF_NUM_HID_KBD, HID_REPORT_ID_PROG_BUTTON, report, 4)) {
        DIAG_INC(usb_pb_drop);
    }
}

void usb_hid_send_consumer_report(const uint8_t report[2]) {
    if (!s_usb_active || !tud_hid_n_ready(ITF_NUM_HID_KBD)) {
        DIAG_INC(usb_pb_drop);   // shared drop counter (consumer events are rare)
        return;
    }
    if (!tud_hid_n_report(ITF_NUM_HID_KBD, HID_REPORT_ID_CONSUMER, report, 2)) {
        DIAG_INC(usb_pb_drop);
    }
}

bool usb_hid_is_active(void) {
    return s_usb_active;
}

// ---------------------------------------------------------------------------
// TinyUSB HID callbacks
// ---------------------------------------------------------------------------

// Called when host sends GET_REPORT (not commonly used for keyboard)
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type,
                                uint8_t* buffer, uint16_t reqlen) {
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer; (void)reqlen;
    return 0;
}

// Called when host sends SET_REPORT (Output Report = LED indicators, Raw HID data)
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                            hid_report_type_t report_type,
                            uint8_t const* buffer, uint16_t bufsize) {
    // Interface 0: LED Output Report (Caps Lock, Num Lock, Scroll Lock)
    if (instance == ITF_NUM_HID_KBD &&
        report_type == HID_REPORT_TYPE_OUTPUT &&
        report_id == HID_REPORT_ID_KEYBOARD &&
        bufsize >= 1) {
        led_indicators_set_state(buffer[0]);
    }

#ifdef RAW_HID_ENABLE
    // Interface 1: Raw HID data from host (WebHID)
    if (instance == ITF_NUM_HID_RAW) {
        raw_hid_receive_handler(buffer, bufsize);
    }
#endif
}

#endif // USB_ENABLE
