// SPDX-License-Identifier: GPL-2.0-only
#include "usb_debug.h"

#if defined(USB_ENABLE) && DEBUG_USB_SERIAL
#include <string.h>
#include "hardware/sync.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_uart.h"
#include "pico/platform.h"
#include "ipc/ipc_core.h"
#include "tusb.h"

// Preserve early boot logs even before USB bringup / a host opening the port.
// Core 0 producers only; Core 1 continues to use the existing IPC log ring.
// Full buffer drops NEW bytes, never waits for USB. IRQ masking also excludes
// FreeRTOS task switches, works before the scheduler, and covers <=64 bytes.
#define LOG_CAPACITY 8192u
#define LOG_CHUNK 64u
static char s_log[LOG_CAPACITY];
static uint32_t s_read, s_count, s_dropped;

static void log_write(const char *buf, int len) {
    // CYW43's SDK diagnostics still call printf directly on Core 1. Forward
    // those through IPC too: masking Core 0 IRQs cannot protect against them.
    if (get_core_num() != 0) {
        while (len > 0) {
            int n = len < IPC_LOG_SLOT_SIZE - 1 ? len : IPC_LOG_SLOT_SIZE - 1;
            core1_log("%.*s", n, buf);
            buf += n;
            len -= n;
        }
        return;
    }
    while (len > 0) {
        uint32_t irq = save_and_disable_interrupts();
        uint32_t n = (uint32_t)len;
        if (n > LOG_CHUNK) n = LOG_CHUNK;
        if (n > LOG_CAPACITY - s_count) n = LOG_CAPACITY - s_count;
        if (!n) {
            uint32_t lost = (uint32_t)len;
            s_dropped = lost > UINT32_MAX - s_dropped ? UINT32_MAX : s_dropped + lost;
            restore_interrupts(irq);
            return;
        }
        uint32_t tail = (s_read + s_count) % LOG_CAPACITY;
        uint32_t first = LOG_CAPACITY - tail;
        if (first > n) first = n;
        memcpy(s_log + tail, buf, first);
        memcpy(s_log, buf + first, n - first);
        s_count += n;
        restore_interrupts(irq);
        buf += n;
        len -= (int)n;
    }
}

static stdio_driver_t s_driver = {
    .out_chars = log_write,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = true,
#endif
};

void usb_debug_init(void) {
    // Keep SDK stdio_usb disabled: it would own TinyUSB init/polling and
    // conflict with our descriptors and the BLE WORKING startup gate.
#if LIB_PICO_STDIO_UART
    stdio_set_driver_enabled(&stdio_uart, false);
#endif
    stdio_set_driver_enabled(&s_driver, true);
}

void usb_debug_poll(void) {
    // CDC is a log output, not a command shell. Drain unsolicited input.
    if (tud_cdc_available()) tud_cdc_read_flush();
    if (!tud_cdc_connected()) return; // DTR; never wait for a monitor

    // Announce overflow as soon as transport has room, even under continuous
    // logging. Retain a partially written notice across polls/disconnection.
    static char notice[64];
    static uint32_t notice_len, notice_pos;
    if (!notice_len) {
        uint32_t irq = save_and_disable_interrupts();
        uint32_t dropped = s_dropped;
        s_dropped = 0;
        restore_interrupts(irq);
        if (dropped) notice_len = (uint32_t)snprintf(notice, sizeof(notice),
                "\r\n[USB-LOG] dropped %lu bytes\r\n", (unsigned long)dropped);
    }
    if (notice_len) {
        notice_pos += tud_cdc_write(notice + notice_pos, notice_len - notice_pos);
        if (notice_pos != notice_len) {
            tud_cdc_write_flush();
            return;
        }
        notice_len = notice_pos = 0;
    }

    // Fixed work budget per USB tick, so logs cannot monopolise this task.
    char buf[LOG_CHUNK];
    for (unsigned chunk = 0; chunk < 4; ++chunk) {
        uint32_t available = tud_cdc_write_available();
        if (!available) break;
        uint32_t irq = save_and_disable_interrupts();
        uint32_t n = s_count;
        if (n > sizeof(buf)) n = sizeof(buf);
        if (n > available) n = available;
        for (uint32_t i = 0; i < n; ++i) buf[i] = s_log[(s_read + i) % LOG_CAPACITY];
        restore_interrupts(irq);
        if (!n) break;
        // No USB calls with interrupts masked. Only this task consumes CDC.
        uint32_t sent = tud_cdc_write(buf, n);
        irq = save_and_disable_interrupts();
        s_read = (s_read + sent) % LOG_CAPACITY;
        s_count -= sent;
        restore_interrupts(irq);
        if (sent < n) break;
    }

    tud_cdc_write_flush();
}
#endif
