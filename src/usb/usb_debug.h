// SPDX-License-Identifier: GPL-2.0-only
#ifndef CMK_USB_DEBUG_H
#define CMK_USB_DEBUG_H

#include "config/cmk_config.h"
#include "config/debug_config.h"

#if defined(USB_ENABLE) && DEBUG_USB_SERIAL
// Core 0 only. Init after stdio_init_all(), before the first boot log.
void usb_debug_init(void);
// Called only by the existing USB task, after tud_task().
void usb_debug_poll(void);
#else
static inline void usb_debug_init(void) {}
static inline void usb_debug_poll(void) {}
#endif
#endif
