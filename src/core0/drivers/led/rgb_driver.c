/*
 * This file is part of CMK — Custom Mechanical Keyboard Firmware.
 *
 * Copyright (C) 2025-2026 André Cusimano <andre.cusimano@hotmail.it>
 * Copyright (C) 2016-2026 QMK Firmware contributors
 * Copyright (C) Jun Wako and TMK Keyboard Firmware contributors
 *
 * This file contains code derived from the QMK Firmware project
 * (https://github.com/qmk/qmk_firmware) and, indirectly, from the
 * TMK Keyboard Firmware by Jun Wako, both distributed under the
 * GNU General Public License version 2.
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
// rgb_driver.c — WS2812 PIO driver wrapper (Phase 13)
//

#include "rgb_driver.h"
#include "config/cmk_config.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pio/ws2812.pio.h"

#include <string.h>

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static rgb_t    s_led_buffer[RGB_LED_COUNT];
static uint32_t s_dma_buf[RGB_LED_COUNT];   // GRB<<8 words streamed to the PIO by DMA
static PIO      s_pio;
static uint     s_sm;
static int      s_dma_chan = -1;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

void rgb_driver_init(void) {
#ifdef PIN_VLED_EN
    // Enable the 5V LED rail (active HIGH). Must be set before driving DIN,
    // otherwise the first PIO frame is sent into a powered-off strip and the
    // chain ends up in an undefined state on power-up.
    gpio_init(PIN_VLED_EN);
    gpio_set_dir(PIN_VLED_EN, GPIO_OUT);
    gpio_put(PIN_VLED_EN, 1);
#endif
    s_pio = pio0;

#if PICO_PIO_USE_GPIO_BASE && (PIN_RGB >= 32)
    // RP2350B: PIO can address either GPIO 0-31 (gpio_base=0, default) or
    // 16-47 (gpio_base=16). PIN_RGB is on GP35 here, so the PIO instance
    // must be flipped to the upper range BEFORE loading the program (the
    // instruction patcher reads gpio_base to encode SIDESET pins).
    pio_set_gpio_base(s_pio, 16);
#endif

    s_sm  = pio_claim_unused_sm(s_pio, true);
    uint offset = pio_add_program(s_pio, &ws2812_program);
    ws2812_program_init(s_pio, s_sm, offset, PIN_RGB, 800000.f, false);
    memset(s_led_buffer, 0, sizeof(s_led_buffer));
    memset(s_dma_buf, 0, sizeof(s_dma_buf));

    // DMA feeds the PIO TX FIFO so the WS2812 frame streams out independently of
    // CPU/IRQ load. CPU-fed (pio_sm_put_blocking) flushes underrun the FIFO when
    // the keyscan task is preempted mid-frame → visible LED flicker.
    s_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(s_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(s_pio, s_sm, true));  // TX DREQ
    dma_channel_configure(s_dma_chan, &c,
                          &s_pio->txf[s_sm],   // write: PIO TX FIFO
                          s_dma_buf,           // read: GRB buffer
                          RGB_LED_COUNT,
                          false);              // don't start yet
}

void rgb_driver_set_led(uint16_t idx, rgb_t color) {
    if (idx < RGB_LED_COUNT) {
        s_led_buffer[idx] = color;
    }
}

void rgb_driver_set_all(rgb_t color) {
    for (uint16_t i = 0; i < RGB_LED_COUNT; i++) {
        s_led_buffer[i] = color;
    }
}

void rgb_driver_flush(void) {
    // Wait for the previous frame's DMA to finish before reusing the buffer
    // (at ~30 fps it completed long ago — returns immediately).
    if (s_dma_chan >= 0) {
        dma_channel_wait_for_finish_blocking(s_dma_chan);
    }

    for (uint16_t i = 0; i < RGB_LED_COUNT; i++) {
        // WS2812 expects GRB order, left-shifted 8 bits for PIO bit count
        s_dma_buf[i] = ((uint32_t)s_led_buffer[i].g << 24)
                     | ((uint32_t)s_led_buffer[i].r << 16)
                     | ((uint32_t)s_led_buffer[i].b <<  8);
    }

    // Kick off the DMA: it streams the frame to the PIO FIFO in the background,
    // immune to CPU preemption / IRQ latency.
    if (s_dma_chan >= 0) {
        dma_channel_set_read_addr(s_dma_chan, s_dma_buf, true);  // true = trigger
    }
}

void rgb_driver_clear(void) {
    memset(s_led_buffer, 0, sizeof(s_led_buffer));
    rgb_driver_flush();
}

void rgb_driver_power(bool on) {
#ifdef PIN_VLED_EN
    // Gate the 5V LED boost rail. Cutting it in low-power sleep removes the
    // boost regulator's quiescent draw and any residual LED current. On power-up
    // the rail must be HIGH before WS2812 data is driven (see rgb_driver_init);
    // the caller (rgb_matrix_tick) skips one frame after re-powering so the rail
    // settles before the next flush.
    gpio_put(PIN_VLED_EN, on ? 1 : 0);
#else
    (void)on;
#endif
}

// ---------------------------------------------------------------------------
// HSV → RGB (QMK-compatible, 6-sector hue wheel)
// ---------------------------------------------------------------------------

rgb_t hsv_to_rgb(hsv_t hsv) {
    if (hsv.s == 0) {
        return (rgb_t){ hsv.v, hsv.v, hsv.v };
    }

    uint8_t region    = hsv.h / 43;
    uint8_t remainder = (hsv.h - (region * 43)) * 6;

    uint8_t p = (uint8_t)((uint16_t)hsv.v * (255 - hsv.s) / 255);
    uint8_t q = (uint8_t)((uint16_t)hsv.v * (255 - ((uint16_t)hsv.s * remainder / 255)) / 255);
    uint8_t t = (uint8_t)((uint16_t)hsv.v * (255 - ((uint16_t)hsv.s * (255 - remainder) / 255)) / 255);

    switch (region) {
        case 0:  return (rgb_t){ hsv.v, t, p };
        case 1:  return (rgb_t){ q, hsv.v, p };
        case 2:  return (rgb_t){ p, hsv.v, t };
        case 3:  return (rgb_t){ p, q, hsv.v };
        case 4:  return (rgb_t){ t, p, hsv.v };
        default: return (rgb_t){ hsv.v, p, q };
    }
}
