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
// led_manager.h — Core 0 LED manager task
//
// GP2  Power LED   always ON
// GP3  BLE LED     driven by BLE connection state (see led_manager.c)
//

#ifndef CMK_LED_MANAGER_H
#define CMK_LED_MANAGER_H

// Initialise GPIO for GP2 and GP3 and turn the power LED on immediately.
// Call from main() before vTaskStartScheduler().
void led_manager_init(void);

// Create and register the FreeRTOS LED task.
// Call from main() before vTaskStartScheduler().
void led_manager_create_task(void);

#endif // CMK_LED_MANAGER_H
