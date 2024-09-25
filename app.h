/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H

#include <stdint.h>
#include "app_timer.h"

#define USS_TRIGGER_PORT        gpioPortA
#define USS_TRIGGER_PIN         8
#define USS_ECHO_PORT           gpioPortA
#define USS_ECHO_PIN            7
#define IR_PORT                 gpioPortA
#define IR_PIN                  4

#define TIMER_PERIODIC_PERIOD   1000
#define TIMER_TRIGGER_PERIOD    1

void gpio_init(void);
void gpio_callback(uint8_t pin);

void uart_init_dma(void);
void uart_dma_send(void);
void uart_dma_receive(void);

void timer_periodic_init(void);
void timer_periodic_callback(app_timer_t *timer, void *data);
void timer_trigger_pulse_init(void);
void timer_trigger_pulse_callback(app_timer_t *timer, void *data);

void handle_device_states(void);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void);

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);

sl_status_t ir_sensor_update_characteristic(void);
sl_status_t ir_sensor_send_notification(void);
sl_status_t uss_distance_update_characteristic(void);
sl_status_t uss_distance_send_notification(void);

#endif // APP_H
