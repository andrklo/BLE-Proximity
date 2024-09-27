/***************************************************************************//**
 * @file
 * @brief Core application logic.
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
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "app.h"
#include "gatt_db.h"
#include "dmadrv.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "app_timer.h"
#include "ble_proximity.h"

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// BLE proximity
static ble_proximity_handle_t ble_proximity_handle;

// Uart
static unsigned int uart_dma_channel_rx;
static unsigned int uart_dma_channel_tx;

// Timers
static app_timer_t trigger_pulse_timer;
static app_timer_t periodic_timer;
static volatile bool periodic_timer_flag = false;

// Инициализация GPIO ()
void gpio_init(void)
{
  // Настройка Trigger пина как output
  GPIO_PinModeSet(USS_TRIGGER_PORT,
                  USS_TRIGGER_PIN,
                  gpioModePushPull,
                  1);
  GPIO_PinModeSet(USS_ECHO_PORT,
                  USS_ECHO_PIN,
                  gpioModeInputPull,
                  1);
  GPIO_PinModeSet(IR_PORT,
                  IR_PIN,
                  gpioModeInputPull,
                  1);

  // Включаем прерывание на Echo пине
  GPIO_ExtIntConfig(USS_ECHO_PORT,
                    USS_ECHO_PIN,
                    USS_ECHO_PIN,
                    true,
                    true,
                    true);
  // Регистрация callback-функции для Echo
  GPIOINT_CallbackRegister(USS_ECHO_PIN,
                           gpio_callback);

  // Включаем прерывание на Echo пине
  GPIO_ExtIntConfig(IR_PORT,
                    IR_PIN,
                    IR_PIN,
                    true,
                    true,
                    true);
  GPIOINT_CallbackRegister(IR_PIN,
                           gpio_callback);
}

// Обработчик прерываний GPIO
void gpio_callback(uint8_t pin)
{
  if (ble_proximity_get_state() > BLE_PROXIMITY_MODULE_STATE_NOT_INIT) {
    if (pin == USS_ECHO_PIN) {
      if (!ble_proximity_get_uss_echo_flag()) {
        // Начало сигнала Echo
        ble_proximity_set_uss_start_time(sl_sleeptimer_get_tick_count());
      } else {
        // Конец сигнала Echo
        ble_proximity_set_uss_end_time(sl_sleeptimer_get_tick_count());
      }
    }
    if (pin == IR_PIN) {
      ble_proximity_set_ir_flag(true);
      ble_proximity_set_ir_state((bool) !GPIO_PinInGet(IR_PORT,
                                                       IR_PIN));
    }
  }
}

void uart_init_dma(void)
{
  // Инициализация DMADRV
  DMADRV_Init();

  // Запросить канал DMA
  DMADRV_AllocateChannel(&uart_dma_channel_rx,
                         NULL);
  DMADRV_AllocateChannel(&uart_dma_channel_tx,
                         NULL);
}

void uart_dma_send(void)
{
  // Передача данных через DMA
  DMADRV_MemoryPeripheral(uart_dma_channel_tx,
                          dmadrvPeripheralSignal_USART0_TXBL,
                          (void*) &(USART0->TXDATA),
                          (void*) ble_proximity_handle.uart.tx_buff,
                          true, // Указание на инкремент адреса
                          BLE_PROXIMITY_UART_PACK_SIZE,
                          dmadrvDataSize1, // Размер данных 1 байт
                          ble_proximity_uart_write_finish,
                          NULL); // Коллбек (необязательно)
}

void uart_dma_receive(void)
{
  // Ожидание приема данных через DMA с callback-функцией
  DMADRV_PeripheralMemory(uart_dma_channel_rx,
                          dmadrvPeripheralSignal_USART0_RXDATAV,
                          (void*) &ble_proximity_handle.uart.rx_buff,
                          (void*) &(USART0->RXDATA),
                          true,
                          BLE_PROXIMITY_UART_PACK_SIZE,
                          dmadrvDataSize1,
                          ble_proximity_uart_read_finish, // Callback для окончания приема
                          NULL);
}

void timer_periodic_init(void)
{
  // Запускаем периодический таймер 1000 миллисекунд
  app_timer_start(&periodic_timer,
                  TIMER_PERIODIC_PERIOD,
                  timer_periodic_callback,
                  NULL,
                  true);
}

void timer_periodic_callback(app_timer_t *timer,
                             void *data)
{
  (void) data;
  (void) timer;
  periodic_timer_flag = true;
}

// Запуск триггерного импульса с использованием app_timer
void timer_trigger_pulse_init(void)
{
  GPIO_PinOutSet(USS_TRIGGER_PORT,
                 USS_TRIGGER_PIN);

  // Запускаем таймер для отключения Trigger через 10 микросекунд
  app_timer_start(&trigger_pulse_timer,
                  TIMER_TRIGGER_PERIOD,
                  timer_trigger_pulse_callback,
                  NULL,
                  false);
}

// Функция завершения триггерного импульса
void timer_trigger_pulse_callback(app_timer_t *timer,
                                  void *data)
{
  (void) data;
  (void) timer;
  GPIO_PinOutClear(USS_TRIGGER_PORT,
                   USS_TRIGGER_PIN);
}

void handle_device_states(void)
{
  sl_status_t sc;

  if (ble_proximity_get_uss_start_req()) {
    ble_proximity_set_uss_start_req(false);
    timer_trigger_pulse_init();
  }

  if (ble_proximity_get_uss_flag()) {
    ble_proximity_set_uss_flag(false);

    sc = uss_distance_update_characteristic();

    if (sc == SL_STATUS_OK) {
      sc = uss_distance_send_notification();
    }

    ble_proximity_send_uss_distance(ble_proximity_get_uss_distance());
  }

  if (ble_proximity_get_ir_flag()) {
    ble_proximity_set_ir_flag(false);

    sc = ir_sensor_update_characteristic();

    if (sc == SL_STATUS_OK) {
      sc = ir_sensor_send_notification();
    }

    ble_proximity_send_ir_state(ble_proximity_get_ir_state());
  }

  if (ble_proximity_uart_get_req(BLE_PROXIMITY_UART_DEST_TX)) {
    ble_proximity_uart_set_req(BLE_PROXIMITY_UART_DEST_TX,
                               false);
    uart_dma_send();
  }
  if (!ble_proximity_uart_get_state(BLE_PROXIMITY_UART_DEST_RX)) {
    uart_dma_receive();
    ble_proximity_uart_set_state(BLE_PROXIMITY_UART_DEST_RX,
                                 true);
  }
}

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  ble_proximity_init(&ble_proximity_handle,
                     true,
                     true,
                     true);
  gpio_init();
  uart_init_dma();
  timer_periodic_init();
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  if (periodic_timer_flag) {
    periodic_timer_flag = false;
    ble_proximity_set_timer_tick();
  }

  ble_proximity_handler();
  handle_device_states();
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(advertising_set_handle,
                                       160, // min. adv. interval (milliseconds * 1.6)
                                       160, // max. adv. interval (milliseconds * 1.6)
                                       0,   // adv. duration
                                       0);  // max. num. adv. events
      app_assert_status(sc);
      // Start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);

      sc = ir_sensor_update_characteristic();
      sc = uss_distance_update_characteristic();

      ble_proximity_set_state(BLE_PROXIMITY_MODULE_STATE_ADVERT);

      ble_proximity_send_state();
      break;

    case sl_bt_evt_connection_opened_id:
      ble_proximity_set_connected(true);
      break;

    case sl_bt_evt_connection_closed_id:
      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);

      ble_proximity_set_connected(false);
      break;

      // This event occurs when the remote device enabled or disabled the
      // notification.
    case sl_bt_evt_gatt_server_characteristic_status_id:
      if (gattdb_uss_distance == evt->data.evt_gatt_server_characteristic_status.characteristic) {
        if (evt->data.evt_gatt_server_characteristic_status.client_config_flags & sl_bt_gatt_notification) {

          sc = uss_distance_send_notification();
        }
      }
      if (gattdb_ir_sensor_state == evt->data.evt_gatt_server_characteristic_status.characteristic) {
        if (evt->data.evt_gatt_server_characteristic_status.client_config_flags & sl_bt_gatt_notification) {

          sc = ir_sensor_send_notification();
        }
      }
      break;

    default:
      break;
  }
}

sl_status_t ir_sensor_update_characteristic(void)
{
  sl_status_t sc;
  uint8_t data_send;

  data_send = (uint8_t) ble_proximity_get_ir_state();

  sc = sl_bt_gatt_server_write_attribute_value(gattdb_ir_sensor_state,
                                               0,
                                               sizeof(data_send),
                                               &data_send);
  return sc;
}

sl_status_t ir_sensor_send_notification(void)
{
  sl_status_t sc;
  uint8_t data_send;
  size_t data_len;

  sc = sl_bt_gatt_server_read_attribute_value(gattdb_ir_sensor_state,
                                              0,
                                              sizeof(data_send),
                                              &data_len,
                                              &data_send);
  if (sc != SL_STATUS_OK) {
    return sc;
  }

  sc = sl_bt_gatt_server_notify_all(gattdb_ir_sensor_state,
                                    sizeof(data_send),
                                    &data_send);
  return sc;
}

sl_status_t uss_distance_update_characteristic(void)
{
  sl_status_t sc;
  float distance = ble_proximity_get_uss_distance();
  uint8_t data_send[2];

  uint8_t int_part = (uint8_t) distance;  // Целая часть
  uint8_t frac_part = (uint8_t) ((distance - int_part) * pow(10,
                                                             2)); // Дробная часть

  data_send[0] = int_part;
  data_send[1] = frac_part;

  sc = sl_bt_gatt_server_write_attribute_value(gattdb_uss_distance,
                                               0,
                                               sizeof(data_send),
                                               data_send);
  return sc;
}

sl_status_t uss_distance_send_notification(void)
{
  sl_status_t sc;
  uint8_t data_send[2];
  size_t data_len;

  sc = sl_bt_gatt_server_read_attribute_value(gattdb_uss_distance,
                                              0,
                                              sizeof(data_send),
                                              &data_len,
                                              data_send);
  if (sc != SL_STATUS_OK) {
    return sc;
  }

  sc = sl_bt_gatt_server_notify_all(gattdb_uss_distance,
                                    sizeof(data_send),
                                    data_send);
  return sc;
}
