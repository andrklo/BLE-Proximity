#ifndef BLE_PROXIMITY_H_
#define BLE_PROXIMITY_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define BLE_PROXIMITY_UART_PREAMBLE                0x45      // Преамбула

#define BLE_PROXIMITY_UART_PACK_POS_PREAMBLE       0
#define BLE_PROXIMITY_UART_PACK_POS_CMD            1
#define BLE_PROXIMITY_UART_PACK_POS_DATA           2
#define BLE_PROXIMITY_UART_PACK_POS_CRC            7

#define BLE_PROXIMITY_UART_QUEUE_SIZE              5        // Размер очереди пакетов
#define BLE_PROXIMITY_UART_PACK_SIZE               8        // Размер пакета
#define BLE_PROXIMITY_UART_PACK_DATA_SIZE          5        // Количество байт данных

#define BLE_PROXIMITY_UART_DEST_RX  false
#define BLE_PROXIMITY_UART_DEST_TX  true

typedef enum {
  BLE_PROXIMITY_CMD_MODULE_STATUS = 0x01,
  BLE_PROXIMITY_CMD_DISTANCE_DATA = 0x02,
  BLE_PROXIMITY_CMD_IR_DATA = 0x03
} ble_proximity_cmd_t;

typedef enum {
  BLE_PROXIMITY_MODULE_STATUS_UNKNOWN = 0x00,  // Статус радиомодуля не известен
  BLE_PROXIMITY_MODULE_STATUS_NOT_INIT = 0x01, // Радиомодуль не инициализирован
  BLE_PROXIMITY_MODULE_STATUS_ADVERT = 0x02,      // Радиомодуль в режиме поиска
  BLE_PROXIMITY_MODULE_STATUS_CONNECTED = 0x03,         // Радиомодуль подключен
  BLE_PROXIMITY_MODULE_STATUS_LOST = 0xFF         // Ошибка в работе радиомодуля
} ble_proximity_module_status_t;

typedef struct {
  ble_proximity_module_status_t status;
  uint8_t radio_id;
  bool connected;
  bool type_changed;
  bool status_changed;
} ble_proximity_module_struct_t;

typedef struct {
  bool status;
  uint8_t cmd;
  uint8_t data[BLE_PROXIMITY_UART_PACK_DATA_SIZE];
  uint8_t crc;
} ble_proximity_uart_pack_t;

typedef struct {
  int front;
  int rear;
  int size;
  ble_proximity_uart_pack_t pack[BLE_PROXIMITY_UART_QUEUE_SIZE];
} ble_proximity_uart_queue_t;

typedef struct {
  ble_proximity_uart_queue_t rx_queue;
  ble_proximity_uart_queue_t tx_queue;
  uint8_t rx_current_pack;
  uint8_t tx_current_pack;
  uint8_t rx_buff[BLE_PROXIMITY_UART_PACK_SIZE];
  uint8_t tx_buff[BLE_PROXIMITY_UART_PACK_SIZE];
  bool rx_active;
  bool tx_active;
  bool rx_req;
  bool tx_req;
} ble_proximity_uart_handle_t;

typedef struct {
  float distance;
  bool start_req;
  bool sensor_flag;
  bool echo_flag;
  uint32_t start_time;
  uint32_t end_time;
} ble_proximity_uss_handle_t;

typedef struct {
  bool sensor_state;
  bool sensor_flag;
} ble_proximity_ir_handle_t;

typedef struct {
  ble_proximity_module_struct_t module;
  ble_proximity_uart_handle_t uart;
  ble_proximity_uss_handle_t uss;
  ble_proximity_ir_handle_t ir;
  bool timer_tick;
} ble_proximity_handle_t;

void ble_proximity_init(ble_proximity_handle_t *handle);
void ble_proximity_handler(void);

void ble_proximity_set_timer_tick(void);

void ble_proximity_set_state(ble_proximity_module_status_t state);
ble_proximity_module_status_t ble_proximity_get_state(void);

bool ble_proximity_get_ir_flag(void);
void ble_proximity_set_ir_flag(bool flag);
bool ble_proximity_get_ir_state(void);
void ble_proximity_set_ir_state(bool state);

bool ble_proximity_get_uss_start_req(void);
void ble_proximity_set_uss_start_req(bool flag);
bool ble_proximity_get_uss_flag(void);
void ble_proximity_set_uss_flag(bool flag);
bool ble_proximity_get_uss_echo_flag(void);
void ble_proximity_set_uss_echo_flag(bool flag);
void ble_proximity_set_uss_start_time(uint32_t time);
void ble_proximity_set_uss_end_time(uint32_t time);
float ble_proximity_get_uss_distance(void);

void ble_proximity_uss_distance_calc(void);

void ble_proximity_uart_set_state(bool dest, bool state);
bool ble_proximity_uart_get_state(bool dest);
void ble_proximity_uart_set_req(bool dest, bool state);
bool ble_proximity_uart_get_req(bool dest);

void ble_proximity_send_state(void);
void ble_proximity_send_uss_distance(float distance);
void ble_proximity_send_ir_state(bool active);

void ble_proximity_uart_write_data(ble_proximity_cmd_t cmd,
                                   uint8_t *data,
                                   uint8_t data_len);
bool ble_proximity_uart_read_finish(unsigned int channel,
                                    unsigned int sequenceNo,
                                    void *userParam);
bool ble_proximity_uart_write_finish(unsigned int channel,
                                     unsigned int sequenceNo,
                                     void *userParam);

void ble_proximity_handle_uart_pack(ble_proximity_uart_pack_t *uart_pack);

bool ble_proximity_chk_queue_empty(ble_proximity_uart_queue_t *queue);
bool ble_proximity_chk_queue_full(ble_proximity_uart_queue_t *queue);
bool ble_proximity_add_in_queue(ble_proximity_uart_queue_t *queue,
                                ble_proximity_uart_pack_t item);
bool ble_proximity_del_from_queue(ble_proximity_uart_queue_t *queue,
                                  ble_proximity_uart_pack_t *item);
void ble_proximity_chk_queue(ble_proximity_uart_queue_t *queue);

void ble_proximity_rx_queue_handler(void);
void ble_proximity_tx_queue_handler(void);

uint8_t ble_proximity_calc_crc(uint8_t *data, uint8_t data_len);

#endif /* BLE_PROXIMITY_H_ */
