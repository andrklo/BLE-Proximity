#ifndef BLE_PROXIMITY_H_
#define BLE_PROXIMITY_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define BLE_PROXIMITY_UART_PREAMBLE                 0x45      // Преамбула

#define BLE_PROXIMITY_UART_PACK_POS_PREAMBLE        0
#define BLE_PROXIMITY_UART_PACK_POS_CMD             1
#define BLE_PROXIMITY_UART_PACK_POS_DATA            2
#define BLE_PROXIMITY_UART_PACK_POS_CRC             7

#define BLE_PROXIMITY_UART_QUEUE_SIZE               5        // Размер очереди пакетов
#define BLE_PROXIMITY_UART_PACK_SIZE                8        // Размер пакета
#define BLE_PROXIMITY_UART_PACK_DATA_SIZE           5        // Количество байт данных

#define BLE_PROXIMITY_UART_DEST_RX                  false
#define BLE_PROXIMITY_UART_DEST_TX                  true

typedef enum
{
  BLE_PROXIMITY_CMD_MODULE_STATE = 0x01,
  BLE_PROXIMITY_CMD_DISTANCE_DATA = 0x02,
  BLE_PROXIMITY_CMD_IR_DATA = 0x03
} ble_proximity_cmd_t;

typedef enum
{
  BLE_PROXIMITY_MODULE_STATE_UNKNOWN = 0x00,     // Статус радиомодуля не известен
  BLE_PROXIMITY_MODULE_STATE_NOT_INIT = 0x01,    // Радиомодуль не инициализирован
  BLE_PROXIMITY_MODULE_STATE_ADVERT = 0x02,      // Радиомодуль в режиме поиска
  BLE_PROXIMITY_MODULE_STATE_CONNECTED = 0x03,   // Радиомодуль подключен
  BLE_PROXIMITY_MODULE_STATE_LOST = 0xFF         // Ошибка в работе радиомодуля
} ble_proximity_module_state_t;

typedef struct
{
  ble_proximity_module_state_t state;
  bool connected;
} ble_proximity_module_struct_t;

typedef struct
{
  bool status;
  uint8_t cmd;
  uint8_t data[BLE_PROXIMITY_UART_PACK_DATA_SIZE];
  uint8_t crc;
} ble_proximity_uart_pack_t;

typedef struct
{
  int front;
  int rear;
  int size;
  ble_proximity_uart_pack_t pack[BLE_PROXIMITY_UART_QUEUE_SIZE];
} ble_proximity_uart_queue_t;

typedef struct
{
  bool enable;
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

typedef struct
{
  bool enable;
  bool flag;
  float distance;
  float distance_trig;
  uint16_t period;
  uint16_t timer_cnt;
  bool start_req;
  bool echo_flag;
  uint32_t start_time;
  uint32_t end_time;
} ble_proximity_uss_handle_t;

typedef struct
{
  bool enable;
  bool flag;
  bool state;
} ble_proximity_ir_handle_t;

typedef struct
{
  ble_proximity_module_struct_t module;
  ble_proximity_uart_handle_t uart;
  ble_proximity_uss_handle_t uss;
  ble_proximity_ir_handle_t ir;
  uint16_t timer_period;
  bool timer_tick;
} ble_proximity_handle_t;

void ble_proximity_init(ble_proximity_handle_t *handle,
                        uint16_t timer_period,
                        bool uart_enable,
                        bool uss_enable,
                        bool ir_enable);
void ble_proximity_handler(void);

void ble_proximity_set_timer_period(uint16_t period);
void ble_proximity_set_timer_tick(void);

void ble_proximity_set_state(ble_proximity_module_state_t state);
ble_proximity_module_state_t ble_proximity_get_state(void);
void ble_proximity_set_connected(bool state);
bool ble_proximity_get_connected(void);

bool ble_proximity_get_ir_enable(void);
void ble_proximity_set_ir_enable(bool enable);
bool ble_proximity_get_ir_flag(void);
void ble_proximity_set_ir_flag(bool flag);
bool ble_proximity_get_ir_state(void);
void ble_proximity_set_ir_state(bool state);

bool ble_proximity_get_uss_enable(void);
void ble_proximity_set_uss_enable(bool enable);
bool ble_proximity_get_uss_start_req(void);
void ble_proximity_set_uss_start_req(bool flag);
bool ble_proximity_get_uss_flag(void);
void ble_proximity_set_uss_flag(bool flag);
bool ble_proximity_get_uss_echo_flag(void);
void ble_proximity_set_uss_echo_flag(bool flag);
void ble_proximity_set_uss_start_time(uint32_t time);
void ble_proximity_set_uss_end_time(uint32_t time);
float ble_proximity_get_uss_distance(void);

bool ble_proximity_get_uart_enable(void);
void ble_proximity_set_uart_enable(bool enable);
void ble_proximity_uart_set_state(bool dest,
                                  bool state);
bool ble_proximity_uart_get_state(bool dest);
void ble_proximity_uart_set_req(bool dest,
                                bool state);
bool ble_proximity_uart_get_req(bool dest);

void ble_proximity_send_state(void);
void ble_proximity_send_uss_distance(float distance);
void ble_proximity_send_ir_state(bool active);

bool ble_proximity_uart_read_finish(unsigned int channel,
                                    unsigned int sequenceNo,
                                    void *userParam);
bool ble_proximity_uart_write_finish(unsigned int channel,
                                     unsigned int sequenceNo,
                                     void *userParam);

#endif /* BLE_PROXIMITY_H_ */
