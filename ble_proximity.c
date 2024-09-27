#include <ble_proximity.h>

static ble_proximity_handle_t *ble_proximity_handle;

static void ble_proximity_rx_queue_handler();
static void ble_proximity_tx_queue_handler();
static void ble_proximity_uart_write_data(ble_proximity_cmd_t cmd,
                                          uint8_t *data,
                                          uint8_t data_len);
static void ble_proximity_handle_uart_pack(ble_proximity_uart_pack_t *uart_pack);
static bool ble_proximity_chk_queue_empty(ble_proximity_uart_queue_t *queue);
static bool ble_proximity_chk_queue_full(ble_proximity_uart_queue_t *queue);
static bool ble_proximity_add_in_queue(ble_proximity_uart_queue_t *queue,
                                       ble_proximity_uart_pack_t item);
static bool ble_proximity_del_from_queue(ble_proximity_uart_queue_t *queue,
                                         ble_proximity_uart_pack_t *item);
static void ble_proximity_chk_queue(ble_proximity_uart_queue_t *queue);

static void ble_proximity_uss_distance_calc(void);
static void ble_proximity_uart_write_data(ble_proximity_cmd_t cmd,
                                   uint8_t *data,
                                   uint8_t data_len);
static uint8_t ble_proximity_calc_crc(uint8_t *data,
                                      uint8_t data_len);

//////////////////////////////////////////////////////////////////////////////////

void ble_proximity_init(ble_proximity_handle_t *handle,
                        uint16_t timer_period,
                        bool uart_enable,
                        bool uss_enable,
                        bool ir_enable)
{
  if (handle == NULL) {
    return;
  }

  ble_proximity_handle = handle;
  ble_proximity_handle->module.state = BLE_PROXIMITY_MODULE_STATE_NOT_INIT;
  ble_proximity_handle->module.connected = false;
  ble_proximity_handle->timer_period = timer_period;
  ble_proximity_handle->timer_tick = false;

  ble_proximity_handle->uss.enable = uss_enable;
  ble_proximity_handle->uss.flag = false;
  ble_proximity_handle->uss.distance = 0;
  ble_proximity_handle->uss.distance_trig = 0;
  ble_proximity_handle->uss.period = timer_period;
  ble_proximity_handle->uss.timer_cnt = 0;
  ble_proximity_handle->uss.start_req = false;
  ble_proximity_handle->uss.echo_flag = false;
  ble_proximity_handle->uss.start_time = 0;
  ble_proximity_handle->uss.end_time = 0;

  ble_proximity_handle->ir.enable = ir_enable;
  ble_proximity_handle->ir.flag = false;
  ble_proximity_handle->ir.state = false;

  ble_proximity_handle->uart.enable = uart_enable;
  ble_proximity_handle->uart.rx_queue.front = 0;
  ble_proximity_handle->uart.rx_queue.rear = -1;
  ble_proximity_handle->uart.rx_queue.size = 0;
  ble_proximity_handle->uart.tx_queue.front = 0;
  ble_proximity_handle->uart.tx_queue.rear = -1;
  ble_proximity_handle->uart.tx_queue.size = 0;
  ble_proximity_handle->uart.rx_current_pack = 0;
  ble_proximity_handle->uart.tx_current_pack = 0;
  for(int i = 0; i < BLE_PROXIMITY_UART_PACK_SIZE; i++) {
    ble_proximity_handle->uart.rx_buff[i] = 0;
    ble_proximity_handle->uart.tx_buff[i] = 0;
  }
  ble_proximity_handle->uart.rx_active = false;
  ble_proximity_handle->uart.tx_active = false;
  ble_proximity_handle->uart.rx_req = false;
  ble_proximity_handle->uart.tx_req = false;
}

void ble_proximity_handler()
{
  if (ble_proximity_handle == NULL) {
    return;
  }
  if (ble_proximity_handle->timer_tick) {
    ble_proximity_handle->timer_tick = false;
    if (ble_proximity_handle->module.state > BLE_PROXIMITY_MODULE_STATE_NOT_INIT) {
      if (ble_proximity_handle->uss.enable) {
        ble_proximity_handle->uss.timer_cnt += ble_proximity_handle->timer_period;
        if (ble_proximity_handle->uss.timer_cnt >= ble_proximity_handle->uss.period) {
          ble_proximity_handle->uss.start_req = true;
        }
      }
    } else {
      ble_proximity_handle->module.state = BLE_PROXIMITY_MODULE_STATE_ADVERT;
    }
  }
  if (ble_proximity_handle->uss.flag) {
    ble_proximity_uss_distance_calc();
  }
  if (ble_proximity_handle->uart.enable) {
    ble_proximity_rx_queue_handler();
    ble_proximity_tx_queue_handler();
  }
}

void ble_proximity_set_timer_period(uint16_t period)
{
  if (ble_proximity_handle != NULL) {
    ble_proximity_handle->timer_period = period;
  }
}

void ble_proximity_set_timer_tick(void)
{
  if (ble_proximity_handle != NULL) {
    ble_proximity_handle->timer_tick = true;
  }
}

void ble_proximity_set_state(ble_proximity_module_state_t state)
{
  if (ble_proximity_handle != NULL) {
    ble_proximity_handle->module.state = state;
  }
}

ble_proximity_module_state_t ble_proximity_get_state(void)
{
  if (ble_proximity_handle != NULL) {
    return ble_proximity_handle->module.state;
  }
  return false;
}

void ble_proximity_set_connected(bool state)
{
  if (ble_proximity_handle != NULL) {
    ble_proximity_handle->module.connected = state;
    state ?
        (ble_proximity_handle->module.state = BLE_PROXIMITY_MODULE_STATE_CONNECTED) :
        (ble_proximity_handle->module.state = BLE_PROXIMITY_MODULE_STATE_ADVERT);
  }
}

bool ble_proximity_get_connected(void)
{
  if (ble_proximity_handle != NULL) {
    return ble_proximity_handle->module.connected;
  }
  return false;
}

bool ble_proximity_get_ir_enable(void)
{
  if (ble_proximity_handle != NULL) {
    return ble_proximity_handle->ir.enable;
  }
  return false;
}

void ble_proximity_set_ir_enable(bool enable)
{
  if (ble_proximity_handle != NULL) {
    ble_proximity_handle->ir.enable = enable;
  }
}

bool ble_proximity_get_ir_flag(void)
{
  if (ble_proximity_handle != NULL) {
    return ble_proximity_handle->ir.flag;
  }
  return false;
}

void ble_proximity_set_ir_flag(bool flag)
{
  if (ble_proximity_handle != NULL) {
    ble_proximity_handle->ir.flag = flag;
  }
}

bool ble_proximity_get_ir_state(void)
{
  if (ble_proximity_handle != NULL) {
    return ble_proximity_handle->ir.state;
  }
  return false;
}

void ble_proximity_set_ir_state(bool state)
{
  if (ble_proximity_handle != NULL) {
    ble_proximity_handle->ir.state = state;
  }
}

bool ble_proximity_get_uss_enable(void)
{
  if (ble_proximity_handle != NULL) {
    return ble_proximity_handle->uss.enable;
  }
  return false;
}

void ble_proximity_set_uss_enable(bool enable)
{
  if (ble_proximity_handle != NULL) {
    ble_proximity_handle->uss.enable = enable;
  }
}

bool ble_proximity_get_uss_start_req(void)
{
  if (ble_proximity_handle != NULL) {
    return ble_proximity_handle->uss.start_req;
  }
  return false;
}

void ble_proximity_set_uss_start_req(bool flag)
{
  if (ble_proximity_handle != NULL) {
    ble_proximity_handle->uss.start_req = flag;
  }

}

bool ble_proximity_get_uss_flag(void)
{
  if (ble_proximity_handle != NULL) {
    return ble_proximity_handle->uss.flag;
  }
  return false;
}

void ble_proximity_set_uss_flag(bool flag)
{
  if (ble_proximity_handle != NULL) {
    ble_proximity_handle->uss.flag = flag;
  }

}

bool ble_proximity_get_uss_echo_flag(void)
{
  if (ble_proximity_handle != NULL) {
    return ble_proximity_handle->uss.echo_flag;
  }
  return false;
}

void ble_proximity_set_uss_echo_flag(bool flag)
{
  if (ble_proximity_handle != NULL) {
    ble_proximity_handle->uss.echo_flag = flag;
  }
}

void ble_proximity_set_uss_start_time(uint32_t time)
{
  if (ble_proximity_handle != NULL && ble_proximity_handle->uss.enable) {
    ble_proximity_handle->uss.start_time = time;
    ble_proximity_handle->uss.echo_flag = true;
  }
}

void ble_proximity_set_uss_end_time(uint32_t time)
{
  if (ble_proximity_handle != NULL && ble_proximity_handle->uss.enable) {
    ble_proximity_handle->uss.end_time = time;
    ble_proximity_handle->uss.flag = true;
    ble_proximity_handle->uss.echo_flag = false;
  }
}

float ble_proximity_get_uss_distance(void)
{
  if (ble_proximity_handle != NULL) {
    return ble_proximity_handle->uss.distance;
  }
  return 0;
}

static void ble_proximity_uss_distance_calc(void)
{
  if (ble_proximity_handle->uss.enable) {
    // Вычисляем количество тиков
    uint32_t ticks = ble_proximity_handle->uss.end_time - ble_proximity_handle->uss.start_time;

    // Переводим тики в секунды
    float time_in_seconds = (float) ticks / 32768;

    // Рассчитываем расстояние (делим на 2, так как звук идет туда и обратно)
    ble_proximity_handle->uss.distance = ((time_in_seconds * 34300) / 2) / 100;
  }
}

bool ble_proximity_get_uart_enable(void)
{
  if (ble_proximity_handle != NULL) {
    return ble_proximity_handle->uart.enable;
  }
  return false;
}
void ble_proximity_set_uart_enable(bool enable)
{
  if (ble_proximity_handle != NULL) {
    ble_proximity_handle->uart.enable = enable;
  }
}

// dest: false - rx, true - tx
void ble_proximity_uart_set_state(bool dest,
                                  bool state)
{
  if (ble_proximity_handle != NULL) {
    dest ? (ble_proximity_handle->uart.tx_active = state) : (ble_proximity_handle->uart.rx_active = state);
  }
}

// dest: false - rx, true - tx
bool ble_proximity_uart_get_state(bool dest)
{
  if (ble_proximity_handle != NULL) {
    return dest ? ble_proximity_handle->uart.tx_active : ble_proximity_handle->uart.rx_active;
  }
  return false;
}

// dest: false - rx, true - tx
void ble_proximity_uart_set_req(bool dest,
                                bool state)
{
  if (ble_proximity_handle != NULL) {
    dest ? (ble_proximity_handle->uart.tx_req = state) : (ble_proximity_handle->uart.rx_req = state);
  }
}

// dest: false - rx, true - tx
bool ble_proximity_uart_get_req(bool dest)
{
  if (ble_proximity_handle != NULL) {
    return dest ? ble_proximity_handle->uart.tx_req : ble_proximity_handle->uart.rx_req;
  }
  return false;
}

void ble_proximity_send_state(void)
{
  if (ble_proximity_handle == NULL) {
    return;
  }

  uint8_t data[1] = {ble_proximity_handle->module.state};

  ble_proximity_uart_write_data(BLE_PROXIMITY_CMD_MODULE_STATE,
                                data,
                                sizeof(data));
}

void ble_proximity_send_uss_distance(float distance)
{
  if (ble_proximity_handle == NULL) {
    return;
  }

  int int_part = (int) distance;  // Целая часть
  int frac_part = (int) ((distance - int_part) * pow(10,
                                                     2));  // Дробная часть

  uint8_t data[2];
  data[0] = int_part;
  data[1] = frac_part;

  ble_proximity_uart_write_data(BLE_PROXIMITY_CMD_DISTANCE_DATA,
                                data,
                                sizeof(data));
}

void ble_proximity_send_ir_state(bool active)
{
  if (ble_proximity_handle == NULL) {
    return;
  }

  uint8_t data[1];
  data[0] = active;

  ble_proximity_uart_write_data(BLE_PROXIMITY_CMD_IR_DATA,
                                data,
                                sizeof(data));
}

static void ble_proximity_uart_write_data(ble_proximity_cmd_t cmd,
                                          uint8_t *data,
                                          uint8_t data_len)
{
  if (!ble_proximity_chk_queue_full(&ble_proximity_handle->uart.tx_queue)) {
    ble_proximity_uart_pack_t tx_packet;
    uint8_t tx_data[BLE_PROXIMITY_UART_PACK_SIZE];
    tx_data[BLE_PROXIMITY_UART_PACK_POS_PREAMBLE] = BLE_PROXIMITY_UART_PREAMBLE;
    tx_data[BLE_PROXIMITY_UART_PACK_POS_CMD] = tx_packet.cmd = cmd;
    for(int i = 0; i < data_len; i++) {
      tx_data[i + 2] = tx_packet.data[i] = data[i];
    }
    tx_packet.crc = ble_proximity_calc_crc(tx_data,
                                           BLE_PROXIMITY_UART_PACK_SIZE - 2); // CRC8

    tx_packet.status = true;

    ble_proximity_add_in_queue(&ble_proximity_handle->uart.tx_queue,
                               tx_packet);
  }
}

bool ble_proximity_uart_read_finish(unsigned int channel,
                                    unsigned int sequenceNo,
                                    void *userParam)
{
  (void) channel;
  (void) sequenceNo;
  (void) userParam;

  if (ble_proximity_handle == NULL) {
    return false;
  }

  if (ble_proximity_handle->uart.rx_buff[BLE_PROXIMITY_UART_PACK_POS_PREAMBLE] == BLE_PROXIMITY_UART_PREAMBLE) {
    if (ble_proximity_handle->uart.rx_buff[BLE_PROXIMITY_UART_PACK_POS_CRC]
        == ble_proximity_calc_crc(ble_proximity_handle->uart.rx_buff,
                                  BLE_PROXIMITY_UART_PACK_SIZE - 2)) {
      if (!ble_proximity_chk_queue_full(&ble_proximity_handle->uart.rx_queue)) {
        ble_proximity_uart_pack_t rx_packet;
        rx_packet.status = true;
        rx_packet.cmd = ble_proximity_handle->uart.rx_buff[BLE_PROXIMITY_UART_PACK_POS_CMD];
        for(int i = 0; i < BLE_PROXIMITY_UART_PACK_DATA_SIZE; i++) {
          rx_packet.data[i] = ble_proximity_handle->uart.rx_buff[i + 2];
        }
        rx_packet.crc = ble_proximity_handle->uart.rx_buff[BLE_PROXIMITY_UART_PACK_POS_CRC];
        ble_proximity_add_in_queue(&ble_proximity_handle->uart.rx_queue,
                                   rx_packet);
        ble_proximity_handle->uart.rx_req = true;
      }
    }
  }
  ble_proximity_handle->uart.rx_active = false;
  return true;
}

bool ble_proximity_uart_write_finish(unsigned int channel,
                                     unsigned int sequenceNo,
                                     void *userParam)
{
  (void) channel;
  (void) sequenceNo;
  (void) userParam;

  if (ble_proximity_handle == NULL) {
    return false;
  }

  /*
  ble_proximity_del_from_queue(&ble_proximity_handle->uart.tx_queue,
                               &ble_proximity_handle->uart.tx_queue.pack[ble_proximity_handle->uart.tx_current_pack]);
  */
  ble_proximity_handle->uart.tx_queue.pack[ble_proximity_handle->uart.tx_current_pack].status =
  false;
  ble_proximity_handle->uart.tx_active = false;
  return true;
}

static void ble_proximity_handle_uart_pack(ble_proximity_uart_pack_t *uart_pack)
{
  switch (uart_pack->cmd) {
    default:
      break;
  }
}

static void ble_proximity_rx_queue_handler()
{
  if (ble_proximity_chk_queue_empty(&ble_proximity_handle->uart.rx_queue)) {
    return;  // Очередь пуста, нечего обходить
  }

  ble_proximity_chk_queue(&ble_proximity_handle->uart.rx_queue);

  int index = ble_proximity_handle->uart.rx_queue.front;
  for(int i = 0; i < ble_proximity_handle->uart.rx_queue.size; i++) {
    if (ble_proximity_handle->uart.rx_queue.pack[index].status) {
      ble_proximity_handle_uart_pack(&ble_proximity_handle->uart.rx_queue.pack[index]);
    }
    index = (index + 1) % BLE_PROXIMITY_UART_QUEUE_SIZE;
  }
}

static void ble_proximity_tx_queue_handler()
{
  if (ble_proximity_chk_queue_empty(&ble_proximity_handle->uart.tx_queue)) {
    return;  // Очередь пуста, нечего обходить
  }

  ble_proximity_chk_queue(&ble_proximity_handle->uart.tx_queue);

  int index = ble_proximity_handle->uart.tx_queue.front;
  for(int i = 0; i < ble_proximity_handle->uart.tx_queue.size; i++) {
    if (ble_proximity_handle->uart.tx_queue.pack[index].status) {
      if (ble_proximity_handle->uart.tx_active != true) {
        ble_proximity_handle->uart.tx_active = true;
        ble_proximity_handle->uart.tx_req = true;
        ble_proximity_handle->uart.tx_current_pack = index;
        ble_proximity_handle->uart.tx_buff[BLE_PROXIMITY_UART_PACK_POS_PREAMBLE] =
        BLE_PROXIMITY_UART_PREAMBLE;
        ble_proximity_handle->uart.tx_buff[BLE_PROXIMITY_UART_PACK_POS_CMD] =
            ble_proximity_handle->uart.tx_queue.pack[index].cmd;
        for(int j = 0; j < BLE_PROXIMITY_UART_PACK_DATA_SIZE; j++) {
          ble_proximity_handle->uart.tx_buff[j + 2] = ble_proximity_handle->uart.tx_queue.pack[index].data[j];
        }
        ble_proximity_handle->uart.tx_buff[BLE_PROXIMITY_UART_PACK_POS_CRC] =
            ble_proximity_handle->uart.tx_queue.pack[index].crc;
      }
    }
    index = (index + 1) % BLE_PROXIMITY_UART_QUEUE_SIZE;
  }
}

static bool ble_proximity_chk_queue_empty(ble_proximity_uart_queue_t *queue)
{
  return (queue->size == 0);
}

static bool ble_proximity_chk_queue_full(ble_proximity_uart_queue_t *queue)
{
  return (queue->size == BLE_PROXIMITY_UART_QUEUE_SIZE);
}

static bool ble_proximity_add_in_queue(ble_proximity_uart_queue_t *queue,
                                       ble_proximity_uart_pack_t pack)
{
  if (ble_proximity_chk_queue_full(queue)) {
    return false;  // Очередь полна, не можем добавить элемент
  } else {
    queue->rear = (queue->rear + 1) % BLE_PROXIMITY_UART_QUEUE_SIZE;
    queue->pack[queue->rear] = pack;
    queue->size++;
    return true;
  }
}

static bool ble_proximity_del_from_queue(ble_proximity_uart_queue_t *queue,
                                         ble_proximity_uart_pack_t *pack)
{
  if (ble_proximity_chk_queue_empty(queue)) {
    return false;  // Очередь пуста, нет элементов для удаления
  } else {
    *pack = queue->pack[queue->front];
    queue->front = (queue->front + 1) % BLE_PROXIMITY_UART_QUEUE_SIZE;
    queue->size--;
    return true;
  }
}

static void ble_proximity_chk_queue(ble_proximity_uart_queue_t *queue)
{
  if (ble_proximity_chk_queue_empty(queue)) {
    return;  // Очередь пуста, нечего обходить
  }

  int index = queue->front;
  int newSize = queue->size;
  for(int i = 0; i < queue->size; i++) {
    if (queue->pack[i].status == 0) {
      // Сдвигаем все последующие элементы на одну позицию назад
      for(int j = index; j != queue->rear; j = (j + 1) % BLE_PROXIMITY_UART_QUEUE_SIZE) {
        queue->pack[j] = queue->pack[(j + 1) % BLE_PROXIMITY_UART_QUEUE_SIZE];
      }
      queue->rear = (queue->rear - 1 + BLE_PROXIMITY_UART_QUEUE_SIZE) % BLE_PROXIMITY_UART_QUEUE_SIZE;
      newSize--;

      // После удаления элемента, сдвиг продолжается с текущего индекса
      index = (index - 1 + BLE_PROXIMITY_UART_QUEUE_SIZE) % BLE_PROXIMITY_UART_QUEUE_SIZE;
    }
    index = (index + 1) % BLE_PROXIMITY_UART_QUEUE_SIZE;
  }
  queue->size = newSize;
}

static uint8_t ble_proximity_calc_crc(uint8_t *data,
                                      uint8_t data_len)
{
  uint8_t crc = 0x00;
  for(uint8_t i = 0; i < data_len; i++) {
    crc ^= data[i];
    for(uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;  // Polynomial x^8 + x^2 + x^1 + x^0
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}
