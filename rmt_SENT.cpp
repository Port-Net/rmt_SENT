#include "rmt_SENT.h"

RMT_SENT_RECEIVER::RMT_SENT_RECEIVER(uint8_t pin, uint8_t tick_time_us) : _pin(pin), _tick_time_us(tick_time_us) {
  
#if ESP_ARDUINO_VERSION_MAJOR == 3
  _rx_chan_config = {
    .gpio_num = (gpio_num_t)pin,
    .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
    .resolution_hz = 1000000,
    .mem_block_symbols = RMT_MEM_NUM_BLOCKS_1 * RMT_SYMBOLS_PER_CHANNEL_BLOCK
  };
  _sent_config = {
    .signal_range_min_ns = 3100,     // longest possible?...
    .signal_range_max_ns = 1000U * tick_time_us * 60 // more than 60 ticks is pause (end mark)
  };
  _cbs = {
    .on_recv_done = rmtCallback
  };
  _serial_msg_callback = nullptr;
  _data_callback = nullptr;
  _packet_count = 0;
  _error_count = 0;
  _serial_msg_bit3 = 0;
#endif
}

bool RMT_SENT_RECEIVER::begin() {
#if ESP_ARDUINO_VERSION_MAJOR == 3
  if(rmt_new_rx_channel(&_rx_chan_config, &_sent_chan) != ESP_OK) {
    Serial.println("init RMT chan failed\n");
    return false;
  }
  if(rmt_rx_register_event_callbacks(_sent_chan, &_cbs, this) != ESP_OK) {
    Serial.println("init RMT cb failed\n");
    return false;
  }
  int ret = rmt_enable(_sent_chan);
  if(ret != ESP_OK) {
    Serial.printf("enable RMT chan failed: %d\n", ret);
    return false;
  }
#else
  if ((_rmt_receiver = rmtInit(_pin, RMT_RX_MODE, RMT_MEM_128)) == NULL) {
    Serial.println("init rmt failed\n");
    return false;
  }
  Serial.println("init rmt ok\n");
  rmtSetTick(_rmt_receiver, 1000.0);
  rmtSetRxThreshold(_rmt_receiver, _tick_time_us * 60); // more than 60 ticks is pause (end mark)
  rmtSetFilter(_rmt_receiver, true, 255);
#endif
  _receive_queue = xQueueCreate(10, sizeof(rmt_queue_item_t));
  xTaskCreatePinnedToCore(&handlerThreadStarter, "sent_handler", 4000, this, 5, NULL, ARDUINO_RUNNING_CORE);
  #if ESP_ARDUINO_VERSION_MAJOR == 3
  ret = rmt_receive(_sent_chan, _rmt_data, _rmt_symbols, &_sent_config);
  if(ret != ESP_OK) {
    Serial.printf("init RMT rec failed: %d\n", ret);
    return false;
  }
  #else
  rmtRead(_rmt_receiver, rmtCallback, this);
  #endif
  return true;
}

int RMT_SENT_RECEIVER::getPin() {
  return _pin;
}

#if ESP_ARDUINO_VERSION_MAJOR == 3
IRAM_ATTR bool RMT_SENT_RECEIVER::rmtCallback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data) {
  BaseType_t high_task_wakeup = pdFALSE;
  RMT_SENT_RECEIVER* me = (RMT_SENT_RECEIVER*)user_data;
  rmt_queue_item_t dt;
  bcopy(edata->received_symbols, dt.rmt_data.received_symbols, min(16, (int)edata->num_symbols) * 4);
  dt.rmt_data.num_symbols = edata->num_symbols;
  dt.timestamp = micros();
  xQueueSendFromISR(me->_receive_queue, edata, &high_task_wakeup);
  rmt_receive(me->_sent_chan, me->_rmt_data, me->_rmt_symbols, &me->_sent_config);
  return high_task_wakeup == pdTRUE;
}

#else

IRAM_ATTR void RMT_SENT_RECEIVER::rmtCallback(uint32_t *data, size_t len, void *user_data) {
  RMT_SENT_RECEIVER* me = (RMT_SENT_RECEIVER*)user_data;
  BaseType_t high_task_wakeup = pdFALSE;
  rmt_queue_item_t dt;
  bcopy(data, dt.rmt_data.received_symbols, min(16, (int)len) * 4);
  dt.rmt_data.num_symbols = len;
  dt.timestamp = micros();
  log_d("cb: %d", len);
  if(len >= 10) {
    xQueueSendFromISR(me->_receive_queue, &dt, &high_task_wakeup);
  }
}
#endif

void RMT_SENT_RECEIVER::handlerThreadStarter(void* parameters) {
  RMT_SENT_RECEIVER* me = (RMT_SENT_RECEIVER*)parameters;
  me->handlerThread();
}

void RMT_SENT_RECEIVER::handlerThread() {
  while(1) {
    rmt_queue_item_t dt;
    xQueueReceive(_receive_queue, &dt, portMAX_DELAY);
    if(!decodeSENT(&dt.rmt_data, dt.timestamp)) {
      _error_count++;
      _serial_msg_bit3 = 0;
    }
  }
}

uint8_t RMT_SENT_RECEIVER::calcCRC4(int8_t* nibbles, uint8_t len) {
  uint8_t crc4_table[16] = {0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5};
  uint8_t crc = 5;
  for(int i = 0; i < len; ++i) {
    crc = nibbles[i] ^ crc4_table[crc];
  }
  crc = crc4_table[crc];
  return crc;
}

bool RMT_SENT_RECEIVER::checkCRC6(uint32_t data, uint8_t crc6) {
  uint8_t crc6_table[64] = {0, 25, 50, 43, 61, 36, 15, 22, 35, 58, 17, 8, 30, 7, 44, 53,
                            31, 6, 45, 52, 34, 59, 16, 9, 60, 37, 14, 23, 1, 24, 51, 42,
                            62, 39, 12, 21, 3, 26, 49, 40, 29, 4, 47, 54, 32, 57, 18, 11,
                            33, 56, 19, 10, 28, 5, 46, 55, 2, 27, 48, 41, 63, 38, 13, 20};
  uint8_t crc = 0x15;
  crc = ((data >> 18) & 0x3F) ^ crc6_table[crc];
  crc = ((data >> 12) & 0x3F) ^ crc6_table[crc];
  crc = ((data >> 6) & 0x3F) ^ crc6_table[crc];
  crc = (data & 0x3F) ^ crc6_table[crc];
  crc = crc6_table[crc];
  return crc == crc6;
}

bool RMT_SENT_RECEIVER::decodeSENT(rmt_rx_done_event_data_t* rx_data, uint32_t timestamp) {
  #if ESP_ARDUINO_VERSION_MAJOR == 3
  rmt_symbol_word_t* d = rx_data->received_symbols;
  #else
  rmt_data_t* d = rx_data->received_symbols;
  #endif
  uint16_t ticks = round((float)(d->duration0 + d->duration1) / _tick_time_us);
  if(rx_data->num_symbols < 9) {
    _last_error = short_frame;
    //Serial.println("short frame");
    return false;
  }
  if((ticks < 55) || (ticks > 57)) {
    _last_error = wrong_start;
    //Serial.println("wrong start");
    return false;
  }

  d = &(rx_data->received_symbols[1]);
  _status = round((float)(d->duration0 + d->duration1) / _tick_time_us) - 12;
  if((_status < 0) || (_status > 15)) {
    _last_error = wrong_nibble;
    //Serial.println("wrong nibble1");
    return false;
  }

  for(int i = 0; i < 7; ++i) {
    d = &(rx_data->received_symbols[i + 2]);
    _nibbles[i] = round((float)(d->duration0 + d->duration1) / _tick_time_us) - 12;
    if((_nibbles[i] < 0) || (_nibbles[i] > 15)) {
      _last_error = wrong_nibble;
      //Serial.println("wrong nibble");
      return false;
    }
  }

  if(calcCRC4(_nibbles, 6) != _nibbles[6]) {
    _last_error = wrong_crc;
    //Serial.println("wrong crc");
    return false;
  }

  if(!processData(timestamp)) {
    _last_error = process_error;
    return false;
  }

  if(_data_callback) {
    _data_callback(_nibbles, timestamp, _data_callback_user_data);
  }

  _packet_count++;

  handleSerialMsg(_status);
  return true;
}

bool RMT_SENT_RECEIVER::handleSerialMsg(uint8_t nibble) {
  //Serial.println(nibble, HEX);
  _serial_msg_bit2 = (_serial_msg_bit2 << 1) | ((nibble >> 2) & 0x01);
  _serial_msg_crc = (_serial_msg_crc << 1) | ((nibble >> 2) & 0x01);
  _serial_msg_bit3 = (_serial_msg_bit3 << 1) | (nibble >> 3);
  _serial_msg_crc = (_serial_msg_crc << 1) | (nibble >> 3);
  if((_serial_msg_bit3 & 0xFFFF) == 0x8000) { // short message
    _serial_msg_bit3 = 0;
    int8_t n[3];
    n[0] = (_serial_msg_bit2 >> 12)  & 0x0F;
    n[1] = (_serial_msg_bit2 >> 8)  & 0x0F;
    n[2] = (_serial_msg_bit2 >> 4)  & 0x0F;
    if(calcCRC4(n, 3) != (_serial_msg_bit2 & 0x0F)) {
      //Serial.println("short serial crc wrong");
      return false;
    }
    uint8_t msg_id = n[0];
    uint16_t msg_data = (_serial_msg_bit2 >> 4) & 0xFF;
    if(!processSerial(msg_id, msg_data)) {
      return false;
    }
    if(_serial_msg_callback) {
      _serial_msg_callback(msg_id, msg_data, _serial_callback_user_data);
    }
  } else if((_serial_msg_bit3 & 0x3f800) == 0x3f000) {
    if(!checkCRC6(_serial_msg_crc, (_serial_msg_bit2 >> 12) & 0x3f)) {
      //Serial.println("serial crc wrong");
      return false;
    }
    uint8_t msg_id;
    uint16_t msg_data;
    if(_serial_msg_bit3 & 0x400) { // 16/4
      msg_id = (_serial_msg_bit3 >> 6) & 0x0F;
      msg_data = _serial_msg_bit2 & 0xFFF | ((_serial_msg_bit3 << 11) & 0xF000);
      //Serial.printf("\rS_16/4: %X %04X    \r\n", msg_id, msg_data);
    } else { // 12/8
      msg_id = ((_serial_msg_bit3 >> 2) & 0xF0) | ((_serial_msg_bit3 >> 1) & 0x0F); 
      msg_data = _serial_msg_bit2 & 0xFFF;
    }
    if(!processSerial(msg_id, msg_data)) {
      return false;
    }
    if(_serial_msg_callback) {
      _serial_msg_callback(msg_id, msg_data, _serial_callback_user_data);
    }
    _serial_msg_bit3 = 0;
  }
  return true;
}

void RMT_SENT_RECEIVER::registerDataCallback(void (*callback)(int8_t* nibbles, uint32_t timestamp, void* user_data), void* user_data) {
  _data_callback = callback;
  _data_callback_user_data = user_data;
}

void RMT_SENT_RECEIVER::registerSerialMsgCallback(void (*callback)(uint8_t msg_id, uint16_t msg_data, void* user_data), void* user_data) {
  _serial_msg_callback = callback;
  _serial_callback_user_data = user_data;
}

RMT_SENT_RECEIVER::SENT_Error_t RMT_SENT_RECEIVER::getLastError() {
  return _last_error;
}

uint8_t RMT_SENT_RECEIVER::getStatus() {
  return _status & 0x03;
}

uint32_t RMT_SENT_RECEIVER::getPacketCount() {
  return _packet_count;
}

uint32_t RMT_SENT_RECEIVER::getErrorCount() {
  return _error_count;
}

void RMT_SENT_RECEIVER::getRawData(int8_t* nibbles) {
  for(int i = 0; i < 6; ++i) {
    nibbles[i] = _nibbles[i];
  }
}

void RMT_SENT_RECEIVER::invalidateSerial() {
  _serial_msg_bit3 = 0;
}

bool RMT_SENT_RECEIVER::processData(uint32_t timestamp) {
  return true;
}

bool RMT_SENT_RECEIVER::processSerial(uint8_t msg_id, uint16_t msg_data) {
  return true;
}

