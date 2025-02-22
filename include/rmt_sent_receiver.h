#ifndef RMT_SENT_RECEIVER_H
#define RMT_SENT_RECEIVER_H

#include <Arduino.h>
#include <driver/rmt_common.h>
#include <driver/rmt_rx.h>

class RMT_SENT_RECEIVER {
public:
  enum SENT_Error_t {
    short_frame = 1,
    wrong_start = 2,
    wrong_nibble = 3,
    wrong_crc = 4
  };
  RMT_SENT_RECEIVER(gpio_num_t pin, uint8_t _tick_time_us);
  bool begin();
  void registerDataCallback(void (*callback)(int8_t* nibbles, void* user_data), void* user_data = NULL);
  void registerSerialMsgCallback(void (*callback)(uint8_t msg_id, uint16_t msg_data, void* user_data), void* user_data = NULL);
  void getRawData(int8_t* nibbles);
  uint8_t getStatus();
  uint32_t getErrorCount();
  SENT_Error_t getLastError();
  void invalidateSerial();
protected:
  virtual bool processData();
  virtual bool processSerial(uint8_t msg_id, uint16_t msg_data);
  int8_t _nibbles[7];
private:
  static IRAM_ATTR bool rtmCallback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data);
  static void handlerThread(void* parameters);
  uint8_t calcCRC4(int8_t* nibbles, uint8_t len);
  bool checkCRC6(uint32_t data, uint8_t crc6);
  bool decodeSENT(rmt_rx_done_event_data_t* rx_data);
  bool handleSerialMsg(uint8_t nibble);
  rmt_channel_handle_t _sent_chan;
  rmt_rx_channel_config_t _rx_chan_config;
  rmt_receive_config_t _sent_config;
  rmt_data_t _rmt_data[RMT_MEM_NUM_BLOCKS_1 * RMT_SYMBOLS_PER_CHANNEL_BLOCK];
  size_t _rmt_symbols = RMT_MEM_NUM_BLOCKS_1 * RMT_SYMBOLS_PER_CHANNEL_BLOCK;
  uint8_t _tick_time_us;
  QueueHandle_t _receive_queue;
  rmt_rx_event_callbacks_t _cbs;
  uint32_t _serial_msg_bit3;
  uint32_t _serial_msg_bit2;
  uint32_t _serial_msg_crc;
  uint8_t _status;
  SENT_Error_t _last_error;
  uint32_t _error_count;
  void* _data_callback_user_data;
  void (*_data_callback)(int8_t* nibbles, void* user_data);
  void* _serial_callback_user_data;
  void (*_serial_msg_callback)(uint8_t msg_id, uint16_t msg_data, void* user_data);
};

#endif