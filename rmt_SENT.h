#ifndef RMT_SENT_RECEIVER_H
#define RMT_SENT_RECEIVER_H

#include <Arduino.h>

#if ESP_ARDUINO_VERSION_MAJOR == 3
#include <driver/rmt_common.h>
#include <driver/rmt_types.h>
#include <driver/rmt_rx.h>
#else
#include <driver/rmt.h>
typedef struct {
  rmt_data_t received_symbols[16];
  size_t num_symbols;
} rmt_rx_done_event_data_t;
#endif

typedef struct {
  rmt_rx_done_event_data_t rmt_data;
  uint32_t timestamp;
} rmt_queue_item_t;

class RMT_SENT_RECEIVER {
public:
  enum SENT_Error_t {
    short_frame = 1,
    wrong_start = 2,
    wrong_nibble = 3,
    wrong_crc = 4,
    process_error = 5
  };
  /**
   * @brief Construct a new rmt sent receiver object
   * 
   * @param pin 
   * @param tick_time_us 
   */
  RMT_SENT_RECEIVER(uint8_t pin, uint8_t tick_time_us);

  /**
   * @brief initialize and start rmt receiver
   * 
   * @return true on success
   * @return false on error
   */
  bool begin();

  int getPin();

  /**
   * @brief register a callback on frame decoded successful. timestamp is micros of queuing
   * 
   * @param callback sig: void func(int8_t* nibbles, uint32_t timestamp, void* user_data)
   * @param user_data optional pointer forwarded to callback
   */
  void registerDataCallback(void (*callback)(int8_t* nibbles, uint32_t timestamp, void* user_data), void* user_data = NULL);

  /**
   * @brief register a callback on serial message received
   * 
   * @param callback sig: void func(uint8_t msg_id, uint16_t msg_data, void* user_data)
   * @param user_data optional pointer forwarded to callback
   */
  void registerSerialMsgCallback(void (*callback)(uint8_t msg_id, uint16_t msg_data, void* user_data), void* user_data = NULL);
  
  /**
   * @brief Get the Raw Data object
   * 
   * @param nibbles pointer to int8 array[6] to store the data nibbles
   */
  void getRawData(int8_t* nibbles);

  /**
   * @brief Get the Status object
   * 
   * @return uint8_t 2 bits of status nibble
   */
  uint8_t getStatus();

  /**
   * @brief Get the Count of frames with errors 
   * 
   * @return uint32_t count
   */
  uint32_t getErrorCount();

  /**
   * @brief Get the Packet Count object
   * 
   * @return uint32_t count of packets 
   */
  uint32_t getPacketCount();

  /**
   * @brief Get the Last Error
   * 
   * @return SENT_Error_t 
   */
  SENT_Error_t getLastError();

  /**
   * @brief invalidates the serial channel. use in case of error detected in frame
   * 
   */
  void invalidateSerial();
protected:
  /**
   * @brief function which is called each time a valid packet is received
   * 
   * @param timestamp micros the packet was received and queued
   */
  virtual bool processData(uint32_t timestamp);

  /**
   * @brief function which is called each time a valid serial message is received
   * 
   */
  virtual bool processSerial(uint8_t msg_id, uint16_t msg_data);

  int8_t _nibbles[7];
  uint32_t _packet_count;
private:
  #if ESP_ARDUINO_VERSION_MAJOR == 3
  static IRAM_ATTR bool rmtCallback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data);
  #else
  static IRAM_ATTR void rmtCallback(uint32_t *data, size_t len, void *user_data);
  #endif
  static void handlerThreadStarter(void* parameters);
  void handlerThread();
  uint8_t calcCRC4(int8_t* nibbles, uint8_t len);
  bool checkCRC6(uint32_t data, uint8_t crc6);
  bool decodeSENT(rmt_rx_done_event_data_t* rx_data, uint32_t timestamp);
  bool handleSerialMsg(uint8_t nibble);
  int _pin;
#if ESP_ARDUINO_VERSION_MAJOR == 3
  rmt_channel_handle_t _sent_chan;
  rmt_rx_channel_config_t _rx_chan_config;
  rmt_receive_config_t _sent_config;
  rmt_data_t _rmt_data[RMT_MEM_NUM_BLOCKS_1 * RMT_SYMBOLS_PER_CHANNEL_BLOCK];
  size_t _rmt_symbols = RMT_MEM_NUM_BLOCKS_1 * RMT_SYMBOLS_PER_CHANNEL_BLOCK;
  rmt_rx_event_callbacks_t _cbs;
#else
  rmt_obj_t* _rmt_receiver = NULL;
  rmt_config_t _rx_config;
#endif

  uint8_t _tick_time_us;
  QueueHandle_t _receive_queue;
  uint32_t _serial_msg_bit3;
  uint32_t _serial_msg_bit2;
  uint32_t _serial_msg_crc;
  uint8_t _status;
  SENT_Error_t _last_error;
  uint32_t _error_count;
  void* _data_callback_user_data;
  void (*_data_callback)(int8_t* nibbles, uint32_t timestamp, void* user_data);
  void* _serial_callback_user_data;
  void (*_serial_msg_callback)(uint8_t msg_id, uint16_t msg_data, void* user_data);
};

#endif
