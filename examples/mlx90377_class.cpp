//
//    FILE: mlx90377_class.cpp
//  AUTHOR: Christian Port
// PURPOSE: demo
//     URL: https://github.com/Port-Net/rmt_SENT
//
//  Example for creating a class for MLX90377ADB-x3x rotation sensor based on rmt_sent receiver

#include <Arduino.h>
#include "rmt_sent_receiver.h"

class my_MLX90377 : public RMT_SENT_RECEIVER {
public:
  my_MLX90377(gpio_num_t pin, uint8_t tick_time_us) : RMT_SENT_RECEIVER(pin, tick_time_us) {
    
  }

  uint16_t getRawAngle() {
    return _raw_angle;
  }

  uint16_t getSerialStatus() {
    return _serial_status;
  }

  uint32_t getMissedPackets() {
    return _missed_packets;
  }

  float getTemperature() {
    return _temperature;
  }

private:
  bool processData() override {
    uint8_t ct = _nibbles[3] << 4 | _nibbles[4];
    _last_counter++;
    if(ct != _last_counter) {
      _missed_packets++;
      _last_counter = ct;
      //Serial.printf("missed %d\r\n", ct);
    }
    if((~(_nibbles[0]) & 0x0f) != _nibbles[5]) {
      //Serial.printf("wrong invert %d\r\n", ct);
      return false;
    }
    _raw_angle = _nibbles[0] << 8 | _nibbles[1] << 4 | _nibbles[2];
    return true;
  }

  bool processSerial(uint8_t msg_id, uint16_t msg_data) override {
    switch(msg_id) {
      case 1:
        _serial_status = msg_data;
        break;
      case 0x23:
        _temperature = msg_data / 8 - 73; // was 73.15, but seems to be only in 1° steps
        break;
    }
    return true;
  }

  uint16_t _raw_angle;
  uint8_t _last_counter;
  uint32_t _missed_packets;
  uint16_t _serial_status;
  float _temperature;
};

my_MLX90377 mlx90377(GPIO_NUM_38, 3);

void setup() {
  delay(4000); //wait for monitor to connect
  Serial.begin(115200);
  Serial.println("init");

  mlx90377.begin();

  Serial.println("init done");
}

void loop() {
  static uint32_t last_print = 0;
  uint32_t ti = millis();
  if((ti - last_print) > 500) {
    last_print = ti;
    Serial.printf("state:%d state2:%X angle:%d temp:%.2f°C pkt:%d missed:%d err: %d\r\n",
        mlx90377.getStatus(), mlx90377.getSerialStatus(), mlx90377.getRawAngle(), mlx90377.getTemperature(),
        mlx90377.getPacketCount(), mlx90377.getMissedPackets(), mlx90377.getErrorCount());
  }
}

