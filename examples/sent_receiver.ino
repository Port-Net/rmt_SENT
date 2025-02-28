//
//    FILE: sent_receiver.cpp
//  AUTHOR: Christian Port
// PURPOSE: demo
//     URL: https://github.com/Port-Net/rmt_SENT
//
//  Example for using callbacks for MLX90377ADB-x3x rotation sensor based on rmt_sent receiver

#include <Arduino.h>
#include "rmt_SENT.h"

// MLX90377 magnetic position sensor

// create receiver on pin 38 with 3 µs ticks
RMT_SENT_RECEIVER receiver(GPIO_NUM_38, 3);

uint8_t last_counter = 0;
uint16_t value = 0;
uint16_t serial_status = 0;
float temp = 0;
uint32_t missed_packets = 0;

void dataCallback(int8_t* nibbles, void* userData) {
  uint8_t ct = nibbles[3] << 4 | nibbles[4];
  last_counter++;
  if(ct != last_counter) {
    missed_packets++;
    Serial.printf("missed %d\r\n", ct);
    receiver.invalidateSerial();
  }
  if((~(nibbles[0]) & 0x0f) != nibbles[5]) {
    Serial.printf("wrong invert %d\r\n", ct);
    receiver.invalidateSerial();
    return;
  }
  last_counter = ct;
  value = nibbles[0] << 8 | nibbles[1] << 4 | nibbles[2];
}

void serialMsgCallback(uint8_t msg_id, uint16_t msg_data, void* userData) {
  switch(msg_id) {
    case 1:
      serial_status = msg_data;
      break;
    case 0x23:
      temp = msg_data / 8 - 73; // was 73.15, but seems to be only in 1° steps
      break;
  }
}

void setup() {
  delay(5000); //wait for monitor to connect
  Serial.begin(115200);
  Serial.println("init");

  receiver.registerDataCallback(dataCallback);
  receiver.registerSerialMsgCallback(serialMsgCallback);
  receiver.begin();

  Serial.println("init done");
}

void loop() {
  static uint32_t last_print = 0;
  uint32_t ti = millis();
  if((ti - last_print) > 500) {
    last_print = ti;
    Serial.printf("state:%d state2:%X angle:%d temp:%.2f°C missed:%d err: %d\r\n",
        receiver.getStatus(), serial_status, value, temp, missed_packets, receiver.getErrorCount());
  }
}