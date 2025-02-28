# rmt_SENT
ESP32 rmt SENT receiver lib for PlatformIO<br>
Documentation is in <a href="https://dokumen.pub/qdownload/sent-single-edge-nibble-transmission-for-automotive-applications-j2716-2016-04.html">J2716</a><br>

<pre>
The library can be used to receive SENT (Single Edge Nibble Transmission)
frames via rmt hardware on ESP32.
It works only with pause pulse which is longer than 60 ticks which is used as end marker.

To use the class as base there are two methods which can be overrided:
bool processData();
bool processSerial(uint8_t msg_id, uint16_t msg_data);
This methods have to return false if an error in the packet is detected, else true. The nibbles are available in _nibbles array.
There is an example included.

To use the class by its own one can register callbacks for data and serial messages.
registerDataCallback(dataCB [, void* user_data]);
registerSerialMsgCallback(serialCB [, void* user_data]);
callback signature:
void dataCB(int8_t* nibbles, void* user_data)
void serialCB(uint8_t msg_id, uint16_t msg_data, void* user_data)
</pre>

