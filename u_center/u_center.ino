#include <HardwareSerial.h>

HardwareSerial gpsSerial(1); // GPS连接在 UART1

void setup() {
  Serial.begin(9600);              // 用来连接 u-center（设置为9600）
  gpsSerial.begin(9600, SERIAL_8N1, 34, 12); // T-Beam GPS接口
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    Serial.write(c); // 转发给 USB 串口
  }
}
