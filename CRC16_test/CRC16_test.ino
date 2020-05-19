#include <CRC16_modbus.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t send[] = {0x3F, 0x10, 0x62, 0x00, 0x00, 0x08, 0x10, 0x00, 0x01,
  0x00, 0x01, 0x86, 0xA0, 0x01, 0xF4, 0x00, 0x64, 0x00, 0x64, 0x00, 0x00, 0x00, 0x10};
  Serial.println((uint16_t)CRC16_MODBUS(send,sizeof(send) ) ,HEX);
  Serial.println((uint16_t)sizeof(send));
  delay(1000);
}
