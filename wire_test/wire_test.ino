#include <Wire.h>
#include <sSense-CCS811.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Wire.begin(I2C_CCS811_ADDRESS);
  Wire.onReceive(callback);
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.requestFrom(I2C_CCS811_ADDRESS,255);
  delay(1000);
}

void callback() {
  while(Wire.available()){
    Serial.print(Wire.read());
  }
}
