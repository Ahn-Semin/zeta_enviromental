// station side
#define RXPIN  4
#define TXPIN  5   

#define SERIAL_SPEED 115200
#define BT_SPEED     115200

#define ONE_SEC 1000

#include <SoftwareSerial.h>

SoftwareSerial BTSerial(RXPIN,TXPIN);

char recByte = '\0';

void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_SPEED);
  delay(0.5*ONE_SEC);
  Serial.println("Serial connected");
  BTSerial.begin(BT_SPEED);
  delay(0.5*ONE_SEC);
  Serial.println("setup fin");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Keep reading from HC-06 and send to Arduino Serial Monitor
  if (BTSerial.available())
    Serial.write(BTSerial.read());
 
  // Keep reading from Arduino Serial Monitor and send to HC-06
  if (Serial.available())
  BTSerial.write(Serial.read());
}
