#include <SoftwareSerial.h>

SoftwareSerial BTSerial(7,8);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(7,INPUT);
  pinMode(8,OUTPUT);
  
  BTSerial.begin(9600);

  Serial.println("Arduino with HC-06 is ready.");
  BTSerial.println("BTSerial started at 9600");
}

void loop() {
  // put your main code here, to run repeatedly:

  //Serial.write("Hello! nice to meet you!");

  //delay(1000);
  
  if(BTSerial.available())
    Serial.write(BTSerial.read());

   if(Serial.available())
    BTSerial.write(Serial.read());
}
