#include <SoftwareSerial.h>

#define NO2_rxPIN 2
#define NO2_txPIN 3

SoftwareSerial NO2_Serial(NO2_rxPIN,NO2_txPIN);

char inputchar = '\0';         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  NO2_Serial.begin(9600);
 
}

void loop() {
  if (stringComplete) {
    Serial.print(inputchar);
    // clear the string:
    NO2_Serial.write(&inputchar,1);
    inputchar = '\0';
    stringComplete = false;
  }
  if(NO2_Serial.available()){
    Serial.print((char)NO2_Serial.read());
  }
  // put your main code here, to run repeatedly:

}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar != '\0') {
      inputchar = inChar;
      stringComplete = true;
    }
  }
}
