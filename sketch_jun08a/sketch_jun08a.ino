#include <PWM.h>

uint32_t t_wake;
#define LEVEL0      0UL
#define LEVEL1      100UL
#define LEVEL2      250UL
#define LEVEL3      500UL   
#define PWM_PIN     3

byte recByte = '\0';

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  InitTimersSafe(); 
  bool success = SetPinFrequencySafe(PWM_PIN, LEVEL1);
  if(success) {
    pinMode(3, OUTPUT);
    digitalWrite(3, HIGH);
  }
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  pwmWrite(PWM_PIN, 122);
  setLevel();
}

void getByte() {
  if(Serial.available()) recByte = Serial.read();
}

void setLevel() {
  getByte();
  if(recByte !='\0') {
    InitTimersSafe();
    Serial.println((char)recByte);
  }
  if(recByte == '1') SetPinFrequencySafe(PWM_PIN, LEVEL1);
  else if(recByte == '2') SetPinFrequencySafe(PWM_PIN, LEVEL2);
  else if(recByte == '3') SetPinFrequencySafe(PWM_PIN, LEVEL3);
  else if(recByte == '0') SetPinFrequencySafe(PWM_PIN, LEVEL0);
  recByte = '\0';
}
