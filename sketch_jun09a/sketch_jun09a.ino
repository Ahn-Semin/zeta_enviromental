//#include <PWM.h>
#include <Servo.h>
Servo servo;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(500);
  /*InitTimersSafe(); 
  bool success = SetPinFrequencySafe(3, 50);
  if(success) {
    pinMode(3, OUTPUT);
    digitalWrite(3, HIGH);
  }
  pwmWrite(3, 6);
  */
  servo.attach(3);
  delay(1000);
}

void loop() {/*
  static int i=3; // 6 33
  if(Serial.available()){
    char temp = Serial.read();
    if(temp == '1') i++;
    else if(temp == '2') i--;
    Serial.print("i = ");Serial.print(i);Serial.print("  duty ratio = ");Serial.print((float)i/255*100);Serial.println("%%");
    delay(200);
  }*/
  /*
  for(int j = 6; j < 34; j++) {
    pwmWrite(3,j);
    delay(200);
  }
  delay(1000);
  for(int j = 33; j > 5; j--) {
    pwmWrite(3,j);
    delay(200);
  }*/
  for(int j = 0; j < 50; j++) {
    servo.write(4*j);
    delay(200);
  }
  for(int j = 50; j > -1; j--) {
    servo.write(4*j);
    delay(200);
  }
  /*
  servo.write(0);
  delay(500);
  
  servo.write(60);
  delay(500);
  servo.write(120);
  delay(500);
  servo.write(60);
  delay(500);*/
  // put your main code here, to run repeatedly:

}
