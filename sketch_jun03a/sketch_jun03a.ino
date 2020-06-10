#include <Timer.h>

Timer tTimer;
int cnt;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(2,INPUT);
  tTimer.every(5000,initcnt);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(!digitalRead(2)) {
    if(cnt++ > 50){
      Serial.println("yes!!!!!!!");
      cnt = 0;
    }
  }
  delay(10);
  tTimer.update();
}

void initcnt() {
  cnt = 0;
}
