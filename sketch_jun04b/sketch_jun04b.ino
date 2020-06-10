void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(18,INPUT_PULLUP);
  delay(50);
  attachInterrupt(digitalPinToInterrupt(18), ContactCallbackP, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void ContactCallbackP() {
  static uint32_t time_pre;
  uint32_t time_current = micros();
  uint32_t diff = time_current - time_pre;
  time_pre = time_current;
  Serial.println(diff);
  
}
