const int PWM_pin = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(2,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(2,122);
  delay(1000);
}
