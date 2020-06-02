void setup() {
  // put your setup code here, to run once:
  pinMode(6,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  digitalWrite(6,HIGH);
  delay(1000);
  digitalWrite(6,LOW);
}
