#define READ_PIN  2
uint32_t cnt = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(READ_PIN),Count,RISING);
  pinMode(2,INPUT);
  pinMode(3,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(cnt);
  digitalWrite(3,HIGH);
  delay(5000);
  digitalWrite(3,LOW);
  delay(5000);
}

void Count() {
  cnt++;
}
