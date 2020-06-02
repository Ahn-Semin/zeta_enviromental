#define READ_PIN  19
uint32_t cnt = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(READ_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(READ_PIN),Count,RISING);
  
  pinMode(9,OUTPUT);
  //pinMode(3,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(9,122);
  Serial.println(cnt);
  //digitalWrite(3,HIGH);
  
}

void Count() {
  cnt++;
}
