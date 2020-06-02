#define Ipin1 18
#define Ipin2 19

uint32_t cnt1;
uint32_t cnt2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(Ipin1,INPUT_PULLUP);
  delay(10);
  pinMode(Ipin2,INPUT_PULLUP);
  delay(10);
  attachInterrupt(digitalPinToInterrupt(Ipin1),ipin1,RISING);
  delay(10);
  attachInterrupt(digitalPinToInterrupt(Ipin2),ipin2,RISING);
  delay(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print(cnt1);Serial.print("         ");Serial.println(cnt2);
  //delay(500);
}

void ipin1() {
  //Serial.println(cnt1++);
  Serial.println("int1");
}

void ipin2() {
  //Serial.println(cnt2++);
  Serial.print("int2");
}
