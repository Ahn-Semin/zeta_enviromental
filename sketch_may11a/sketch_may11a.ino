#define       FULL    255
#define       DUTY    0.5f

void setup() {
  // put your setup code here, to run once:
  pinMode(10,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  digitalWrite(3,HIGH);
  
}

void loop() {
  digitalWrite(2,LOW);
  analogWrite(10,FULL*DUTY);
  delay(60000);
  digitalWrite(2,HIGH);
  delay(60000);
  // put your main code here, to run repeatedly:

}
