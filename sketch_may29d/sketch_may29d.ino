void setup() {
  // put your setup code here, to run once:
  pinMode(2,OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  char rec = '\0';
  if(Serial.available()){
    rec =  Serial.read();
  }
  if(rec == 'l'){
    digitalWrite(2,LOW);
  }
  if(rec == 'h') {
    digitalWrite(2,HIGH); 
  }
  
}
