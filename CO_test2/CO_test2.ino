void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A0,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t analog = 0;
  char data_str[10] = {0};
  analog = analogRead(A0);
  //Serial.print("current voltage = ");Serial.print( (float)analog*5/1024 );Serial.println("V");
  Serial.print("current voltage = ");Serial.print( (float)analog*5/1024 );Serial.println("V");
 
  
  delay(1000);
}
