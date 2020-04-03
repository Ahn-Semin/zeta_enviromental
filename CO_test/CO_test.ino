char data[10];
float CO=0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Serial1.begin(9600,SERIAL_8N1);
  pinMode(A0,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*if(Serial1.available()){
    int no = Serial1.readBytes(data,9);
    data[no] = '\0';
    //CO = CO2ppm(CO_bit);
    Serial.println(data);
    //CO = (((float)CO_bit)*5/1024 - 0.39)*500/1.6;
    //Serial.println(CO);
    delay(500);
  }*/
  int curbit = analogRead(A0);
  Serial.println(curbit); //dun no why uart do not response
}

float CO2ppm(uint16_t CObit){
  float temp = 0.0;
  temp = (((float)CObit)*5/1024 - 0.39)*500/1.6;
  //if(temp<0.01)s
   // temp = 0.0;
}
