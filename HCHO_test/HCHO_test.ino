#include <SoftwareSerial.h>

SoftwareSerial HCHO_Serial(2,3);

uint32_t cnt = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  HCHO_Serial.begin(9600);
  HCHO_ActiveMode();
  //HCHO_qnaMode();
  pinMode(A0,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(HCHO_Serial.available()){
    Serial.print((char)HCHO_Serial.read());
  }
  if(!(cnt++%10000)){
    //Serial.println(analogRead(A0)*5/1024.0);
  }
}

void HCHO_ActiveMode() {
  byte send_data[9] = {0xff, 0x01, 0x78, 0x40, 0x00, 0x00, 0x00, 0x00, 0x47};
  HCHO_Serial.write(send_data,9);
  Serial.println("FS00501 Active Mode on!");
}

void HCHO_qnaMode() {
  byte send_data[9] = {0xff, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
  HCHO_Serial.write(send_data,9);
  Serial.println("FS00501 QnA Mode on!");
}
