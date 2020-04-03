/*
 * 아두이노에서 포센서 포름알데히드 센서를 구동테스트하기 위한 코드입니다. 
 * 소프트웨어 시리얼을 사용합니다. 
 * 아두이노 나노 혹은 아두이노 우노의 13번핀에 센서의 Tx를 연결하고
 * 12번핀에 센서의 Rx를 연결하십시오.  
 */


#include "SoftwareSerial.h"
#define LENGTH 9
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
//#define OLED_ADDR 0x3c
//#define OLED_RESET 6
//Adafruit_SSD1306 display(OLED_RESET);

SoftwareSerial HCHO_serial(10,11);
int incomingByte[9] = {0,};

unsigned long current_time = 0;
unsigned long prev_time = 0;
int set_time = 30000;

float HCHO_value = 0;
float zero = 0;
float sum = 0;
int cnt = 0;
bool zero_flag = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  HCHO_serial.begin(9600);
  pinMode(A5,INPUT);
  prev_time = millis();
  Serial.println("setup fin.");
  //display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  //display.clearDisplay();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(HCHO_serial.available()>0){
    for(int i = 0; i < LENGTH; i++){
      incomingByte[i] = HCHO_serial.read();
      Serial.print("0x");Serial.print(incomingByte[i],HEX);Serial.print(" ");
      delay(1);
      
    }
    Serial.println("");
    HCHO_value = (incomingByte[4] * 256 + incomingByte[5])- zero + 40;
    /*
     * 
    Serial.print("HCHO: ");
    Serial.print(HCHO_value);
    Serial.print(" ppb");
    Serial.println("");
    */
    Serial.print("HCHO: ");
    Serial.print(HCHO_value,0);
    Serial.print(" ppb");
    Serial.println("");
    
  }
  current_time = millis();
  if(current_time - prev_time >= set_time && zero_flag){
    if(++cnt>10) {
      zero = sum / 10.0;
      zero_flag = false;
    }
    sum+=HCHO_value;
  }
  
}
