#include "FastLED.h"
#include <FreeRTOS_AVR.h>

SemaphoreHandle_t sem;

#define NUM_STRIPS 5

 #define LED1_NUM   40
 #define LED2_NUM   13
 #define LED3_NUM   13
 #define LED4_NUM   9
 #define LED5_NUM   9
 
// How many leds are connected?
#define NUMLEDS 142
 
// Define the Pin
#define DATA_PIN1 4
#define DATA_PIN2 5
#define DATA_PIN3 6
#define DATA_PIN4 7
#define DATA_PIN5 8
#define READ_PIN1 12
#define READ_PIN2 11

#define FORWARD   1
#define BACKWARD  2
 
// Define the array of leds
//CRGB leds[NUM_LEDS];

CRGB leds[NUM_STRIPS][LED1_NUM];
//CRGB leds2[LED2_NUM];
//CRGB leds3[LED3_NUM];
//CRGB leds4[LED4_NUM];
//CRGB leds5[LED5_NUM];

uint8_t lednum[NUM_STRIPS]={LED1_NUM, LED2_NUM, LED3_NUM, LED4_NUM, LED5_NUM};

CLEDController *controllers[NUM_STRIPS];

uint8_t gBrightness = 200;
//uint8_t gBrightness = 128;
uint8_t curPos=0, prevPos=0;

//CRGB prevColor=black;
//CRGB buff[LED1_NUM];

uint8_t thisdelay = 100;                                      // A delay value for the sequence(s).
uint8_t  thisfade = 192;                                      // How quickly does it fade? Lower = slower fade rate.

uint8_t x[LED1_NUM];                              // arrays for the 2d coordinates of any led
uint8_t y[LED1_NUM];

int val1=0, val2=0;

#define MIC_PIN    A1                                          // Analog port for microphone
#define DC_OFFSET  0                                          // DC offset in mic signal - if unusure, leave 0
 

#define qsubd(x, b)  ((x>b)?wavebright:0)                     // A digital unsigned subtraction macro. if result <0, then => 0. Otherwise, take on fixed value.
#define qsuba(x, b)  ((x>b)?x-b:0)                            // Analog Unsigned subtraction macro. if result <0, then => 0
// Initialize global variables for sequences
int wavebright = 10;

 byte colors[3][3] = { {0xff, 0,0}, {0xff, 0xff, 0xff}, {0   , 0   , 0xff} };

long firstval = 0xff00ff;
CRGB rgbval(50,0,500);
CHSV hsvval(150,255,200);

#define COLOR_ORDER BGR                                       // It's GRB for WS2812 and BGR for APA102.
#define LED_TYPE WS2812                                       // Using APA102, WS2812, WS2801. Don't forget to change 

byte r,g,b,dt;
byte r2,g2,b2;
byte inc, dec;
byte es, sd, rd;

byte cnt = 0;
byte buff[20] = {0};

void Thread1(void* arg) {
  while (1) {

    //세마포어를 받을때까지 스레드1은 계속 기다리게 됩니다.
    xSemaphoreTake(sem, portMAX_DELAY);

    //세마포어를 받을 경우 LED를 끕니다.
    //digitalWrite(LED_PIN, LOW);
  }
}

void Thread2(void* arg) {

  //pinMode(LED_PIN, OUTPUT);

  while (1) {
    //13번 LED를 켭니다
    //digitalWrite(LED_PIN, HIGH);

    //200ms를 대기합니다.
    vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);

    //세마포어를 전달합니다.
    xSemaphoreGive(sem);

    //다시 200ms 대기합니다.
    vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);
  }
}

void setup() { 
   portBASE_TYPE s1, s2;
   
   pinMode(DATA_PIN1,OUTPUT);
   pinMode(DATA_PIN2,OUTPUT);
   pinMode(DATA_PIN3,OUTPUT);
   pinMode(DATA_PIN4,OUTPUT);
   pinMode(DATA_PIN5,OUTPUT);
   pinMode(READ_PIN1,INPUT);
   pinMode(READ_PIN2,INPUT);
   
   pinMode(MIC_PIN,INPUT);
   
//  FastLED.addLeds<NEOPIXEL,DATA_PIN>(leds, NUM_LEDS);
  controllers[0] = &FastLED.addLeds<NEOPIXEL,DATA_PIN1>(leds[0], LED1_NUM);
  controllers[1] = &FastLED.addLeds<NEOPIXEL,DATA_PIN2>(leds[1], LED2_NUM);
  controllers[2] = &FastLED.addLeds<NEOPIXEL,DATA_PIN3>(leds[2], LED3_NUM);
  controllers[3] = &FastLED.addLeds<NEOPIXEL,DATA_PIN4>(leds[3], LED4_NUM);
  controllers[4] = &FastLED.addLeds<NEOPIXEL,DATA_PIN5>(leds[4], LED5_NUM);

  //LEDS.addLeds<LED_TYPE, DATA_PIN1, COLOR_ORDER>(leds[0], LED1_NUM);
 
  FastLED.setBrightness(gBrightness);

    // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial) {
    ;
  }
   
  set_max_power_in_volts_and_milliamps(5, 500);               // FastLED power management set at 5V, 500mA.

  sem = xSemaphoreCreateCounting(1, 0);

  s1 = xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  // initialize semaphore
  sem = xSemaphoreCreateCounting(1, 0);

  // create task at priority two
  s1 = xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  // create task at priority one
  s2 = xTaskCreate(Thread2, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  // check for creation errors
  if (sem== NULL || s1 != pdPASS || s2 != pdPASS ) {
    Serial.println(F("Creation problem"));
    while(1);
  }
  // start scheduler
  vTaskStartScheduler();
  
  delay(1000);
  
  Serial.println("Connect software serail!!\n");
  
}
 
void loop() { 

//NewKITT(0xff, 0, 0, 8, 10, 50,0);
//  CenterToOutside(0xff, 00, 0xff,00, 00, 0xff, 3, 10, 50, 0);
//  CenterToOutside(0xff, 00, 0xff,00, 00, 0xff, 2, 50, 100, 1);
//  CenterToOutside(0xff, 00, 0xff,00, 00, 0xff, 1, 80, 150, 3);
  
  if(Serial.available()) {
    buff[cnt++] = Serial.read();
  }

  if(cnt>=10) {
    cnt = 0;
    switch(buff[0]) {
      case 1:
        r= buff[1];
        g= buff[2];
        b= buff[3];
        FillSolid_All(r,g,b);       
        break;

      case 2:
        r= buff[1];
        g= buff[2];
        b= buff[3];
        dt= buff[4];
        inc= buff[5];
        FadeInAll(r, g, b, dt, inc);
        break;

      case 3:
        r= buff[1];
        g= buff[2];
        b= buff[3];
        dt= buff[4];
        dec= buff[5];
        FadeOutAll(r, g, b, dt, dec);
        break;

      case 4:
        r= buff[1];
        g= buff[2];
        b= buff[3];
        r2= buff[4];
        g2= buff[5];
        b2= buff[6];       
        dt= buff[7];
        inc= buff[8];
        FadeOut2All(r, g, b, r2, g2, b2, dt, inc);
        break;

      case 5:
        r= buff[1];
        g= buff[2];
        b= buff[3];
        r2= buff[4];
        g2= buff[5];
        b2= buff[6];       
        sd= buff[7];
        rd= buff[8];
        CenterToOutsideAll2(r, g, b, r2, g2, b2, sd, rd);
        break;

                
    }
  }
  
  
 /*FillSolid_All(255,0,0);
 delay(1000);
 FillSolid_All(0,255,0);
 delay(1000);
 FillSolid_All(0,0,255);
 delay(1000);
 FillSolid_All(255,255,0);
 delay(1000);
 FillSolid_All(255,128,0);
 delay(1000);
 */
 
}

void CenterToOutsideAll2(byte reds, byte greens, byte blues, byte red, byte green, byte blue, int SpeedDelay, int ReturnDelay) {
  byte center;
  //byte d[5] = {0, 10, 20, 30, 40};

  for(int j=0; j<5; j++)
    setAll(reds,greens,blues,j);

  for(int j=0; j<5; j++) {
    center = (lednum[j]/2);
    setPixel(center, red, green, blue, j);
  }
  delay(SpeedDelay);

  for(int i =1; i<=(lednum[0]/2); i++) {    
//  for(int i =(lednum[0]/2); i>=0; i--) {    
    for(int j=0; j<5; j++) {
      center = (lednum[j]/2);
      if((center+i) <= lednum[j])
        setPixel(center+i, red, green, blue, j);
      if((center-i) >= 0)
        setPixel(center-i, red, green, blue, j);
      
    }
    for(int j=0; j<5; j++)
      showStrip(j);

    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}

void CenterToOutsideAll(byte reds, byte greens, byte blues, byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay) {
  for(int i =((lednum[0]-EyeSize)/2); i>=0; i--) {
    for(int j=0; j<5; j++)
      setAll(reds,greens,blues,j);
      
    for(int j=0; j<5; j++)
      setPixel(i, red/10, green/10, blue/10, j);
      
    for(int j = 1; j <= EyeSize; j++) {
      for(int j=0; j<5; j++)
        setPixel(i+j, red, green, blue, j); 
    }
    for(int j=0; j<5; j++) {
      setPixel(i+EyeSize+1, red/10, green/10, blue/10, j);
      setPixel(lednum[j]-i, red/10, green/10, blue/10, j);
    }
    
    for(int j = 1; j <= EyeSize; j++) {
      for(int j=0; j<5; j++)
        setPixel(lednum[j]-i-j, red, green, blue, j); 
    }
    for(int j=0; j<5; j++)
      setPixel(lednum[j]-i-EyeSize-1, red/10, green/10, blue/10, j);

    for(int j=0; j<5; j++)
      showStrip(j);
      
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}

void CenterToOutside(byte reds, byte greens, byte blues, byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay, byte indexled) {
  for(int i =((lednum[indexled]-EyeSize)/2); i>=0; i--) {
    setAll(reds,greens,blues,indexled);
    
    setPixel(i, red/10, green/10, blue/10, indexled);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(i+j, red, green, blue, indexled); 
    }
    setPixel(i+EyeSize+1, red/10, green/10, blue/10, indexled);
    
    setPixel(lednum[indexled]-i, red/10, green/10, blue/10, indexled);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(lednum[indexled]-i-j, red, green, blue, indexled); 
    }
    setPixel(lednum[indexled]-i-EyeSize-1, red/10, green/10, blue/10, indexled);
    
    showStrip(indexled);
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}

void FillSolid_All(byte red, byte green, byte blue)
{
  for(int i=0; i<5; i++)
    setAll(red, green, blue, i);
  
}

void FadeInAll(byte red, byte green, byte blue, byte dtime, byte inc)
{
  float r, g, b;
  for(int i=0; i<5; i++) {
    setAll(0,0,0,i);
    showStrip(i);
  }
      
  for(int k = 0; k < 256; k+=inc) { 
    r = (k/256.0)*red;
    g = (k/256.0)*green;
    b = (k/256.0)*blue;
    setAll(r,g,b, 0);
    showStrip(0);
    setAll(r,g,b, 1);
    showStrip(1);
    setAll(r,g,b, 2);
    showStrip(2);
    setAll(r,g,b, 3);
    showStrip(3);
    setAll(r,g,b, 4);
    showStrip(4);
    delay(dtime);
  }
}

void FadeIn(byte red, byte green, byte blue, byte indexled, word dtime, byte inc){
  float r, g, b;

  setAll(0,0,0,indexled);
  showStrip(indexled);      
  
  for(int k = 0; k < 256; k+=inc) { 
    r = (k/256.0)*red;
    g = (k/256.0)*green;
    b = (k/256.0)*blue;
    setAll(r,g,b, indexled);
    showStrip(indexled);
    delay(dtime);
  }
}


void FadeOut2All(byte reds, byte greens, byte blues, byte rede, byte greene, byte bluee, byte dtime, byte inc){
  float r, g, b;
  float rs, gs, bs;

  rs = (float)(reds - rede)/256.;
  gs = (float)(greens - greene)/256.;
  bs = (float)(blues - bluee)/256.;

  for(int i=0; i<5; i++) {
    setAll(reds,greens,blues,i);
    showStrip(i);
  }
  
//  for(int k = 255; k >= 0; k=k+1) {
  for(int k = 0; k <= 255; k+=inc) {
    r = (reds - rs*k);
    g = (greens - gs*k);
    b = (blues - bs*k);
    setAll(r,g,b, 0);
    showStrip(0);
    setAll(r,g,b, 0);
    showStrip(0);
    setAll(r,g,b, 1);
    showStrip(1);
    setAll(r,g,b, 2);
    showStrip(2);
    setAll(r,g,b, 3);
    showStrip(3);
    setAll(r,g,b, 4);
    showStrip(4);   
    delay(dtime);
  }
}

void FadeOutAll(byte red, byte green, byte blue, byte dtime, byte dec){
  float r, g, b;

  for(int i=0; i<5; i++) {
    setAll(red,green,blue,i);
    showStrip(i);
  }
  
  for(int k = 255; k >= 0; k-= dec) {
    r = (k/256.0)*red;
    g = (k/256.0)*green;
    b = (k/256.0)*blue;
    setAll(r,g,b, 0);
    showStrip(0);
    setAll(r,g,b, 0);
    showStrip(0);
    setAll(r,g,b, 1);
    showStrip(1);
    setAll(r,g,b, 2);
    showStrip(2);
    setAll(r,g,b, 3);
    showStrip(3);
    setAll(r,g,b, 4);
    showStrip(4);   
    delay(dtime);
  }

  for(int i=0; i<5; i++) {
    setAll(0,0,0,i);
    showStrip(i);
  }  
}

void FadeOut(byte red, byte green, byte blue, byte indexled, byte dtime, byte dec){
  float r, g, b;
  
  setAll(red,green,blue,indexled);
  showStrip(indexled);
    
  for(int k = 255; k >= 0; k-=dec) {
    r = (k/256.0)*red;
    g = (k/256.0)*green;
    b = (k/256.0)*blue;
    setAll(r,g,b, indexled);
    showStrip(indexled);
    delay(dtime);
  }
}

void FadeInOut(byte red, byte green, byte blue, byte indexled, word dtime){
  float r, g, b;
      
  for(int k = 0; k < 256; k=k-1) { 
    r = (k/256.0)*red;
    g = (k/256.0)*green;
    b = (k/256.0)*blue;
    setAll(r,g,b, indexled);
    showStrip(indexled);
    delay(dtime);
  }
     
  for(int k = 255; k >= 0; k=k-2) {
    r = (k/256.0)*red;
    g = (k/256.0)*green;
    b = (k/256.0)*blue;
    setAll(r,g,b, indexled);
    showStrip(indexled);
    delay(dtime);
  }
}

void showStrip(byte indexled) {
  controllers[indexled]->showLeds(gBrightness);
}

void setPixel(int Pixel, byte red, byte green, byte blue, byte indexled) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[indexled][Pixel].r = red;
   leds[indexled][Pixel].g = green;
   leds[indexled][Pixel].b = blue;
 #endif
}

void setAll(byte red, byte green, byte blue, byte indexled) {
  for(int i = 0; i < lednum[indexled]; i++ ) {
    setPixel(i, red, green, blue, indexled); 
  }
  showStrip(indexled);
}
