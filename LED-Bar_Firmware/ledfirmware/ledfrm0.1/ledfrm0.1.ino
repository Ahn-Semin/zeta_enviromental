#include "FastLED.h"

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
 
void setup() { 
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

  FastLED.setBrightness(gBrightness);
  set_max_power_in_volts_and_milliamps(5, 500);               // FastLED power management set at 5V, 500mA.

   // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial) {
    ;
  }

  delay(1000);
  
  Serial.println("Connect software serail!!\n");
  
  
}
 
void loop() {  
  printf("HI");

  ripple(0, LED1_NUM, 255);

  val1 = digitalRead(READ_PIN1);
  val2 = digitalRead(READ_PIN2);

  if(!val1) {
    Serial.println("Pressed touch sensor 1!\n");
    RGBLoop(0);
  }

   if(!val2) {
    Serial.println("Pressed touch sensor 2!\n");
    theaterChase(10, 0, 0,0xff,0,50,0, LED1_NUM);
   }
    
  //soundmems(0, LED1_NUM);

 // ExampleTotal(0, LED1_NUM);
  
  //theaterChase(10, 1, 0xff,0,0,50,0, LED1_NUM);
  //theaterChase(10, 0, 0,0xff,0,50,0, LED1_NUM);
  //theaterChase(10, 1, 0,0,0xff,50,0, LED1_NUM);

  //RGBLoop(0);
  
  //theaterChaseRainbow(100, 1, 50, 0, LED1_NUM);
  //theaterChaseRainbow(100, 0, 50, 0, LED1_NUM);
  
  //Sparkle(0xff, 0xff, 0xff, 0, 0, LED1_NUM);

  //delay(1000);
  
  //SnowSparkle(0x10, 0x10, 0x10, 20, random(10,100), 0, LED1_NUM);
  
  //RunningLights(0xff,0xff,0x00, 50, 0, LED1_NUM);

  //TwinkleRandom(LED1_NUM, 100, false,0,LED1_NUM);

  //rainbowCycle(10, 0,LED1_NUM);
    //HalloweenEyes(0xff, 0x00, 0x00, 1, 4, true, random(5,50), random(50,150), random(1000, 10000),0);
                
   // Fire(55,120,15,0);

  //colorWipe(0x00,0xff,0x00, 50, 0);
  //colorWipe(0x00,0x00,0x00, 50, 1);
  //colorWipe(0x10,0x10,0x10, 50, 0);


  //BouncingBalls(0,0xff,0xff, 3, 0);

 /*byte colors[3][3] = { {0xff, 0,0}, 
                        {0xff, 0xff, 0xff}, 
                        {0   , 0   , 0xff} };

  BouncingColoredBalls(3, colors, 0);
  */
  //FadeInOut(0xff, 0x77, 0x00, 0, 5);

  Strobe(0xff, 0xff, 0xff, 10, 50, 1000, 1);
  Strobe(0xff, 0x70, 0x70, 10, 50, 1000, 2);

  //NewKITT(0xff, 0, 0, 8, 10, 50,0);

 //CylonBounce(0xff, 0, 0, 4, 10, 50, 0);
 CylonBounce(0, 0xff, 0, 4, 30, 50, 1);
 CylonBounce(0, 0, 0xff, 4, 30, 50, 2);
 CylonBounce(0x55, 0x55, 0, 4, 50, 50, 3);
 CylonBounce(0x55, 0, 0x55, 4, 50, 50, 4);
 
   //ChangeMe();                                                 // Check the demo loop for changes to the variables.
  //mover(0); 
  
#if 0
 static int sign = 1;

 //FastLED.clear();
  //controllers[0]->clear();
  //controllers[1]->clear();

//  FastLED[0].clear();
 //fill_solid(leds[0], LED1_NUM, CRGB::Red);
 fill_solid(leds[1], LED2_NUM, CRGB::Green);
 fill_solid(leds[2], LED3_NUM, CRGB::Blue);
 fill_solid(leds[3], LED4_NUM, CRGB::Yellow);
 fill_solid(leds[4], LED5_NUM, CRGB::Orange);

// controllers[0]->showLeds(gBrightness);
 controllers[1]->showLeds(gBrightness);
 controllers[2]->showLeds(gBrightness);
 controllers[3]->showLeds(gBrightness);
 controllers[4]->showLeds(gBrightness);

  gBrightness = 0;
  while(1) {
 //for(int i=0; i<100000; i++) {
  gBrightness  += sign;

  //if(gBrightness<=0) {
    //if(sign==1)
    //  sign = -1
    // else
    // sign = 1;

  if(sign==1) {
    if(gBrightness>=255) {
      gBrightness=255;
      sign=-1;
    }
  } else if(sign==-1) {
    if(gBrightness<=0) {
      gBrightness=0;
      sign =1;
    }
  }
  //controllers[0]->showLeds(gBrightness);

  /*for(int i=0; i<lednum[0]; i++) {
    leds[0][i] = CRGB::Black;
  }*/

  //fill_solid(leds[0], LED1_NUM, CRGB::Black);
  
  //prevPos = 0;
 // curPos = 0;
  //RotBit(FORWARD, 1, 0, CRGB::Black, CRGB::Blue);
  
  controllers[1]->showLeds(gBrightness);
  controllers[2]->showLeds(gBrightness);
  controllers[3]->showLeds(gBrightness);
  controllers[4]->showLeds(gBrightness);
 
  delay(20);
 }
#endif
 
 /* for(int i=0; i<LED1_NUM; i++) {
    leds1[i] = CRGB::Green;
  }
  
  for(int i=0; i<LED2_NUM; i++) {
    leds2[i] = CRGB::Red;
  }

  for(int i=0; i<LED3_NUM; i++) {
    leds3[i] = CRGB::Green;
  }

  for(int i=0; i<LED4_NUM; i++) {
    leds4[i] = CRGB::Yellow;
  }
  for(int i=0; i<LED5_NUM; i++) {
    leds5[i] = CRGB::Orange;
  }

  FastLED.setBrightness(50);
  FastLED.show();*/
  

#if 0 
  //int val = analogRead(1);
  int val = 950;
  if (val < 1000) {
 
    // Map the pot values to 0 - Number of Leds
    int numLedsToLight = map(val, 0, 950, 0, NUM_LEDS);
 
    // Clear the existing led values
    FastLED.clear();
 
    // Change led colors
    for(int led = 0; led < numLedsToLight; led++) { 
      if(led < 12)leds[led] = CRGB::Green;
      if(led >=12 & led < 24)leds[led] = CRGB::Orange;
      if(led >=24 & led < 36)leds[led] = CRGB::Pink;
      if(led >=36 & led < 48)leds[led] = CRGB::Cyan;
      if(led >=36 & led < 48)leds[led] = CRGB::Honeydew;
      if(led >=48 & led < 60)leds[led] = CRGB::Ivory;
      if(led >=60 & led < 72)leds[led] = CRGB::LightBlue;
      if(led >=72 & led < 84 )leds[led] = CRGB::Magenta;
      if(led >=84 & led < 96 )leds[led] = CRGB::MediumSpringGreen;
      if(led >=96 & led < 108 )leds[led] = CRGB::PaleGoldenrod;
      if(led >=108 & led < 120 )leds[led] = CRGB::RosyBrown;
      if(led >=120 & led < 132 )leds[led] = CRGB::Seashell;      
      if(led >=132)leds[led] = CRGB::Red;

      
     }
 
     FastLED.setBrightness(50);
     FastLED.show();

    for(int led = 0; led < numLedsToLight; led++) { 
        leds[led] = CRGB::Blue;
        FastLED.show();

        leds[led] = CRGB::Black;
        delay(20);
    }
     
   }
   else {
 
//    Loop for the Matrix example
  
     FastLED.clear();
     for(int i = 0; i < NUM_LEDS; i++) {
       leds[i] = invader1a[i];
   }
   FastLED.setBrightness(50);
   FastLED.show();
   delay(500);
  
   for(int i = 0; i < NUM_LEDS; i++) {
     leds[i] = invader1b[i];
   }
 
   FastLED.setBrightness(50);
   FastLED.show();
   delay(500);

 
 }
 #endif
 
}

void RotBit(uint8_t dir, uint8_t patternsize, byte ledindex, CRGB bcolor, CRGB fcolor)
{
  /*for(int i=0; i<lednum[ledindex]; i++) {
    leds[ledindex][i] = CRGB::Black;
  }*/

//  int8_t pos;
  
  if(dir==FORWARD) {
    curPos++;
    if(curPos>=lednum[ledindex])
      curPos = lednum[ledindex];

    for(int i=curPos; i<patternsize; i++) {
      leds[ledindex][i] = bcolor;
    }      
    for(int i=curPos; i<patternsize; i++) {
      leds[ledindex][i] = fcolor;
    }

    prevPos = curPos;

    controllers[ledindex]->showLeds(gBrightness);
    
  } else if(dir==BACKWARD) {
    curPos--;
    
  }
}


void mover(byte indexled) { 
  static uint8_t hue = 0;
  for (int i = 0; i < lednum[indexled]; i++) {
    leds[indexled][i] += CHSV(hue, 255, 255);
    leds[indexled][(i+5) % lednum[indexled]] += CHSV(hue+85, 255, 255);         // We use modulus so that the location is between 0 and NUM_LEDS
    leds[indexled][(i+10) % lednum[indexled]] += CHSV(hue+170, 255, 255);       // Same here.
    show_at_max_brightness_for_power();
    fadeToBlackBy(leds[indexled], lednum[indexled], thisfade);                  // Low values = slower fade.
    delay(thisdelay);                                         // UGH!!!! A blocking delay. If you want to add controls, they may not work reliably.
  }
} // mover()


void ChangeMe() {                                             // A time (rather than loop) based demo sequencer. This gives us full control over the length of each sequence.
  uint8_t secondHand = (millis() / 1000) % 15;                // IMPORTANT!!! Change '15' to a different value to change duration of the loop.
  static uint8_t lastSecond = 99;                             // Static variable, means it's only defined once. This is our 'debounce' variable.
  if (lastSecond != secondHand) {                             // Debounce to make sure we're not repeating an assignment.
    lastSecond = secondHand;
    switch(secondHand) {
      case  0: thisdelay=20; thisfade=240; break;             // You can change values here, one at a time , or altogether.
      case  5: thisdelay=50; thisfade=128; break;
      case 10: thisdelay=100; thisfade=64; break;             // Only gets called once, and not continuously for the next several seconds. Therefore, no rainbows.
      case 15: break;
    }
  }
} // ChangeMe()

void CylonBounce(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay, byte indexled){

  for(int i = 0; i < lednum[indexled]-EyeSize-2; i++) {
    setAll(0,0,0, indexled);
    setPixel(i, red/10, green/10, blue/10, indexled);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(i+j, red, green, blue,indexled); 
    }
    setPixel(i+EyeSize+1, red/10, green/10, blue/10,indexled);
    showStrip(indexled);
    delay(SpeedDelay);
  }

  delay(ReturnDelay);

  for(int i = lednum[indexled]-EyeSize-2; i > 0; i--) {
    setAll(0,0,0,indexled);
    setPixel(i, red/10, green/10, blue/10,indexled);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(i+j, red, green, blue, indexled); 
    }
    setPixel(i+EyeSize+1, red/10, green/10, blue/10,indexled);
    showStrip(indexled);
    delay(SpeedDelay);
  }
  
  delay(ReturnDelay);
}


void NewKITT(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay, byte indexled){
  RightToLeft(red, green, blue, EyeSize, SpeedDelay, ReturnDelay, indexled);
  LeftToRight(red, green, blue, EyeSize, SpeedDelay, ReturnDelay, indexled);
  OutsideToCenter(red, green, blue, EyeSize, SpeedDelay, ReturnDelay, indexled);
  CenterToOutside(red, green, blue, EyeSize, SpeedDelay, ReturnDelay, indexled);
  LeftToRight(red, green, blue, EyeSize, SpeedDelay, ReturnDelay, indexled);
  RightToLeft(red, green, blue, EyeSize, SpeedDelay, ReturnDelay, indexled);
  OutsideToCenter(red, green, blue, EyeSize, SpeedDelay, ReturnDelay, indexled);
  CenterToOutside(red, green, blue, EyeSize, SpeedDelay, ReturnDelay, indexled);
}

void CenterToOutside(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay, byte indexled) {
  for(int i =((lednum[indexled]-EyeSize)/2); i>=0; i--) {
    setAll(0,0,0,indexled);
    
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

void OutsideToCenter(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay, byte indexled) {
  for(int i = 0; i<=((lednum[indexled]-EyeSize)/2); i++) {
    setAll(0,0,0,indexled);
    
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

void LeftToRight(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay, byte indexled) {
  for(int i = 0; i < lednum[indexled]-EyeSize-2; i++) {
    setAll(0,0,0,indexled);
    setPixel(i, red/10, green/10, blue/10, indexled);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(i+j, red, green, blue, indexled); 
    }
    setPixel(i+EyeSize+1, red/10, green/10, blue/10, indexled);
    showStrip(indexled);
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}

void RightToLeft(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay, byte indexled) {
  for(int i = lednum[indexled]-EyeSize-2; i > 0; i--) {
    setAll(0,0,0,indexled);
    setPixel(i, red/10, green/10, blue/10, indexled);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(i+j, red, green, blue, indexled); 
    }
    setPixel(i+EyeSize+1, red/10, green/10, blue/10, indexled);
    showStrip(indexled);
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}


void FadeInOut(byte red, byte green, byte blue, byte indexled, word dtime){
  float r, g, b;
      
  for(int k = 0; k < 256; k=k+1) { 
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


void RGBLoop(byte indexled){
  for(int j = 0; j < 3; j++ ) { 
    // Fade IN
    for(int k = 0; k < 256; k++) { 
      switch(j) { 
        case 0: setAll(k,0,0,indexled); break;
        case 1: setAll(0,k,0,indexled); break;
        case 2: setAll(0,0,k,indexled); break;
      }
      showStrip(indexled);
      delay(3);
    }
    // Fade OUT
    for(int k = 255; k >= 0; k--) { 
      switch(j) { 
        case 0: setAll(k,0,0,indexled); break;
        case 1: setAll(0,k,0,indexled); break;
        case 2: setAll(0,0,k,indexled); break;
      }
      showStrip(indexled);
      delay(3);
    }
  }
}

void Strobe(byte red, byte green, byte blue, int StrobeCount, int FlashDelay, int EndPause, byte indexled){
  for(int j = 0; j < StrobeCount; j++) {
    setAll(red,green,blue,indexled);
    showStrip(indexled);
    delay(FlashDelay);
    setAll(0,0,0,indexled);
    showStrip(indexled);
    delay(FlashDelay);
  }
 
 delay(EndPause);
}


void BouncingColoredBalls(int BallCount, byte colors[][3], byte indexled) {
  float Gravity = -9.81;
  int StartHeight = 1;
  
  float Height[BallCount];
  float ImpactVelocityStart = sqrt( -2 * Gravity * StartHeight );
  float ImpactVelocity[BallCount];
  float TimeSinceLastBounce[BallCount];
  int   Position[BallCount];
  long  ClockTimeSinceLastBounce[BallCount];
  float Dampening[BallCount];
  
  for (int i = 0 ; i < BallCount ; i++) {   
    ClockTimeSinceLastBounce[i] = millis();
    Height[i] = StartHeight;
    Position[i] = 0; 
    ImpactVelocity[i] = ImpactVelocityStart;
    TimeSinceLastBounce[i] = 0;
    Dampening[i] = 0.90 - float(i)/pow(BallCount,2); 
  }

  while (true) {
    for (int i = 0 ; i < BallCount ; i++) {
      TimeSinceLastBounce[i] =  millis() - ClockTimeSinceLastBounce[i];
      Height[i] = 0.5 * Gravity * pow( TimeSinceLastBounce[i]/1000 , 2.0 ) + ImpactVelocity[i] * TimeSinceLastBounce[i]/1000;
  
      if ( Height[i] < 0 ) {                      
        Height[i] = 0;
        ImpactVelocity[i] = Dampening[i] * ImpactVelocity[i];
        ClockTimeSinceLastBounce[i] = millis();
  
        if ( ImpactVelocity[i] < 0.01 ) {
          ImpactVelocity[i] = ImpactVelocityStart;
        }
      }
      Position[i] = round( Height[i] * (lednum[indexled] - 1) / StartHeight);
    }
  
    for (int i = 0 ; i < BallCount ; i++) {
      setPixel(Position[i],colors[i][0],colors[i][1],colors[i][2], indexled);
    }
    
    showStrip(indexled);
    setAll(0,0,0, indexled);
  }
}


void BouncingBalls(byte red, byte green, byte blue, int BallCount, byte indexled) {
  float Gravity = -9.81;
  int StartHeight = 1;
  
  float Height[BallCount];
  float ImpactVelocityStart = sqrt( -2 * Gravity * StartHeight );
  float ImpactVelocity[BallCount];
  float TimeSinceLastBounce[BallCount];
  int   Position[BallCount];
  long  ClockTimeSinceLastBounce[BallCount];
  float Dampening[BallCount];
  
  for (int i = 0 ; i < BallCount ; i++) {   
    ClockTimeSinceLastBounce[i] = millis();
    Height[i] = StartHeight;
    Position[i] = 0; 
    ImpactVelocity[i] = ImpactVelocityStart;
    TimeSinceLastBounce[i] = 0;
    Dampening[i] = 0.90 - float(i)/pow(BallCount,2); 
  }

  while (true) {
    for (int i = 0 ; i < BallCount ; i++) {
      TimeSinceLastBounce[i] =  millis() - ClockTimeSinceLastBounce[i];
      Height[i] = 0.5 * Gravity * pow( TimeSinceLastBounce[i]/1000 , 2.0 ) + ImpactVelocity[i] * TimeSinceLastBounce[i]/1000;
  
      if ( Height[i] < 0 ) {                      
        Height[i] = 0;
        ImpactVelocity[i] = Dampening[i] * ImpactVelocity[i];
        ClockTimeSinceLastBounce[i] = millis();
  
        if ( ImpactVelocity[i] < 0.01 ) {
          ImpactVelocity[i] = ImpactVelocityStart;
        }
      }
      Position[i] = round( Height[i] * (lednum[indexled] - 1) / StartHeight);
    }
  
    for (int i = 0 ; i < BallCount ; i++) {
      setPixel(Position[i],red,green,blue,indexled);
    }
    
    showStrip(indexled);
    setAll(0,0,0,indexled);
  }
}

void colorWipe(byte red, byte green, byte blue, int SpeedDelay, byte indexled) {
  for(uint16_t i=0; i<lednum[indexled]; i++) {
      setPixel(i, red, green, blue,indexled);
      showStrip(indexled);
      delay(SpeedDelay);
  }
}


void Fire(int Cooling, int Sparking, int SpeedDelay, byte indexled) {
  static byte heat[LED1_NUM];
  int cooldown;
  
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < lednum[indexled]; i++) {
    cooldown = random(0, ((Cooling * 10) / lednum[indexled]) + 2);
    
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
  
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= lednum[indexled] - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
    
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < lednum[indexled]; j++) {
    setPixelHeatColor(j, heat[j], indexled );
  }

  showStrip(indexled);
  delay(SpeedDelay);
}

void setPixelHeatColor (int Pixel, byte temperature, byte indexled) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);
 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixel(Pixel, 255, 255, heatramp, indexled);
  } else if( t192 > 0x40 ) {             // middle
    setPixel(Pixel, 255, heatramp, 0, indexled);
  } else {                               // coolest
    setPixel(Pixel, heatramp, 0, 0, indexled);
  }
}


void HalloweenEyes(byte red, byte green, byte blue, 
                   int EyeWidth, int EyeSpace, 
                   boolean Fade, int Steps, int FadeDelay,
                   int EndPause, byte indexled){
  randomSeed(100);
  
  int i;
  int StartPoint  = random( 0, lednum[indexled] - (2*EyeWidth) - EyeSpace );
  int Start2ndEye = StartPoint + EyeWidth + EyeSpace;
  
  for(i = 0; i < EyeWidth; i++) {
    setPixel(StartPoint + i, red, green, blue, indexled);
    setPixel(Start2ndEye + i, red, green, blue, indexled);
  }
  
  showStrip(indexled);
  
  if(Fade==true) {
    float r, g, b;
  
    for(int j = Steps; j >= 0; j--) {
      r = j*(red/Steps);
      g = j*(green/Steps);
      b = j*(blue/Steps);
      
      for(i = 0; i < EyeWidth; i++) {
        setPixel(StartPoint + i, r, g, b, indexled);
        setPixel(Start2ndEye + i, r, g, b, indexled);
      }
      
      showStrip(indexled);
      delay(FadeDelay);
    }
  }
}


void rainbowCycle(int SpeedDelay, byte indexled, byte lednum) {
  byte *c;
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< lednum; i++) {
      c=Wheel(((i * 256 / lednum) + j) & 255);
      setPixel(i, *c, *(c+1), *(c+2),indexled);
    }
    showStrip(indexled);
    delay(SpeedDelay);
  }
}

byte * Wheel(byte WheelPos) {
  static byte c[3];
  
  if(WheelPos < 85) {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  } else {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }

  return c;
}


void TwinkleRandom(int Count, int SpeedDelay, boolean OnlyOne, byte indexled, byte lednum) {
  setAll(0,0,0,indexled);
  
  for (int i=0; i<Count; i++) {
     setPixel(random(lednum),random(0,255),random(0,255),random(0,255),indexled);
     showStrip(indexled);
     delay(SpeedDelay);
     if(OnlyOne) { 
       setAll(0,0,0,indexled); 
     }
   }
  
  delay(SpeedDelay);
}


void RunningLights(byte red, byte green, byte blue, int WaveDelay, byte indexled, byte lednum) {
  int Position=0;
  
  for(int i=0; i<lednum*2; i++)
  {
      Position++; // = 0; //Position + Rate;
      for(int i=0; i<lednum; i++) {
        // sine wave, 3 offset waves make a rainbow!
        //float level = sin(i+Position) * 127 + 128;
        //setPixel(i,level,0,0);
        //float level = sin(i+Position) * 127 + 128;
        setPixel(i,((sin(i+Position) * 127 + 128)/255)*red,
                   ((sin(i+Position) * 127 + 128)/255)*green,
                   ((sin(i+Position) * 127 + 128)/255)*blue,indexled);
      }
      
      showStrip(indexled);
      delay(WaveDelay);
  }
}

void SnowSparkle(byte red, byte green, byte blue, int SparkleDelay, int SpeedDelay, byte indexled, byte lednum) {
  setAll(red,green,blue,indexled);
  
  int Pixel = random(lednum);
  setPixel(Pixel,0xff,0xff,0xff,indexled);
  showStrip(indexled);
  delay(SparkleDelay);
  setPixel(Pixel,red,green,blue,indexled);
  showStrip(indexled);
  delay(SpeedDelay);
}

void Sparkle(byte red, byte green, byte blue, int SpeedDelay, byte indexled, byte lednum) {
  int Pixel = random(lednum);
  setPixel(Pixel,red,green,blue,indexled);
  showStrip(indexled);
  delay(SpeedDelay);
  setPixel(Pixel,0,0,0,indexled);
}


void theaterChaseRainbow(int runnum, byte dir, int SpeedDelay, byte indexled, byte lednum) {
  byte *c;
  
  for (int j=0; j < runnum; j++) {     // cycle all 256 colors in the wheel
    if(dir==1) {
      for (int q=0; q < 3; q++) {
          for (int i=0; i < lednum; i=i+3) {
            c = Wheel2( (i+j) % 255);
            setPixel(i+q, *c, *(c+1), *(c+2), indexled);    //turn every third pixel on
          }
          showStrip(indexled);
         
          delay(SpeedDelay);
         
          for (int i=0; i < lednum; i=i+3) {
            setPixel(i+q, 0,0,0,indexled);        //turn every third pixel off
          }
      }
    } else {
      for (int q=3; q >= 0; q--) {
          for (int i=0; i < lednum; i=i+3) {
            c = Wheel2( (i+j) % 255);
            setPixel(i+q, *c, *(c+1), *(c+2), indexled);    //turn every third pixel on
          }
          showStrip(indexled);
         
          delay(SpeedDelay);
         
          for (int i=0; i < lednum; i=i+3) {
            setPixel(i+q, 0,0,0,indexled);        //turn every third pixel off
          }
      }    
    }
  }
}

byte * Wheel2(byte WheelPos) {
  static byte c[3];
  
  if(WheelPos < 85) {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  } else {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }

  return c;
}

void theaterChase(int runnum, byte dir, byte red, byte green, byte blue, int SpeedDelay, byte indexled, byte lednum) {
  for (int j=0; j<runnum; j++) {  //do 10 cycles of chasing
    if(dir==1) {
      for (int q=0; q < 3; q++) {
        for (int i=0; i < lednum; i=i+3) {
          setPixel(i+q, red, green, blue, indexled);    //turn every third pixel on
        }
        showStrip(indexled);
       
        delay(SpeedDelay);
       
        for (int i=0; i < lednum; i=i+3) {
          setPixel(i+q, 0,0,0, indexled);        //turn every third pixel off
        }
      }
    } else {
       for (int q=3; q >=0; q--) {
        for (int i=0; i < lednum; i=i+3) {
          setPixel(i+q, red, green, blue, indexled);    //turn every third pixel on
        }
        showStrip(indexled);
       
        delay(SpeedDelay);
       
        for (int i=0; i < lednum; i=i+3) {
          setPixel(i+q, 0,0,0, indexled);        //turn every third pixel off
        }
      }     
    }
  }
}


// moves a noise up and down while slowly shifting to the side
void example_1(byte indexled, byte lednum) {

  uint8_t scale = 1000;                               // the "zoom factor" for the noise

  for (uint16_t i = 0; i < lednum; i++) {

    uint16_t shift_x = beatsin8(17);                  // the x position of the noise field swings @ 17 bpm
    uint16_t shift_y = millis() / 100;                // the y position becomes slowly incremented

    uint32_t real_x = (x[i] + shift_x) * scale;       // calculate the coordinates within the noise field
    uint32_t real_y = (y[i] + shift_y) * scale;       // based on the precalculated positions

    uint8_t noise = inoise16(real_x, real_y, 4223) >> 8;           // get the noise data and scale it down

    uint8_t index = noise * 3;                        // map led color based on noise data
    uint8_t bri   = noise;

    CRGB color = CHSV( index, 255, bri);
    leds[indexled][i] = color;
  }
}

// just moving along one axis = "lavalamp effect"
void example_2(byte indexled, byte lednum) {

  uint8_t scale = 1000;                               // the "zoom factor" for the noise

  for (uint16_t i = 0; i < lednum; i++) {

    uint16_t shift_x = millis() / 10;                 // x as a function of time
    uint16_t shift_y = 0;

    uint32_t real_x = (x[i] + shift_x) * scale;       // calculate the coordinates within the noise field
    uint32_t real_y = (y[i] + shift_y) * scale;       // based on the precalculated positions

    uint8_t noise = inoise16(real_x, real_y, 4223) >> 8;           // get the noise data and scale it down

    uint8_t index = noise * 3;                        // map led color based on noise data
    uint8_t bri   = noise;

    CRGB color = CHSV( index, 255, bri);
    leds[indexled][i] = color;
  }
}

// no x/y shifting but scrolling along z
void example_3(byte indexled, byte lednum) {

  uint8_t scale = 1000;                               // the "zoom factor" for the noise

  for (uint16_t i = 0; i < lednum; i++) {

    uint16_t shift_x = 0;                             // no movement along x and y
    uint16_t shift_y = 0;


    uint32_t real_x = (x[i] + shift_x) * scale;       // calculate the coordinates within the noise field
    uint32_t real_y = (y[i] + shift_y) * scale;       // based on the precalculated positions
    
    uint32_t real_z = millis() * 20;                  // increment z linear

    uint8_t noise = inoise16(real_x, real_y, real_z) >> 8;           // get the noise data and scale it down

    uint8_t index = noise * 3;                        // map led color based on noise data
    uint8_t bri   = noise;

    CRGB color = CHSV( index, 255, bri);
    leds[indexled][i] = color;
  }
}

void ExampleTotal(byte indexled, byte lednum)
{
    for (uint16_t i = 0; i < 5000; i++) {
    example_1(indexled, lednum);
    showStrip(indexled);
    //FastLED.show();
  }

  for (uint16_t i = 0; i < 5000; i++) {
    example_2(indexled, lednum);
    showStrip(indexled);
    //FastLED.show();
  }

  for (uint16_t i = 0; i < 5000; i++) {
    example_3(indexled, lednum);
    showStrip(indexled);
    //FastLED.show();
  }
}

void soundmems(byte indexled, byte lednum) {
  int p, q, r;

  for (int i = 0; i<lednum; i++) {
    p = analogRead(MIC_PIN);                                  // Raw reading from mic
    q = abs(p - 512 - DC_OFFSET);                             // Center on zero
    r = qsuba(q, wavebright);                                 // Get rid of noise
    Serial.print(p);
    Serial.print(" ");
    Serial.print(q);
    Serial.print(" ");
    Serial.print(r);
    Serial.println("");
     
    //printf("%6d  %6d  %6d", p, q, r);                       // Nice debug output.
    
    leds[indexled][i] = CHSV((r*4 % 255), 255, (r*2)% 255);             // Amplify it and use it as hue and brightness.
    
  }
  showStrip(indexled);
} // soundmems()

#define maxsteps 16

void ripple(byte indexled, byte lednum, uint8_t myfade) {
  uint8_t colour;                                               // Ripple colour is randomized.
  int center = 0;                                               // Center of the current ripple.
  int step = -1;                                                // -1 is the initializing
  uint8_t bgcol = 0;  

  for (int i = 0; i < lednum; i++) 
    leds[indexled][i] = CHSV(bgcol++, 255, 15);  // Rotate background colour.

  switch (step) {

    case -1:                                                          // Initialize ripple variables.
      center = random(lednum);
      colour = random8();
      step = 0;
      break;

    case 0:
      leds[indexled][center] = CHSV(colour, 255, 255);                          // Display the first pixel of the ripple.
      step ++;
      break;

    case maxsteps:                                                    // At the end of the ripples.
      step = -1;
      break;

    default:                                                             // Middle of the ripples.
      leds[indexled][(center + step + lednum) % lednum] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller
      leds[indexled][(center - step + lednum) % lednum] += CHSV(colour, 255, myfade/step*2);
      step ++;                                                         // Next step.
      break;  
  } // switch step
} // ripple()

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
