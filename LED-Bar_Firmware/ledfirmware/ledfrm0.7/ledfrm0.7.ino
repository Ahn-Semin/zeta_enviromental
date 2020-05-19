#include "FastLED.h"
//#include <j2thread.h>

#define NUM_STRIPS 5

 #define LED1_NUM   300
 #define LED2_NUM   10
 #define LED3_NUM   10
 #define LED4_NUM   6
 #define LED5_NUM   6
 
// How many leds are connected?
#define NUMLEDS 300
 
// Define the Pin
#define DATA_PIN1 6
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

 #define TRUE   1
 #define FALSE  0

#define qsubd(x, b)  ((x>b)?wavebright:0)                     // A digital unsigned subtraction macro. if result <0, then => 0. Otherwise, take on fixed value.
#define qsuba(x, b)  ((x>b)?x-b:0)                            // Analog Unsigned subtraction macro. if result <0, then => 0
// Initialize global variables for sequences
int wavebright = 10;

long firstval = 0xff00ff;
CRGB rgbval(50,0,500);
CHSV hsvval(150,255,200);

#define COLOR_ORDER BGR                                       // It's GRB for WS2812 and BGR for APA102.
#define LED_TYPE WS2812                                       // Using APA102, WS2812, WS2801. Don't forget to change 

byte r,g,b,dt;
byte r2,g2,b2;
byte inc, dec;
byte es, sd, rd;
byte rnum, tdelay, fdelay, ledn;
byte eyesize, sdelay, rdelay;
byte scount, epause, bcount;
byte cooling, sparking;
byte dir;

byte cnt = 0;
byte buff[20] = {0};

byte runcmd[5];
byte b_RStart = FALSE;
byte b_ROK = FALSE;
byte rdata;

unsigned int ledreset_cnt = 0;
  
void setup() { 
   //portBASE_TYPE s1, s2;
   
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
    
  delay(1000);
  
  Serial.println("Connect software serail!!\n");

  cnt = 0;
  b_RStart = FALSE;
  b_ROK = FALSE;
  
}

#if 1
void loop() { 
  val1 = digitalRead(READ_PIN1);
  val2 = digitalRead(READ_PIN2);

  if(!val1) {
    Serial.println("Touch1");
    //StrobeAll(0xff, 0xff, 0xff, 10, 50, 255);
    //SparkleAll(0xff, 0xff, 0xff, 5, 5);
  }

  if(!val2) {
    Serial.println("Touch2");
    //RunningLightsAll(0x00,0xaa,0xaa, 50);
      //NewKITTAll3(0xff, 0, 0, 3, 30, 50);
  }

  FadeInOut(0, 0, 255, 1, 100);
   
  if(Serial.available()) {
    rdata = Serial.read();
    //buff[cnt] = Serial.read();
    if((rdata==0xAA) && (b_RStart==FALSE)) {
      b_RStart = TRUE;      
    } else if((b_RStart==TRUE) && (rdata==0x55) && cnt>=9) {
       b_ROK = TRUE;
       b_RStart = FALSE;
    } else {      
      buff[cnt++] = rdata;
    }
  }

  if(b_ROK) {
    if(cnt>=10) {
      b_ROK = FALSE;
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
          sd= buff[4];
          rd= buff[5];
          CenterToOutsideAll(r, g, b, sd, rd);
          break;
  
        case 6:
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
  
        case 7:
          r= buff[1];
          g= buff[2];
          b= buff[3];
          sd= buff[4];
          rd= buff[5];
          OutsideToCenterAll(r, g, b, sd, rd);
          break;
                  
        case 8:
          r= buff[1];
          g= buff[2];
          b= buff[3];
          r2= buff[4];
          g2= buff[5];
          b2= buff[6];       
          sd= buff[7];
          rd= buff[8];
          OutsideToCenterAll2(r, g, b, r2, g2, b2, sd, rd);
          break;
  
       case 9:
          r= buff[1];
          g= buff[2];
          b= buff[3];   
          sd= buff[4];
          rd= buff[5];
          LeftToRightAll(r, g, b, sd, rd);
          break;  
          
        case 10:
          r= buff[1];
          g= buff[2];
          b= buff[3];
          r2= buff[4];
          g2= buff[5];
          b2= buff[6];       
          sd= buff[7];
          rd= buff[8];
          LeftToRightAll2(r, g, b, r2, g2, b2, sd, rd);
          break;    
  
        case 11:
          r= buff[1];
          g= buff[2];
          b= buff[3];      
          sd= buff[4];
          rd= buff[5];
          RightToLeftAll(r, g, b, sd, rd);
          break;   
  
       case 12:
          r= buff[1];
          g= buff[2];
          b= buff[3];
          r2= buff[4];
          g2= buff[5];
          b2= buff[6];       
          sd= buff[7];
          rd= buff[8];
          RightToLeftAll2(r, g, b, r2, g2, b2, sd, rd);
          break;                      
  
       case 13:
          r= buff[1];
          g= buff[2];
          b= buff[3];
          sd= buff[4];
          rd= buff[5];
          NewKITTAll(r, g, b, sd, rd);
          break;                      
  
       case 14:
          rnum= buff[1];
          tdelay= buff[2];
          fdelay= buff[3];
          ledn= buff[4];
          mover(rnum, tdelay, fdelay, ledn);
          break;             
  
       case 15:
          r= buff[1];
          g= buff[2];
          b= buff[3];
          eyesize= buff[4];
          sdelay= buff[5];
          rdelay= buff[6];
          rnum = buff[7];
          CylonBounceAll(rnum, r, g, b, eyesize, sdelay, rdelay);
          break;             
  
        case 16:
          r= buff[1];
          g= buff[2];
          b= buff[3];       
          eyesize= buff[4];
          sd= buff[5];
          rd= buff[6];
          CenterToOutsideAll3(r, g, b, eyesize, sd, rd);
          break;
  
        case 17:
          r= buff[1];
          g= buff[2];
          b= buff[3];       
          eyesize= buff[4];
          sd= buff[5];
          rd= buff[6];
          OutsideToCenterAll3(r, g, b, eyesize, sd, rd);
          break;
  
       case 18:
          r= buff[1];
          g= buff[2];
          b= buff[3];   
          eyesize= buff[4];
          sd= buff[5];
          rd= buff[6];
          LeftToRightAll3(r, g, b, eyesize, sd, rd);
          break;   
  
       case 19:
          r= buff[1];
          g= buff[2];
          b= buff[3];   
          eyesize= buff[4];
          sd= buff[5];
          rd= buff[6];
          RightToLeftAll3(r, g, b, eyesize, sd, rd);
          break;   
  
       case 20:
          r= buff[1];
          g= buff[2];
          b= buff[3];   
          eyesize= buff[4];
          sd= buff[5];
          rd= buff[6];
          NewKITTAll3(r, g, b, eyesize, sd, rd);
          break;          
  
        case 21:
          r= buff[1];
          g= buff[2];
          b= buff[3];   
          scount= buff[4];
          fdelay= buff[5];
          epause= buff[6];
          StrobeAll(r, g, b, scount, fdelay, epause);
          break;     
  
       case 22:
          r= buff[1];
          g= buff[2];
          b= buff[3];   
          bcount= buff[4];
          rnum= buff[5];
          BouncingBallsAll(r, g, b, bcount, rnum);
          break;  
  
       case 23:
          r= buff[1];
          g= buff[2];
          b= buff[3];   
          sdelay= buff[4];
          colorWipeAll(r, g, b, sdelay);
          break;          
  
       case 24:
          rnum= buff[1];
          cooling= buff[2];
          sparking= buff[3];   
          sdelay= buff[4];
          FireAll(rnum, cooling, sparking, sdelay);
          break;
  
       case 25:
          rnum= buff[1];
          sdelay= buff[2];
          rainbowCycleAll(rnum, sdelay);
          break;
  
       case 26:
          r= buff[1];
          g= buff[2];
          b= buff[3];   
          sdelay= buff[4];
          RunningLightsAll(r, g, b, sdelay);
          break;       
  
       case 27:
          r= buff[1];
          g= buff[2];
          b= buff[3];   
          rnum= buff[4];   
          sdelay= buff[5];
          SparkleAll(r, g, b, rnum, sdelay);
          break;               
  
       case 28:
          rnum= buff[1];
          dir= buff[2];
          sdelay= buff[3];
          theaterChaseRainbowAll(rnum, dir, sdelay);
          break;  
  
       case 29:
          rnum= buff[1];
          dir= buff[2];
          r= buff[3];
          g= buff[4];
          b= buff[5];          
          sdelay= buff[6];
          theaterChaseAll(rnum, dir, r, g, b, sdelay);
          break;

        case 30:
          r= buff[1];
          g= buff[2];
          b= buff[3];
          dt= buff[4];
          inc= buff[5];
          dec= buff[6];
          Serial.println("inside 30");
          FadeInOutAll(r, g, b, dt, inc, dec);
          break;          
                                                                                       
      }

      for(int k=0; k<5; k++)
        setAll(0,0,0,k);  
      
    }  else {
      b_ROK = FALSE;
      cnt = 0;
    }
  }

  ledreset_cnt++;

  if(ledreset_cnt>500) {
    ledreset_cnt = 0;

    for(int k=0; k<5; k++)
      setAll(0,0,0,k);  
  }
  delay(10);
}

#endif


void theaterChaseAll(int runnum, byte dir, byte red, byte green, byte blue, int SpeedDelay) {
  for (int j=0; j<runnum; j++) {  //do 10 cycles of chasing
    for(int k=0; k<5; k++) {
      if(dir==1) {
        for (int q=0; q < 3; q++) {
          for (int i=0; i < lednum[k]; i=i+3) {
            if((i+q)<=lednum[k])
              setPixel(i+q, red, green, blue, k);    //turn every third pixel on
          }
          showStrip(k);
         
          delay(SpeedDelay);
         
          for (int i=0; i < lednum[k]; i=i+3) {
            if((i+q)<=lednum[k])
              setPixel(i+q, 0,0,0, k);        //turn every third pixel off
          }
        }
      } else {
         for (int q=3; q >=0; q--) {
          for (int i=0; i < lednum[k]; i=i+3) {
            if((i+q)<=lednum[k])
              setPixel(i+q, red, green, blue, k);    //turn every third pixel on
          }
          showStrip(k);
         
          delay(SpeedDelay);
         
          for (int i=0; i < lednum[k]; i=i+3) {
            if((i+q)<=lednum[k])
              setPixel(i+q, 0,0,0, k);        //turn every third pixel off
          }
        }     
      }
    }
  }
  for(int k=0; k<5; k++)
    setAll(0,0,0,k);  
}


void theaterChaseRainbowAll(int runnum, byte dir, int SpeedDelay) {
  byte *c;
  
  for (int j=0; j < runnum; j++) {     // cycle all 256 colors in the wheel
    for(int k=0; k<5; k++) {
      if(dir==1) {
        for (int q=0; q < 3; q++) {
            for (int i=0; i < lednum[k]; i=i+3) {
              c = Wheel2( (i+j) % 255);
              if((i+q)<=lednum[k])
                setPixel(i+q, *c, *(c+1), *(c+2), k);    //turn every third pixel on
            }
            showStrip(k);
           
            delay(SpeedDelay);
           
            for (int i=0; i < lednum[k]; i=i+3) {
              if((i+q)<=lednum[k])
                setPixel(i+q, 0,0,0,k);        //turn every third pixel off
            }
        }
      } else {
        for (int q=3; q >= 0; q--) {
            for (int i=0; i < lednum[k]; i=i+3) {
              c = Wheel2( (i+j) % 255);
              if((i+q)<=lednum[k])
                setPixel(i+q, *c, *(c+1), *(c+2), k);    //turn every third pixel on
            }
            showStrip(k);
           
            delay(SpeedDelay);
           
            for (int i=0; i < lednum[k]; i=i+3) {
              if((i+q)<=lednum[k])
                setPixel(i+q, 0,0,0,k);        //turn every third pixel off
            }
        }    
      }
    }
  }
  for(int k=0; k<5; k++)
    setAll(0,0,0,k);    
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

void SparkleAll(byte red, byte green, byte blue, byte rnum, byte SpeedDelay) {
  for(int i=0; i<rnum; i++) {
    for(int j=0; j<5; j++) {
      int Pixel = random(lednum[j]);
      if(Pixel<=lednum[j])
        setPixel(Pixel,red,green,blue,j);
      showStrip(j);
      delay(SpeedDelay);
      if(Pixel<=lednum[j])
        setPixel(Pixel,0,0,0,j);
    }
  }
  for(int k=0; k<5; k++)
    setAll(0,0,0,k);  
}

void RunningLightsAll(byte red, byte green, byte blue, int WaveDelay) {
  int Position=0;
  
  for(int i=0; i<lednum[0]*2; i++)
  {
    for(int j=0; j<5; j++) {
        Position++; // = 0; //Position + Rate;
        for(int k=0; k<lednum[j]; k++) {
          if(k<=lednum[j])
            setPixel(k,((sin(i+Position) * 127 + 128)/255)*red,
                       ((sin(i+Position) * 127 + 128)/255)*green,
                       ((sin(i+Position) * 127 + 128)/255)*blue,j);
        }
        
        showStrip(j);
    }
      delay(WaveDelay);
  }
  for(int k=0; k<5; k++)
    setAll(0,0,0,k);    
}

void rainbowCycleAll(byte rnum, byte SpeedDelay) {
  byte *c;
  uint16_t i, j, k;

  for(j=0; j<256*rnum; j++) { // 5 cycles of all colors on wheel
    for(k=0; k<5; k++) {
      for(i=0; i< lednum[k]; i++) {
        c=Wheel(((i * 256 / lednum[k]) + j) & 255);
        if(i<=lednum[k])
          setPixel(i, *c, *(c+1), *(c+2),k);
      }
      showStrip(k);
    }
    delay(SpeedDelay);
  }
  for(int k=0; k<5; k++)
    setAll(0,0,0,k);  
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

void FireAll(byte rnum, byte Cooling, byte Sparking, byte SpeedDelay) {
  static byte heat[LED1_NUM];
  int cooldown;

for(int j=0; j<rnum; j++) {
    for(int s=0; s<5; s++) {
      // Step 1.  Cool down every cell a little
      for( int i = 0; i < lednum[s]; i++) {
        cooldown = random(0, ((Cooling * 10) / lednum[s]) + 2);
        
        if(cooldown>heat[i]) {
          heat[i]=0;
        } else {
          heat[i]=heat[i]-cooldown;
        }
      }
      
      // Step 2.  Heat from each cell drifts 'up' and diffuses a little
      for( int k= lednum[s] - 1; k >= 2; k--) {
        heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
      }
        
      // Step 3.  Randomly ignite new 'sparks' near the bottom
      if( random(255) < Sparking ) {
        int y = random(7);
        heat[y] = heat[y] + random(160,255);
        //heat[y] = random(160,255);
      }
    
      // Step 4.  Convert heat to LED colors
      for( int j = 0; j < lednum[s]; j++) {
        setPixelHeatColor(j, heat[j], s );
      }
    showStrip(s);      
  }
   
  delay(SpeedDelay);
 }

  for(int k=0; k<5; k++)
    setAll(0,0,0,k);
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


void colorWipeAll(byte red, byte green, byte blue, int SpeedDelay) {
  for(uint16_t i=0; i<lednum[0]; i++) {
    for(int j=0; j<5; j++) {
      if(i<=lednum[j])
          setPixel(i, red, green, blue,j);
        showStrip(j);
    }
    delay(SpeedDelay);
  }
}

void BouncingBallsAll(byte red, byte green, byte blue, int BallCount, byte rnum) {
  float Gravity = -9.81;
  int StartHeight = 1;
  
  float Height[BallCount];
  float ImpactVelocityStart = sqrt( -2 * Gravity * StartHeight );
  float ImpactVelocity[BallCount];
  float TimeSinceLastBounce[BallCount];
  int   Position[BallCount];
  long  ClockTimeSinceLastBounce[BallCount];
  float Dampening[BallCount];
  int rcnt = 0, num = rnum*1000;
  
  for (int i = 0 ; i < BallCount ; i++) {   
    ClockTimeSinceLastBounce[i] = millis();
    Height[i] = StartHeight;
    Position[i] = 0; 
    ImpactVelocity[i] = ImpactVelocityStart;
    TimeSinceLastBounce[i] = 0;
    Dampening[i] = 0.90 - float(i)/pow(BallCount,2); 
  }

  

  while (rcnt<rnum) {
    for(int j=0; j<5; j++) {
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
        Position[i] = round( Height[i] * (lednum[j] - 1) / StartHeight);
  
      }
    
      for (int i = 0 ; i < BallCount ; i++) {
        if(Position[i]<=lednum[j])
          setPixel(Position[i],red,green,blue,j);
      }
      
      showStrip(j);
      setAll(0,0,0,j);
      rcnt++;
    }
  }
}

void StrobeAll(byte red, byte green, byte blue, int StrobeCount, int FlashDelay, int EndPause){
  for(int j = 0; j < StrobeCount; j++) {
    for(int k=0; k<5; k++) {
      setAll(red,green,blue,k);
      showStrip(k);
      delay(FlashDelay);
      setAll(0,0,0,k);
      showStrip(k);
    }
    delay(FlashDelay);
  }
 
 delay(EndPause);
}

void NewKITTAll3(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay){
  RightToLeftAll3(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  LeftToRightAll3(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  OutsideToCenterAll3(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  CenterToOutsideAll3(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  LeftToRightAll3(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  RightToLeftAll3(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  OutsideToCenterAll3(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);
  CenterToOutsideAll3(red, green, blue, EyeSize, SpeedDelay, ReturnDelay);

  for(int k=0; k<5; k++)
      setAll(0,0,0,k);
}

void RightToLeftAll3(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay) {
  for(int i = lednum[0]-EyeSize-2; i > 0; i--) {
    for(int k=0; k<5; k++) {
      setAll(0,0,0,k);
      setPixel(i, red/10, green/10, blue/10, k);
      for(int j = 1; j <= EyeSize; j++) {
        if((i+j)<=lednum[k])
          setPixel(i+j, red, green, blue, k); 
      }
      if((i+EyeSize+1)<=lednum[k])
        setPixel(i+EyeSize+1, red/10, green/10, blue/10, k);
      showStrip(k);
    }
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}

void LeftToRightAll3(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay) {
  for(int i = 0; i < lednum[0]-EyeSize-2; i++) {
    for(int k=0; k<5; k++) {
      setAll(0,0,0,k);
      setPixel(i, red/10, green/10, blue/10, k);
      for(int j = 1; j <= EyeSize; j++) {
        if((i+j)<=lednum[k])
          setPixel(i+j, red, green, blue, k); 
      }
      if((i+EyeSize+1)<=lednum[k])
        setPixel(i+EyeSize+1, red/10, green/10, blue/10, k);
      showStrip(k);
    }
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}


void CenterToOutsideAll3(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay) {
  for(int i =((lednum[0]-EyeSize)/2); i>=0; i--) {
    for(int k=0; k<5; k++) {
      setAll(0,0,0,k);
      
      setPixel(i, red/10, green/10, blue/10, k);
      for(int j = 1; j <= EyeSize; j++) {
        setPixel(i+j, red, green, blue, k); 
      }
      if((i+EyeSize+1)<=lednum[k])
        setPixel(i+EyeSize+1, red/10, green/10, blue/10, k);

      if((lednum[k]-i) >=0)
        setPixel(lednum[k]-i, red/10, green/10, blue/10, k);
      for(int j = 1; j <= EyeSize; j++) {
        if((lednum[k]-i-j)>=0)
          setPixel(lednum[k]-i-j, red, green, blue, k); 
      }
      if((lednum[k]-i-EyeSize-1)>=0)
        setPixel(lednum[k]-i-EyeSize-1, red/10, green/10, blue/10, k);
      
      showStrip(k);
    }
    delay(SpeedDelay);
  }

  for(int k=0; k<5; k++)
      setAll(0,0,0,k);
      
  delay(ReturnDelay);
  
}

void OutsideToCenterAll3(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay) {
  for(int i = 0; i<=((lednum[0]-EyeSize)/2); i++) {
    for(int k=0; k<5; k++) {
      setAll(0,0,0,k);
      
      setPixel(i, red/10, green/10, blue/10, k);
      for(int j = 1; j <= EyeSize; j++) {
        if((i+j)<=lednum[k])
          setPixel(i+j, red, green, blue, k); 
      }
      if((i+EyeSize+1)<=lednum[k])
        setPixel(i+EyeSize+1, red/10, green/10, blue/10, k);

      if((lednum[k]-i)>=0)
        setPixel(lednum[k]-i, red/10, green/10, blue/10, k);
      for(int j = 1; j <= EyeSize; j++) {
        if((lednum[k]-i-j)>=0)
          setPixel(lednum[k]-i-j, red, green, blue, k); 
      }
      if((lednum[k]-i-EyeSize-1)>=0)
        setPixel(lednum[k]-i-EyeSize-1, red/10, green/10, blue/10, k);
      
      showStrip(k);
    }
    delay(SpeedDelay);
  }

  for(int k=0; k<5; k++)
      setAll(0,0,0,k);
        
  delay(ReturnDelay);
}

void CenterToOutside3(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay, byte indexled) {
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

void OutsideToCenter3(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay, byte indexled) {
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

void LeftToRight3(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay, byte indexled) {
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

void RightToLeft3(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay, byte indexled) {
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


void CylonBounceAll(byte cnt, byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay){

  for(int s=0; s<cnt; s++) {
    for(int i = 0; i < lednum[0]-EyeSize-2; i++) {
      for(int k=0; k<5; k++) {
        setAll(0,0,0, k);
        if(i<=lednum[k])
          setPixel(i, red/10, green/10, blue/10, k);
        for(int j = 1; j <= EyeSize; j++) {
          setPixel(i+j, red, green, blue,k); 
        }
        setPixel(i+EyeSize+1, red/10, green/10, blue/10,k);
        showStrip(k);
      }
      delay(SpeedDelay);
    }
  
    delay(ReturnDelay);
  
    for(int i = lednum[0]-EyeSize-2; i > 0; i--) {
      for(int k=0; k<5; k++) {
        setAll(0,0,0,k);
        setPixel(i, red/10, green/10, blue/10, k);
        for(int j = 1; j <= EyeSize; j++) {
          setPixel(i+j, red, green, blue, k); 
        }
        setPixel(i+EyeSize+1, red/10, green/10, blue/10,k);
        showStrip(k);
      }
      delay(SpeedDelay);
    }
    
    delay(ReturnDelay);
  }

  for(int k=0; k<5; k++)
    setAll(0,0,0,k);
  
}

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

void mover(byte cnt, byte thisdelay, byte thisfade, byte indexled) { 
  static uint8_t hue = 0;
  for(int j=0; j<cnt; j++) { 
    ChangeMe();
    for (int i = 0; i < lednum[indexled]; i++) {
      leds[indexled][i] += CHSV(hue, 255, 255);
      leds[indexled][(i+5) % lednum[indexled]] += CHSV(hue+85, 255, 255);         // We use modulus so that the location is between 0 and NUM_LEDS
      leds[indexled][(i+10) % lednum[indexled]] += CHSV(hue+170, 255, 255);       // Same here.
      show_at_max_brightness_for_power();
      fadeToBlackBy(leds[indexled], lednum[indexled], thisfade);                  // Low values = slower fade.
      delay(thisdelay);                                         // UGH!!!! A blocking delay. If you want to add controls, they may not work reliably.
    }
  }

   setAll(0,0,0,indexled);
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

void NewKITTAll(byte red, byte green, byte blue, int SpeedDelay, int ReturnDelay){
  for(int j=0; j<5; j++)
    setAll(0,0,0,j);

    RightToLeftAll(red, green, blue, SpeedDelay, ReturnDelay);
    LeftToRightAll(red, green, blue, SpeedDelay, ReturnDelay);
    OutsideToCenterAll(red, green, blue, SpeedDelay, ReturnDelay);
    CenterToOutsideAll(red, green, blue, SpeedDelay, ReturnDelay);
    LeftToRightAll(red, green, blue, SpeedDelay, ReturnDelay);
    RightToLeftAll(red, green, blue, SpeedDelay, ReturnDelay);
    OutsideToCenterAll(red, green, blue, SpeedDelay, ReturnDelay);
    CenterToOutsideAll(red, green, blue, SpeedDelay, ReturnDelay);
}

void RightToLeftAll2(byte reds, byte greens, byte blues, byte red, byte green, byte blue, int SpeedDelay, int ReturnDelay) {
  byte center;
  byte cnt[5] = {lednum[0], lednum[1], lednum[2], lednum[3], lednum[4]};
  
  for(int j=0; j<5; j++)
    setAll(reds,greens,blues,j);
      
  for(int i = 0; i <=lednum[0]; i++) {
    for(int j=0; j<5; j++) {
      if(cnt[j]>=0) {
        setPixel(cnt[j], red, green, blue, j);
        if((cnt[j]-1)>=0)
          setPixel(cnt[j]-1, red/10, green/10, blue/10, j);
        showStrip(j);
      }
      cnt[j]--;
    }
    delay(SpeedDelay);
  }
  
  delay(ReturnDelay);
}

void RightToLeftAll(byte red, byte green, byte blue, int SpeedDelay, int ReturnDelay) {
  byte center;
  byte cnt[5] = {lednum[0], lednum[1], lednum[2], lednum[3], lednum[4]};
  
  for(int i = 0; i <=lednum[0]; i++) {
    for(int j=0; j<5; j++) {
      setAll(0,0,0,j);
      if(cnt[j]>=0) {
        setPixel(cnt[j], red, green, blue, j);
        if((cnt[j]-1)>=0)
          setPixel(cnt[j]-1, red/10, green/10, blue/10, j);
        showStrip(j);
      }
      cnt[j]--;
    }
    delay(SpeedDelay);
  }

  for(int j=0; j<5; j++)
    setAll(0,0,0,j);
      
  delay(ReturnDelay);
}

void LeftToRightAll2(byte reds, byte greens, byte blues, byte red, byte green, byte blue, int SpeedDelay, int ReturnDelay) {
  byte center;
  
  for(int j=0; j<5; j++)
    setAll(reds,greens,blues,j);
      
  for(int i = 0; i < lednum[0]; i++) {
    for(int j=0; j<5; j++) {
      if(i<=lednum[j]) {
        setPixel(i, red, green, blue, j);
        if((i+1)<=lednum[j])
          setPixel(i+1, red/10, green/10, blue/10, j);
        showStrip(j);
      }
    }
    delay(SpeedDelay);
  }
  
  delay(ReturnDelay);
}

void LeftToRightAll(byte red, byte green, byte blue, int SpeedDelay, int ReturnDelay) {
  byte center;
  
  for(int i = 0; i < lednum[0]; i++) {
    for(int j=0; j<5; j++) {
      setAll(0,0,0,j);
      if(i<=lednum[j]) {
        setPixel(i, red, green, blue, j);
        if((i+1)<=lednum[j])
          setPixel(i+1, red/10, green/10, blue/10, j);
        showStrip(j);
      }
    }
    delay(SpeedDelay);
  }

  for(int j=0; j<5; j++)
    setAll(0,0,0,j);
      
  delay(ReturnDelay);
}

void OutsideToCenterAll2(byte reds, byte greens, byte blues, byte red, byte green, byte blue, int SpeedDelay, int ReturnDelay) {
  byte center;
  
  for(int j=0; j<5; j++)
    setAll(reds,greens,blues,j);

  for(int j=0; j<5; j++) {
    setPixel(0, red, green, blue, j);
    setPixel(lednum[0]-1, red, green, blue, j);
  }
  delay(SpeedDelay);

  for(int i =1; i<(lednum[0]/2); i++) {    
    for(int j=0; j<5; j++) {
      center = (lednum[j]/2);
      if((lednum[j]-i) >= center) {
        setPixel(lednum[j]-i, red, green, blue, j);
        if((lednum[j]-i-1) >= center)
          setPixel(lednum[j]-i-1, red/10, green/10, blue/10, j);
      }
        
      if(i <= center) {
        setPixel(i, red, green, blue, j);      
        if(i+1 <= center)
          setPixel(i+1, red/10, green/10, blue/10, j);
      }      
      showStrip(j);
    }
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}

void OutsideToCenterAll(byte red, byte green, byte blue, int SpeedDelay, int ReturnDelay) {
  byte center;
  
  for(int j=0; j<5; j++) {
    setAll(0,0,0,j);
    setPixel(0, red, green, blue, j);
    setPixel(lednum[0]-1, red, green, blue, j);
  }
  delay(SpeedDelay);

  for(int i =1; i<=(lednum[0]/2); i++) {    
    for(int j=0; j<5; j++) {
      setAll(0,0,0,j);
      center = (lednum[j]/2);
      if((lednum[j]-i) >= center) {
        setPixel(lednum[j]-i, red, green, blue, j);
        if((lednum[j]-i-1) >= center)
          setPixel(lednum[j]-i-1, red/10, green/10, blue/10, j);
      }
        
      if(i <= center) {
        setPixel(i, red, green, blue, j);      
        if(i+1 <= center)
          setPixel(i+1, red/10, green/10, blue/10, j);
      }      
      showStrip(j);
    }
    delay(SpeedDelay);
  }

  for(int j=0; j<5; j++)
    setAll(0,0,0,j);
  
  delay(ReturnDelay);
}

void CenterToOutsideAll2(byte reds, byte greens, byte blues, byte red, byte green, byte blue, int SpeedDelay, int ReturnDelay) {
  byte center;

  for(int j=0; j<5; j++)
    setAll(reds,greens,blues,j);

  for(int j=0; j<5; j++) {
    center = (lednum[j]/2);
    setPixel(center, red, green, blue, j);
  }
  delay(SpeedDelay);

  for(int i =1; i<=(lednum[0]/2); i++) {    
    for(int j=0; j<5; j++) {
      center = (lednum[j]/2);
      if((center+i) <= lednum[j]) {
        setPixel(center+i, red, green, blue, j);
        if((center+i+1) <= lednum[j])
          setPixel(center+i+1, red/10, green/10, blue/10, j);
      }
      if((center-i) >= 0) {
        setPixel(center-i, red, green, blue, j);
        if((center-i-1) >= 0)
          setPixel(center-i-1, red/10, green/10, blue/10, j);
      }
      showStrip(j);      
    }

    delay(SpeedDelay);
  }
  delay(ReturnDelay);
}


void CenterToOutsideAll(byte red, byte green, byte blue, int SpeedDelay, int ReturnDelay) {
  byte center;
  for(int j=0; j<5; j++) {
    setAll(0,0,0,j);
    center = (lednum[j]/2);
    setPixel(center, red, green, blue, j);
  }
  delay(SpeedDelay);

  for(int i =1; i<=(lednum[0]/2); i++) {    
    for(int j=0; j<5; j++) {
      setAll(0,0,0,j);
      center = (lednum[j]/2);
      if((center+i) <= lednum[j]) {
        setPixel(center+i, red, green, blue, j);
        if((center+i+1) <= lednum[j])
          setPixel(center+i+1, red/10, green/10, blue/10, j);
      }
      if((center-i) >= 0) {
        setPixel(center-i, red, green, blue, j);
        if((center-i-1) >= 0)
          setPixel(center-i-1, red/10, green/10, blue/10, j);
      }
      showStrip(j);
    }
    delay(SpeedDelay);
  }

  for(int j=0; j<5; j++)
    setAll(0,0,0,j);
      
  delay(ReturnDelay);
}

/*
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
}*/

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


void FadeOut2(byte reds, byte greens, byte blues, byte rede, byte greene, byte bluee, byte indexled, byte dtime, byte inc){
  float r, g, b;
  float rs, gs, bs;

  rs = (float)(reds - rede)/256.;
  gs = (float)(greens - greene)/256.;
  bs = (float)(blues - bluee)/256.;

  setAll(reds,greens,blues,indexled);
  showStrip(indexled);
  
  for(int k = 0; k <= 255; k+=inc) {
    r = (reds - rs*k);
    g = (greens - gs*k);
    b = (blues - bs*k);
    setAll(r,g,b, indexled);
    showStrip(0);
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

void FadeInOutAll(byte red, byte green, byte blue, byte dtime, byte inc, byte dec){
  float r, g, b;
      
  for(int k = 0; k < inc; k=k+1) { 
    for(int i=0; i<5; i++) {
      r = (k/256.0)*red;
      g = (k/256.0)*green;
      b = (k/256.0)*blue;
      setAll(r,g,b, i);
      showStrip(i);
    }
    delay(dtime);
  }
     
  for(int k = dec; k >= 0; k=k-2) {
    for(int i=0; i<5; i++) {
      r = (k/256.0)*red;
      g = (k/256.0)*green;
      b = (k/256.0)*blue;
      setAll(r,g,b, i);
      showStrip(i);
    }
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
