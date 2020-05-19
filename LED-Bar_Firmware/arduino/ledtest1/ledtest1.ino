#include "FastLED.h"
 
// How many leds are connected?
#define NUM_LEDS 142
 
// Define the Pins
#define DATA_PIN 2
 
// Define the array of leds
CRGB leds[NUM_LEDS];
 
// Put color values in arrays
long invader1a[] =
{
   0x008000, 0x000000, 0x000000,0x000000,0x000000,0x000000,0x000000, 0x008000,
0x008000, 0xFFFF00, 0x0000FF, 0xFFFF00, 0xFFFF00, 0x0000FF, 0xFFFF00, 0x008000,
0x008000, 0x000000, 0xFFFF00, 0x800080, 0x800080, 0xFFFF00, 0x000000, 0x008000,
0x000000, 0x000000, 0x000000, 0xFF0000, 0xFF0000, 0x000000, 0x000000, 0x000000
};
 
long invader1b[] =
{
   0x000000, 0x000000, 0x0000FF, 0xFFFF00, 0xFFFF00, 0x0000FF, 0x000000, 0x000000,
0x000000, 0x008000, 0xFFFF00, 0x800080, 0x800080, 0xFFFF00, 0x008000, 0x000000,
0x008000, 0x000000, 0x000000, 0xFFFF00, 0xFFFF00, 0x000000, 0x000000, 0x008000,
0x000000, 0x008000, 0x000000, 0xFF0000, 0xFF0000, 0x000000, 0x008000, 0x000000
};
 
 
void setup() { 
  FastLED.addLeds<NEOPIXEL,DATA_PIN>(leds, NUM_LEDS);
}
 
void loop() {  
 
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
}


