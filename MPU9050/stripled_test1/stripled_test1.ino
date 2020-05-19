#include <RTOS.h>

#define NUMBER_OF_LEDS  200
#define LED_ARRAY_SIZE  (NUMBER_OF_LEDS*3)
#define LED_PIN         2

unsigned char ledData[LED_ARRAY_SIZE];

osThreadId thread_id_loop;
osThreadId thread_id_led;

void WS2812Write();
void SetAllRGB(unsigned char red, unsigned char green, unsigned char blue);
void SetColor(unsigned char ledNumber, unsigned char color);

unsigned char red;
unsigned char green;
unsigned char blue;

static void Thread_Loop(void const *argument)
{
  (void) argument;


  for(;;)
  {
    loop();
  }
}


void setup() 
{
  Serial.begin(115200);

  // define thread
  osThreadDef(THREAD_NAME_LOOP, Thread_Loop, osPriorityNormal, 0, 1024);
  osThreadDef(THREAD_NAME_LED,  Thread_Led,  osPriorityNormal, 0, 1024);

  // create thread
  thread_id_loop = osThreadCreate(osThread(THREAD_NAME_LOOP), NULL);
  thread_id_led  = osThreadCreate(osThread(THREAD_NAME_LED), NULL);

  // start kernel
  osKernelStart();

}

void loop() 
{
  static uint32_t cnt = 0;
  
  Serial.print("RTOS Cnt : ");
  Serial.println(cnt++);
  osDelay(100);  
}

static void Thread_Led(void const *argument)
{
  (void) argument;


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  red = 0;
  green = 0;
  blue = 0;
  for(;;)
  {
    //digitalWrite(13, !digitalRead(13));

    red += 3;
    if(red>255)
      red = 0;

    green += 2;
    if(green>255)
      green = 0;

    blue += 5;
    if(blue>255)
      blue = 0;
      
    SetAllRGB(red, green, blue);
    WS2812Write();
    
    osDelay(1000);
  }
}

void WS2812Write(void)
{
  unsigned char mask = 0x01;
  unsigned char val = 0;
  
  delay_us(50);

  for(int i=0; i<LED_ARRAY_SIZE; i++) {
    mask = 0x01;
    
    for(int j=0; j<8; j++) {
      val = ledData[i] & mask;
      if(val==1) {
          digitalWrite(LED_PIN, HIGH);
          delay_ns(800);
          digitalWrite(LED_PIN, LOW);
          delay_ns(450);
      } else {
          digitalWrite(LED_PIN, HIGH);
          delay_ns(400);
          digitalWrite(LED_PIN, LOW);
          delay_ns(850);
      }

      mask <<= 1;
      
    }
    
  }
}

void SetAllRGB(unsigned char red, unsigned char green, unsigned char blue)
{
    unsigned char idx = 0;

    for (int i=0; i<NUMBER_OF_LEDS; i++)
    {
        ledData[idx] = green;
        idx++;
        ledData[idx] = red;
        idx++;
        ledData[idx] = blue;
        idx++;
    }
}

// Write a color wheel value (hue) to a single LED.
// colour is between 0 and 255.
// 0 = Red,
// 85 = Green
// 170 = Blue
// back to 255 = red again
void SetColor(unsigned char ledNumber, unsigned char color)
{
    unsigned char red;
    unsigned char green;
    unsigned char blue;

    if (color <= 85)
    {
        red = 3*(85-color);
        green = 3*color;
        blue = 0;
    }
    else if (color <=170)
    {
        red = 0;
        green = 3*(170-color);
        blue = 3*(color-85);
    }
    else
    {
        red = 3*(color-170);
        green = 0;
        blue = 3*(255-color);
    }

    unsigned char i = 3 * ledNumber;
    ledData[i] = green;
    i++;
    ledData[i] = red;
    i++;
    ledData[i] = blue;

}
