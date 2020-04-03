/*    Dust Sensor Pin   Arduino Pin
    1. PWM-VO          D8    ---> measure pin
    2. LED-GND         GND (according to datasheet, need be separated to S-GND. so TWO PINS of Arudino GND pins are used).
    3. S-GND           GND (another)
    4. Vled            5V (150ohm resistor from VCC, 220uF capacitor to LED-GND)
    5. VCC             5V

 *  DHT22 Temperature/Humitidy Sensor
    1. VCC              5V
    2. Signal           D7   ---> measure pin
    3. NC               x
    4. GND              GND
 *
 */

#include "DHT.h";   // dependant upon Adafruit_Sensors library

//---------- defines & const ----------
#define USE_SERIAL_PLOTTER  0
#define DEBUG_MSG       1

#define PULSE_DELAY         100       // msec, low pulse delay config
#define POWEROFF_DELAY  3000     // msec, power off pulse delay config
#define PRE_DELAY             50        // msec, pre delay before give pulse
#define INTERVAL_DELAY    200      // msec, delay between pulse

#define MODEPOWEROFF  0
#define MODEAUTO          1
#define MODESLEEP         2
#define MODETURBO        3

const long interval = 1000;           // interval at which to blink (milliseconds)
const long interval2 = 300;           // interval at which to blink (milliseconds)

#define DHTTYPE   DHT22     // DHT 22 (AM2302)


#define NUM_PARALLEL_JOBS   4   // array size of tTimer[]
//- tTimer[0]
#define DUST_SENSE_PERIOD     0.5  //hz. 1s
//- tTimer[1]
#define TEMPR_SENSE_PERIOD    0.5  //hz. 1s
//- tTimer[2]
#define HUMID_SENSE_PERIOD    0.5   //hz, 1s
//- tTimer[3]
#define LED_PERIOD                   4   //hz. 250ms

//---------- pin config ----------
const int led_pin =  13;      // the number of the LED pin
const int shaomi_switch_pin =  12;      // shaomi control switch on D12
const int ssr_pin =  11;      // additional LED on D11

const int pwm_vo_pin  = 8;   // pin D8 (Mega:D8) pwm receiving is ok.
const int dht_pin = 7;           // pin D7 (Mega:D7)


//---------- global variables ----------
static uint32_t tTime[NUM_PARALLEL_JOBS];       // currently manage NUM_PARALLEL_JOBS number of timers

int g_ledState = LOW;             // ledState used to set the LED
int g_led2State = LOW;

unsigned long previousMillis = 0;        // will store last time LED was updated

float hum;    // in percent
float temp;   // in Celcius
int iterations = 5;

DHT dht( dht_pin, DHTTYPE );


//---------- setup() ----------
void setup()
{
  // set the digital pin as output:
  pinMode(led_pin, OUTPUT);
  pinMode(shaomi_switch_pin, OUTPUT);
  pinMode(ssr_pin, OUTPUT);
  pinMode( pwm_vo_pin, INPUT );

  Serial.begin( 115200  /* 9600 */ );

  // initially set ON  -> ??? Set to LOW as SSR have problem
  digitalWrite(shaomi_switch_pin, LOW);

  digitalWrite(ssr_pin, LOW);

  dht.begin();
  delay(2000);      // delay so DHT-22 sensor can stabilize
  Serial.println("setup finish");
}

//---------- global variables for loop() ----------
byte rx_byte = 0;


int g_iMode = MODEPOWEROFF;
int g_iPrevMode = MODEAUTO;     // as previous mode is recovered when power-on, keep prev mode before power-off

unsigned long pulse = 0;
float ugm3 = 0;

//---------- utility functions ----------
float pulse2ugm3( unsigned long pulse_ )
{
  float value = (pulse_ - 1350 /*1400*/) / 14.0;    // 1400 is original, but it makes some values below zero
  if ( value >= 300 ) {    // sensor can support max. 300 ug/m3
    value = 0;
  }
  return value;
}

void givePulse(int num)
{
  delay(PRE_DELAY);

  for (int i=0; i<num; i++) {
    digitalWrite(shaomi_switch_pin, LOW);
    delay(PULSE_DELAY);
    digitalWrite(shaomi_switch_pin, HIGH);
    delay(INTERVAL_DELAY);
  }
}

void setPowerOff()
{
#if DEBUG_MSG
  Serial.println( "- set Power Off" );
#endif

  if (g_iMode != MODEPOWEROFF) {    // do nothing when already in PowerOff mode.
    digitalWrite(shaomi_switch_pin, LOW);
    delay(POWEROFF_DELAY);    // for 3 sec
    digitalWrite(shaomi_switch_pin, HIGH);

    g_iPrevMode = g_iMode;      // save mode when power-off. as the mode is recovered when power-on
    Serial.print("--- before power-off, save current mode : "); Serial.println( g_iMode );
  }

  g_iMode = MODEPOWEROFF;
}

void setAutoMode()
{
#if DEBUG_MSG
  Serial.println( "- set Auto Mode" );
#endif
  if (g_iMode == MODEPOWEROFF) {
    givePulse(1);     // just power-on

    g_iMode = g_iPrevMode;
    Serial.print("--- restored previous mode : "); Serial.println( g_iMode );

    return;
  }
  else if (g_iMode == MODEAUTO) {
    givePulse(0);
  }
  else if (g_iMode == MODESLEEP) {
    givePulse(2);
  }
  else if (g_iMode == MODETURBO) {
    givePulse(1);
  }

  g_iPrevMode = g_iMode;
  g_iMode = MODEAUTO;
}

void setSleepMode()
{
#if DEBUG_MSG
    Serial.println( "- set Sleep Mode" );
#endif
  if (g_iMode == MODEPOWEROFF) {
    givePulse(1);     // just power-on

    g_iMode = g_iPrevMode;
    Serial.print("--- restored previous mode : "); Serial.println( g_iMode );

    return;
  }
  else if (g_iMode == MODEAUTO) {
    givePulse(1);
  }
  else if (g_iMode == MODESLEEP) {
    givePulse(0);
  }
  else if (g_iMode == MODETURBO) {
    givePulse(2);
  }

  g_iPrevMode = g_iMode;
  g_iMode = MODESLEEP;
}

void setTurboMode()
{
#if DEBUG_MSG
    Serial.println( "- set Turbo Mode" );
#endif
  if (g_iMode == MODEPOWEROFF) {
    givePulse(1);     // just power-on

    g_iMode = g_iPrevMode;
    Serial.print("--- restored previous mode : "); Serial.println( g_iMode );

    return;
  }
  else if (g_iMode == MODEAUTO) {
    givePulse(2);
  }
  else if (g_iMode == MODESLEEP) {
    givePulse(1);
  }
  else if (g_iMode == MODETURBO) {
    givePulse(0);
  }

  g_iPrevMode = g_iMode;
  g_iMode = MODETURBO;
}

//---------- loop() ----------
void loop()
{
  unsigned long currentMillis = millis();

  if (Serial.available()) {
    rx_byte = Serial.read();
    Serial.print("- Got command : "); Serial.write(rx_byte); Serial.println("");

    if (rx_byte == '0') {
      Serial.println("- Cmd: Power Off");
      setPowerOff();
    }
    else if (rx_byte == '1') {
      Serial.println("- Cmd: Auto mode");
      setAutoMode();
    }
    else if (rx_byte == '2') {
      Serial.println("- Cmd: Sleep mode");
      setSleepMode();
    }
    else if (rx_byte == '3') {
      Serial.println("- Cmd: Turbo mode");
      setTurboMode();
    }

    else if (rx_byte == 'p') {
      Serial.println("- Cmd: Give 1 pulse");     // give 1 pulse without changing status g_iMode
      givePulse(1);
    }
    else if (rx_byte == 's') {
      Serial.println("- Cmd: print current Mode");     // give 1 pulse without changing status g_iMode

      if (g_iMode == MODEPOWEROFF) {
        Serial.println("- Status: Power-Off Mode");
      }
      if (g_iMode == MODEAUTO) {
        Serial.println("- Status: Auto Mode");
      }
      if (g_iMode == MODESLEEP) {
        Serial.println("- Status: Sleep Mode");
      }
      if (g_iMode == MODETURBO) {
        Serial.println("- Status: Turbo Mode");
      }      
      Serial.print("    prev mode: "); Serial.println( g_iPrevMode);
    }
    else if (rx_byte == 'n') {    
      Serial.println("- ssr_pin : HIGH");
      digitalWrite(shaomi_switch_pin, LOW);   // when ssr On, switch pin should not be HIGH
      digitalWrite(ssr_pin, HIGH);
      delay(5000);
      digitalWrite(shaomi_switch_pin, HIGH);   // not sure if this line is needed
    }
    else if (rx_byte == 'm') {    
      Serial.println("- ssr_pin : LOW");
      digitalWrite(ssr_pin, LOW);
    }
    else if (rx_byte == 'h') {    
      Serial.println("[Command] 0:Power Off, 1:Auto Mode, 2: Sleep Mode, 3: Turbo Mode");
      Serial.println("  s: show current Mode, p: give one pulse, n: relay On, m: relay Off, h: this message");
    }
    else {
      Serial.println("! Unknown command");
    }
  }


  if (currentMillis - tTime[0] >= (1000 / DUST_SENSE_PERIOD) ) {
    pulse = pulseIn( pwm_vo_pin, LOW, 20000 );
    ugm3 = pulse2ugm3( pulse );

#if USE_SERIAL_PLOTTER    // to display 3 parallel values. hum and temp print duplicated mesg as not sensed often.
    Serial.print( ugm3, 4 ); Serial.print( " " );  Serial.print( hum ); Serial.print( " "); Serial.print( temp ); Serial.println( " ");
#else
    Serial.print( ugm3, 4 ); Serial.println( " ug/m3" );
#endif

    tTime[0] = currentMillis;
  }

  if (currentMillis - tTime[1] >= (1000 / TEMPR_SENSE_PERIOD) ) {
    temp = dht.readTemperature();
   
#if USE_SERIAL_PLOTTER
#else
    Serial.print( "Temp: "); Serial.print( temp ); Serial.println( " C" );
#endif

    tTime[1] = currentMillis;
  }
 
  if (currentMillis - tTime[2] >= (1000 / HUMID_SENSE_PERIOD) ) {
    hum =  dht.readHumidity();

#if USE_SERIAL_PLOTTER
#else
    Serial.print( "Humid: "); Serial.print( hum );  Serial.println( " %");
#endif
   
    tTime[2] = currentMillis;
  }
  if (currentMillis - tTime[3] >= (1000 / LED_PERIOD) ) {   // built-in default LED control
    // if the LED is off turn it on and vice-versa:
    if (g_ledState == LOW) {
      g_ledState = HIGH;
    } else {
      g_ledState = LOW;
    }
    // set the LED with the g_ledState of the variable:
    digitalWrite(led_pin, g_ledState);
    // save the last time you blinked the LED
    tTime[3] = currentMillis;
  }

  delay(10);    // delay for the whole loop
}

/***** EOF *****/
