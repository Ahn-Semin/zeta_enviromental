/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
#include "DHT.h"
#include <ros.h>
#include <std_msgs/String.h>

#define DHTTYPE   DHT22

void airOpCallback(const std_msgs::String& str_msg);


ros::NodeHandle  nh;
std_msgs::String str_msg;
std_msgs::String air_msg;

ros::Publisher chatter("chatter", &str_msg);
ros::Publisher air_pub("air", &air_msg);
ros::Subscriber<std_msgs::String> led_sub("air_op", airOpCallback);

char hello[13] = "hello world!";

// Air sensor of Zetabank



//---------- pin config ----------

const int led_pin =  13;      // the number of the LED pin
const int shaomi_switch_pin =  12;      // shaomi control switch on D12
const int ssr_pin =  11;      // additional LED on D11
const int pwm_vo_pin  = 8;   // pin D13 (Uno:D8) pin D8/D13 is not pwm generating, but receiving is ok.
const int dht_pin = 7;      // pin D12 (Uno:D7)

float hum;    // in percent
float temp;   // in Celcius


//dust
unsigned long pulse = 0;
float ugm3 = 0;
//radon
float radon;
char rec_data[50];

DHT dht( dht_pin, DHTTYPE );


//-----------air setting
#define PULSE_DELAY         100       // msec, low pulse delay config
#define POWEROFF_DELAY  3000     // msec, power off pulse delay config
#define PRE_DELAY             50        // msec, pre delay before give pulse
#define INTERVAL_DELAY    200      // msec, delay between pulse

#define MODEPOWEROFF  0
#define MODEPOWERON  1
#define MODEAUTO          1
#define MODESLEEP         2
#define MODETURBO        3

int g_iMode = MODESLEEP;
int g_iPrevMode = MODEAUTO;
int g_pMode = MODEPOWERON;

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(air_pub);
  nh.subscribe(led_sub);

    // set the digital pin as output:
  pinMode(led_pin, OUTPUT);
  pinMode(shaomi_switch_pin, OUTPUT);
  pinMode(ssr_pin, OUTPUT);
  pinMode( pwm_vo_pin, INPUT );
  digitalWrite(ssr_pin, LOW);

  dht.begin();
  delay(2000);      // delay so DHT-22 sensor can stabilize
  //digitalWrite(shaomi_switch_pin, LOW);   // when ssr On, switch pin should not be HIGH
  digitalWrite(ssr_pin, HIGH);
  delay(5000);
  delay(3000);
  digitalWrite(shaomi_switch_pin, HIGH);

 
      // just power-on
}

void loop()
{
 
 
  if(nh.connected() )
  {

    str_msg.data = hello;
    chatter.publish( &str_msg );

    air_condition_check();
    delay(500);
  }
  else{
   // givePulse(0);
  }
  nh.spinOnce();
 
}

void airOpCallback (const std_msgs::String& str_msg){
  const char* total_led = str_msg.data;
  char sbin[20] = "";
  nh.loginfo("airopCallback");
  sprintf(sbin, "air op mode : %s", total_led);
  nh.loginfo(sbin);
  if( total_led[1] == '0'){
    nh.loginfo("power off");
    setPowerOff();
  }else if ( total_led[1] == '1'){
    nh.loginfo("sleep");
    givePulse(1);
  }
}

void air_condition_check()
{
  char t_air_msg[60] = "";
  char temp_msg[8] = "";
  pulse = pulseIn( pwm_vo_pin, LOW, 20000 );
  ugm3 = pulse2ugm3( pulse );

  temp = dht.readTemperature();
  hum =  dht.readHumidity();
  /*
  int rec_size = Serial2.available();

  if (rec_size > 0)
  {
    for (int i = 0; i < rec_size; i++)
    {
      rec_data[i] = Serial2.read();
      //Serial.print(rec_data[i]);
    }
  }
  */
 /*                                    
  String s_temp;
  s_temp = String(ugm3);
  s_temp.toCharArray(temp_msg, 8);
  sprintf(t_air_msg,"%s|",temp_msg);

   
  s_temp = String(temp);
  s_temp.toCharArray(temp_msg, 8);  
  sprintf(t_air_msg + strlen(t_air_msg),"%s|",temp_msg);

  s_temp = String(hum);
  s_temp.toCharArray(temp_msg, 8);  
  sprintf(t_air_msg + strlen(t_air_msg),"%s",temp_msg);
 
*/
 
  Serial.print( "Humid: "); Serial.print( hum );  Serial.println( " %");
  rec_data[7]= 0x00;
  sprintf(t_air_msg, "%d|%d|%d|0.13",(int) ugm3, (int)temp, (int)hum); //rec_data+3 );
  nh.loginfo(t_air_msg);
  air_msg.data = t_air_msg;         //echo는 10진수, bin은 2진수문자열
  air_pub.publish(&air_msg);
}


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
  if(g_iMode == MODESLEEP){
    for (int i=0; i<2; i++) {
      digitalWrite(shaomi_switch_pin, LOW);
      delay(PULSE_DELAY);
      digitalWrite(shaomi_switch_pin, HIGH);
      delay(INTERVAL_DELAY);
      g_iMode = MODETURBO;  
    }
  }else if (g_iMode = MODETURBO){
    digitalWrite(shaomi_switch_pin, LOW);
      delay(PULSE_DELAY);
      digitalWrite(shaomi_switch_pin, HIGH);
      delay(INTERVAL_DELAY);
      g_iMode = MODESLEEP;  
  }
}


void setPowerOff()
{
  if (g_pMode == MODEPOWERON) {    // do nothing when already in PowerOff mode.
    digitalWrite(shaomi_switch_pin, LOW);
    delay(POWEROFF_DELAY);    // for 3 sec
    digitalWrite(shaomi_switch_pin, HIGH);

    g_iPrevMode = g_iMode;      // save mode when power-off. as the mode is recovered when power-on
    //digitalWrite(ssr_pin, LOW);
    g_pMode = MODEPOWEROFF;
   
  }else if (g_pMode == MODEPOWEROFF){
   
    digitalWrite(shaomi_switch_pin, LOW);
    delay(POWEROFF_DELAY);    // for 3 sec
    digitalWrite(shaomi_switch_pin, HIGH);
    g_pMode = MODEPOWERON;
  }
}
