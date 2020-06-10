//
// JETTABOT-PWR 1.0 for JETTABOT-PWR Board
//
// Copyright (c) 2018, robotmaker@robotnmore.com
// All right reserved.
//
// This file is licenced under a Creative Commons license:
// http://creativecommons.org/licenses/by/4.0/
//
#include <Timer.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>

#define MAX_BUF 4

#define ON 1
#define OFF 0

#define ENABLE_5V 2
#define ENABLE_24V 6
#define ENABLE_12V 7

#define OD1_ENABLE 3
#define OD2_ENABLE 4
#define CONEXANT_ENABLE 5

#define LCD24_ENABLE 8
#define LCD10_ENABLE 9
#define OPENCR_ENABLE 10
#define LIDAR_ENABLE 11
#define NUC_ENABLE 12

#define STATUS_LED 13

int16_t pre_battery = 0;
int16_t battery_buf = 0;
int16_t battery_pub_cnt = 0;

ros::NodeHandle  nh;
std_msgs::String battery;
ros::Publisher bat_pub("battery", &battery);

#define DEBUG 0


int Batt_Mon = A0;    // select the input pin for the potentiometer
int AD_Battery = 0;

byte g_index = 0;
byte g_buf[MAX_BUF];
byte g_is_recv = 0;

byte ADDR_DATA=2;
byte CMD_DATA=10;
byte BATT_DATA=0;

void SendPacket(byte data){
 
  Serial.print(0xFE, HEX); 
  Serial.print(0x52, HEX); 
  Serial.print(data, HEX); 
  Serial.print(0xFF-data, HEX); 
}

#if DEBUG
void serialEvent(){

  byte buffer = Serial.read();
  //Serial.print(g_index);
  //Serial.println(buffer); 


  if ((g_is_recv == 0) && ( buffer == 0xFF)) {
    g_index = 0;
    g_is_recv = 1;
  }
 
  g_buf[g_index] = buffer;
  g_index++;
 

  if ((g_index == MAX_BUF) && (g_buf[0] == 0xFF) && (g_buf[1] == 0x43)) {
    ADDR_DATA =  g_buf[2];
    CMD_DATA =  g_buf[3];
    //Serial.print(ADDR_DATA, HEX); 
    //Serial.print(CMD_DATA, HEX); 
    g_index = 0;
    g_is_recv = 0;
   
    if(ADDR_DATA == 0x02){
      if (CMD_DATA == 1) digitalWrite(ENABLE_5V,ON);
      if (CMD_DATA == 0) digitalWrite(ENABLE_5V,OFF);  
    }
   
    else if(ADDR_DATA == 0x03){
      if (CMD_DATA == 1) digitalWrite(OD1_ENABLE,ON);
      if (CMD_DATA == 0) digitalWrite(OD1_ENABLE,OFF);  
    }
   
    else if(ADDR_DATA == 0x04){
      if (CMD_DATA == 1) digitalWrite(OD2_ENABLE,ON);
      if (CMD_DATA == 0) digitalWrite(OD2_ENABLE,OFF);  
    }
   
    else if(ADDR_DATA == 0x05){
      if (CMD_DATA == 1) digitalWrite(CONEXANT_ENABLE,ON);
      if (CMD_DATA == 0) digitalWrite(CONEXANT_ENABLE,OFF);  
    }
   
    else if(ADDR_DATA == 0x06){
      if (CMD_DATA == 1) digitalWrite(ENABLE_24V,ON);
      if (CMD_DATA == 0) digitalWrite(ENABLE_24V,OFF);  
    }
   
    else if(ADDR_DATA == 0x07){
      if (CMD_DATA == 1) digitalWrite(ENABLE_12V,ON);
      if (CMD_DATA == 0) digitalWrite(ENABLE_12V,OFF);  
    }
   
    else if(ADDR_DATA == 0x08){
      if (CMD_DATA == 1) digitalWrite(LCD24_ENABLE,ON);
      if (CMD_DATA == 0) digitalWrite(LCD24_ENABLE,OFF);  
    }
   
    else if(ADDR_DATA == 0x09){
      if (CMD_DATA == 1) digitalWrite(LCD10_ENABLE,ON);
      if (CMD_DATA == 0) digitalWrite(LCD10_ENABLE,OFF);  
    }
   
    else if(ADDR_DATA == 0x0a){
      if (CMD_DATA == 1) digitalWrite(OPENCR_ENABLE,ON);
      if (CMD_DATA == 0) digitalWrite(OPENCR_ENABLE,OFF);  
    }
   
    else if(ADDR_DATA == 0x0b){
      if (CMD_DATA == 1) digitalWrite(LIDAR_ENABLE,ON);
      if (CMD_DATA == 0) digitalWrite(LIDAR_ENABLE,OFF);  
    }
   
    else if(ADDR_DATA == 0x0c){
      if (CMD_DATA == 1) digitalWrite(NUC_ENABLE,ON);
      if (CMD_DATA == 0) digitalWrite(NUC_ENABLE,OFF);  
    }
   
    else if(ADDR_DATA == 0x0d){
      if (CMD_DATA == 1) digitalWrite(STATUS_LED,ON);
      if (CMD_DATA == 0) digitalWrite(STATUS_LED,OFF);  
    }
   
    else if(ADDR_DATA == 0x40){
      BATT_DATA = analogRead(A0);
      SendPacket(BATT_DATA);
    }
    

  }  

}

 #endif

void setup() {
  // put your setup code here, to run once:
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(bat_pub);
    //Serial.begin(115200);
     pinMode(2,OUTPUT);
    pinMode(3,OUTPUT);
    pinMode(4,OUTPUT);
    pinMode(5,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
    pinMode(8,OUTPUT);
    pinMode(9,OUTPUT); 
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
    pinMode(12,OUTPUT);
    pinMode(13,OUTPUT);

  // put your setup code here, to initial run at Power on.:

    digitalWrite(ENABLE_5V,ON);
    delay(100);
    digitalWrite(ENABLE_12V,ON);
    delay(100);
    digitalWrite(ENABLE_24V,ON);
    delay(100);   
    digitalWrite(NUC_ENABLE,ON);
    delay(100);       
    digitalWrite(OPENCR_ENABLE,ON);
    delay(100);
    digitalWrite(OD1_ENABLE,ON);
    digitalWrite(OD2_ENABLE,ON);
    delay(100);
    digitalWrite( CONEXANT_ENABLE,ON);
    digitalWrite( LCD24_ENABLE,ON);
    delay(100);
    digitalWrite( LCD10_ENABLE,ON);
    digitalWrite( LIDAR_ENABLE,ON);
    delay(100);
    pre_battery = analogRead(A0);
    delay(100);
  // put your setup code here, to initial run at Power on.:

}

void loop() {
  // put your main code here, to run repeatedly:
  battery_buf = analogRead(A0); //Assign potval.data to be the analog values from A0.
  battery_buf = (battery_buf + pre_battery)/2;

  pre_battery = battery_buf;

  if(battery_pub_cnt >=10){
    char send_msg[10] = "";
    sprintf(send_msg, "%d", pre_battery);
    battery.data = send_msg;
    bat_pub.publish( &battery ); //Publish the data.
    battery_pub_cnt = 0;
 
   }
  battery_pub_cnt++;
  nh.spinOnce();

  delay(100);
} 
