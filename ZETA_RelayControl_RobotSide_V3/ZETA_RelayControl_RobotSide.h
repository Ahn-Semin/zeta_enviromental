 /* 
  *       Author         Ahn semin
  *       Created        2020.05.07
  *       Last modified  2020.06.09 17:07
  *       Description    Relay switch control for Autonomous charging(Robot side Arduino NANO)
  */

#ifndef _ZETA_RELAYCONTROL_ROBOTSIDE_H_
#define _ZETA_RELAYCONTROL_ROBOTSIDE_H_

/*      Pin configuration       */

// BT 
#define rxPIN                 19
#define txPIN                 18

// Relay switch
#define RELAYP_PIN            6
#define RELAYN_PIN            7


/*                              */
typedef bool flag_t;
#define SERIAL_SPEED          115200
#define BUFFER_SIZE           64
#define CALLBACK_PERIOD       1000       // ms
#define ONE_SEC               1000

// BT variables
#define CONTACT                         'c'
#define FULLY_CHARGED                   'f'
#define PROXIMITY                       'p'
#define RESET                           'r'
#define START                           's'
#define OK                              'o'
#define CHARGING                        'i'
#define CONTACTLEFT                     'L'
#define CONTACTRIGHT                    'R'

// Relay variables
#define BATTERY_ON                      LOW
#define ARDUINO_ON                      HIGH


#define MAX_TIMER                       10
#define MAX_FLAG                        16

#include <ros.h>
#include <std_msgs/String.h>
#include <Timer.h>
//#include <SoftwareSerial.h>
#include <NewPing.h>

// ROS variables
ros::NodeHandle   nh;
std_msgs::String  ChargingPubMsg;
String            ChargingSubMsg;

ros::Publisher    ChargingPub("charge_state_INO",&ChargingPubMsg);

// ROS subscribe callback func.
void ChargingSubCallback(const std_msgs::String& msg);

ros::Subscriber<std_msgs::String> ChargingSub("charge_state_NUC",&ChargingSubCallback);

// 

Timer tTimer[MAX_TIMER];

// 

// BT
//SoftwareSerial BTSerial(rxPIN,txPIN);
HardwareSerial* BTSerial = &Serial2;
void ROSPublishing(String sendmsg);

void(* resetFunc) (void) = 0; // software reset
#endif
// EOF
