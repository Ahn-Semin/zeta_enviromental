 /* 
  *       Author         Ahn semin
  *       Created        2020.05.07
  *       Last modified  2020.05.15 18:20
  *       Description      Relay switch control for Autonomous charging(Station side Arduino Nano)
  *                      Peripheral device must be paired.
  */
#ifndef _ZETA_RELAYCONTROL_STATIONSIDE_H_
#define _ZETA_RELAYCONTROL_STATIONSIDE_H_

//#define MAC                             "24:62:AB:B2:22:96" //





// pin configuration
#define PWM_PINP                        11
#define PWM_PINN                        10
#define RX_PIN                          19  // UART1
#define TX_PIN                          18
#define RELAYP_PIN                      8
#define RELAYN_PIN                      9

#define SERIAL_SPEED                    115200

// PWM variables
#define ONE_SEC                         1000
#define PWM_DUTY                        0.5f
#define PWM_WIDTH                       255

// BT variables
#define CONTACT                         'c'
#define FULLY_CHARGED                   'f'
#define PROXIMITY                       'p'
#define RESET                           'r'
#define START                           's'
#define OK                              'o'
#define CHARGING                        'i'

// Relay variables
#define CHARGER_ON                      LOW
#define ARDUINO_ON                      HIGH

// UUIDs
#define ROS_PUB_UUID                    "8bab6c96-0238-40ad-85b4-2358049a553c"  // custom V4 UUID
#define ROS_SUB_UUID                    "abb844b1-132f-46db-b71e-4fb8d4ef63a8"
#define MSG_REC_UUID                    "810df03c-90b9-47c2-875d-88cd9ca8d9db"
#define MSG_SEND_UUID                   "61d1c3a8-ba3f-4a86-9dc5-2f4e49090a9c"
#define SerialPortServiceClass_UUID     "00001101-0000-1000-8000-00805F9B34FB"  // Android standard

#define BUFFER_SIZE                     255


// headers
//#include <ros.h>
//#include <std_msgs/String.h>
#include <Timer.h>      // periodic callback function lib.
//#include <ArduinoBLE.h> // Bluetooth Low Energy lib.
#include <SoftwareSerial.h>


Timer tTimer;

//SoftwareSerial BTSerial(RX_PIN,TX_PIN);
HardwareSerial* BTSerial = &Serial1;
/*
BLEDevice peripheral; // Arduino & Central: NUC
BLEService ROS_PUB(ROS_PUB_UUID);
BLEService ROS_SUB(ROS_SUB_UUID);
BLECharacteristic ROS_RecMsg(MSG_REC_UUID, BLERead | BLENotify, sizeof(char), sizeof(char)*BUFFER_SIZE);
BLECharacteristic ROS_SendMsg(MSG_SEND_UUID, BLEWrite | BLENotify, sizeof(char), sizeof(char)*BUFFER_SIZE);
*/

/*
ros::NodeHandle   nh;
std_msgs::String  ROS_SendStr;  // topic send buffer
String            ROS_RecStr;   // topic receive buffer

ros::Publisher    ContactPub("contact", &ROS_SendStr); // send contact information to NUC by BT
*/
#endif
