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
#define SWITCH_PINL                     8
#define SWITCH_PINR                     9
#define RX_PIN                          2
#define TX_PIN                          3
#define RELAYL_PIN                      5
#define RELAYR_PIN                      6

#define SERIAL_SPEED                    115200

#define ONE_SEC                         1000

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
#define CHARGER_ON                      LOW
#define CHARGER_OFF                     HIGH

// UUIDs
#define ROS_PUB_UUID                    "8bab6c96-0238-40ad-85b4-2358049a553c"  // custom V4 UUID
#define ROS_SUB_UUID                    "abb844b1-132f-46db-b71e-4fb8d4ef63a8"
#define MSG_REC_UUID                    "810df03c-90b9-47c2-875d-88cd9ca8d9db"
#define MSG_SEND_UUID                   "61d1c3a8-ba3f-4a86-9dc5-2f4e49090a9c"
#define SerialPortServiceClass_UUID     "00001101-0000-1000-8000-00805F9B34FB"  // Android standard

#define BUFFER_SIZE                     255


// headers
#include <Timer.h>      // periodic callback function lib.
#include <SoftwareSerial.h>


Timer tTimer;

SoftwareSerial BTSerial(RX_PIN,TX_PIN);

#endif
