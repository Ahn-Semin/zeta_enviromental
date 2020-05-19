 /* 
  *       Author         Ahn semin
  *       Created        2020.05.07
  *       Last modified  2020.05.15 18:20
  *       Description      Relay switch control for Autonomous charging(Station side Arduino Nano)
  *                      Peripheral device must be paired.
  */
  
#define DEBUG                           1

#include "ZETA_RelayControl_StationSide.h"

void setup() {
  // put your setup code here, to run once:
  char temp[] = "";
  #if DEBUG
  Serial.begin(SERIAL_SPEED);
  delay(ONE_SEC);
  #endif
  pinMode(PWM_PINP,OUTPUT);
  pinMode(PWM_PINN,OUTPUT); // PWM generation pins
  BLE.begin();
  delay(ONE_SEC);
  BLE.setAdvertisedServiceUuid(SerialPortServiceClass_UUID);  // this uuid shows on central devices
  BLE.setLocalName("Station");
  
  BLE.setAdvertisedService(ROS_PUB);
  ROS_PUB.addCharacteristic(ROS_SendMsg);
  
  BLE.setAdvertisedService(ROS_SUB);
  ROS_SUB.addCharacteristic(ROS_RecMsg);
  
  BLE.addService(ROS_PUB);
  BLE.addService(ROS_SUB);

  //BLE.setAdvertisingInterval(100); // 100* 0.625 ms, default 100ms
  BLE.advertise();
  peripheral = BLE.available();
  
  tTimer.every(ONE_SEC,PWM_OUTP);
  tTimer.every(ONE_SEC,PWM_OUTN); // generates periodic signal at every x msec
  delay(ONE_SEC);
  nh.getHardware() -> setBaud(SERIAL_SPEED);
  nh.initNode();
  nh.advertise(ContactPub);
  delay(0.5*ONE_SEC);
  ROS_SendStr.data = temp;
  ContactPub.publish(&ROS_SendStr);
  nh.spinOnce();
  delay(0.1*ONE_SEC);
}

void loop() {
  // put your main code here, to run repeatedly:
  BLEDevice central = BLE.central();  // request BLE connection
  if(central) { // if Arduino is connected to NUC
    char temp[] = "send msg";
    int i = 0;
    ContactPub.publish(&ROS_SendStr);
    while(temp[i] != '\0') 
    delay(100);
    nh.spinOnce();
  }
  tTimer.update();
}

void PWM_OUTP() {
  analogWrite(PWM_PINP,PWM_WIDTH*PWM_DUTY);
}

void PWM_OUTN() {
  analogWrite(PWM_PINN,PWM_WIDTH*PWM_DUTY);
}
