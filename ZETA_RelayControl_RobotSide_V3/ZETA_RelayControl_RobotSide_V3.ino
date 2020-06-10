 /* 
  *       Author         Ahn semin
  *       Created        2020.05.07
  *       Last modified  2020.06.09 17:06
  *       Description    Relay switch control for Autonomous charging(Robot side Arduino NANO)
  */

#define DEBUG     0

#include "ZETA_RelayControl_RobotSide.h"


// other variables
flag_t FLAG[MAX_FLAG]         = {false,};

//
char recByte = '\0';
int timerIndex                = 0;

void setup() {
  // put your setup code here, to run once:
  #if DEBUG
  Serial.begin(SERIAL_SPEED);
  delay(ONE_SEC);
  #endif
  BTSerial->begin(SERIAL_SPEED);
  delay(ONE_SEC);
  pinMode(RELAYP_PIN,OUTPUT);
  pinMode(RELAYN_PIN,OUTPUT);
  digitalWrite(RELAYP_PIN,ARDUINO_ON);
  digitalWrite(RELAYN_PIN,ARDUINO_ON);
  ChargingSubMsg.reserve(BUFFER_SIZE);
  
  nh.getHardware()->setBaud(SERIAL_SPEED);
  nh.initNode();
  nh.advertise(ChargingPub);
  nh.subscribe(ChargingSub);
  delay(0.05*ONE_SEC);

  tTimer[0].every(0.05*CALLBACK_PERIOD, Refresh);
  //tTimer[0].every(0.5*CALLBACK_PERIOD, checkConnection);
  timerIndex++;
  tTimer[1].every(CALLBACK_PERIOD, BTCom);
  timerIndex++;
  nh.loginfo("charge_state_INO setup finish");
}


void loop() {
  // put your main code here, to run repeatedly:
  getBT();
  BTCom();
  FLAG[6] = false;  // stop publishing BT return
  TimerUpdate();
}


// ROS publishing func.
void ROSPublishing(String sendmsg) {
  char buffer[BUFFER_SIZE] = {'\0',};
  sendmsg.toCharArray(buffer, sendmsg.length()+1);
  ChargingPubMsg.data = buffer;
  ChargingPub.publish(&ChargingPubMsg);
  nh.spinOnce();
}

// ROS subscribe callback func.
void ChargingSubCallback(const std_msgs::String& msg) {
  ChargingSubMsg = msg.data;
}

//
void Refresh() {
  if(FLAG[5]) ROSPublishing("start");
  else if(FLAG[0]) ROSPublishing("contact");
  else if(FLAG[2]) ROSPublishing("charging...");
  else if(FLAG[6]) ROSPublishing("waiting BT return...");
  else ROSPublishing("waiting...");
}

void ChargingFinish() {
  if(FLAG[4]) batteryOff();
}

void getBT() {
  if(BTSerial->available()) recByte = BTSerial->read();
  if(recByte == CONTACTLEFT) ROSPublishing("left");
  else if(recByte == CONTACTRIGHT) ROSPublishing("right");
  else if(recByte == CONTACT) {
    ROSPublishing("contact");
    BTSerial->write(OK);
    FLAG[0] = true;
    delay(3 * ONE_SEC);
    batteryOn();
    FLAG[5] = false;
  }
  if(recByte != OK) recByte = '\0';
}

void batteryOn() {
  digitalWrite(RELAYN_PIN, BATTERY_ON);
  digitalWrite(RELAYP_PIN, BATTERY_ON);
}

void batteryOff() {
  digitalWrite(RELAYN_PIN, ARDUINO_ON);
  digitalWrite(RELAYP_PIN, ARDUINO_ON);
}

void BTCom() {
  static int waiting_cnt;
  if(ChargingSubMsg.indexOf("start") != -1 && !FLAG[3]) {
    if(!FLAG[7]) FLAG[7] = true;
    while(recByte != OK) {
      BTSerial->write(PROXIMITY);  // if string exists
      delay(0.3*ONE_SEC);
      getBT();
      FLAG[6] = true;
      #if DEBUG
      nh.loginfo("inf loop before start subscribe");
      #endif
      tTimer[0].update();
      if(waiting_cnt++ > 10) {
        waiting_cnt = 0;
        ChargingSubMsg = "";
        return;
      }
    }
    FLAG[5] = true;
    FLAG[3] = true;
    FLAG[4] = false;
  } else if(ChargingSubMsg.indexOf("finish") != -1 && !FLAG[4]){
    FLAG[4] = true;
    FLAG[5] = false;
    FLAG[3] = false;
    ChargingFinish();
    while(recByte != OK) {
      BTSerial->write(FULLY_CHARGED);
      delay(0.3*ONE_SEC);
      getBT();
      FLAG[6] = true;
      tTimer[0].update();
      #if DEBUG
      nh.loginfo("inf loop after finish subscribe");
      #endif
    }  
  } else if(ChargingSubMsg.indexOf("charging") != -1) {
    BTSerial->write(CHARGING); // ing
    FLAG[2] = true;
  }
  ChargingSubMsg = "";
  waiting_cnt = 0;
  recByte = '\0';
}

void checkConnection() {
  static int empty_cnt;
  if(FLAG[7] && ChargingSubMsg.length() == 0) empty_cnt++;
  ChargingSubMsg = "";
  if(empty_cnt > 2) {
    BTSerial->write(RESET);
    empty_cnt = 0;
    nh.loginfo("Aruino reset...");
    resetFunc(); 
  }
}

void TimerUpdate() {
  for(int i = 0; i < timerIndex; i++) {
    tTimer[i].update();
  }
}

// EOF
