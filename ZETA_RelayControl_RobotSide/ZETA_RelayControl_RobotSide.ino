 /* 
  *       Author         Ahn semin
  *       Created        2020.05.07
  *       Last modified  2020.06.02 17:48
  *       Description    Relay switch control for Autonomous charging(Robot side Arduino NANO)
  */

#define DEBUG     0

#include "ZETA_RelayControl_RobotSide.h"

bool ContactFlag_P       = false;
bool ContactFlag_N       = false;

// other variables
uint32_t TimeChargingStart    = 0;
uint32_t TimeChargingCurrent  = 0;
bool ContactFlag              = false;
bool Charging                 = false;
bool ChargingFinishFlag       = false;
bool ChargingFlag             = false;
bool ProximityFlag            = false;
bool FinishFlag               = false;
bool StartFlag                = false;
//
char recByte = '\0';


void setup() {
  // put your setup code here, to run once:
  #if DEBUG
  Serial.begin(SERIAL_SPEED);
  delay(ONE_SEC);
  #endif
  BTSerial->begin(SERIAL_SPEED);
  delay(ONE_SEC);
  pinMode(CNT_PINP,INPUT);
  //pinMode(CNT_PINN,INPUT);
  delay(0.05*ONE_SEC);
  //attachInterrupt(digitalPinToInterrupt(CNT_PINP), ContactCallbackP, RISING);  // reads PWM which is generated from NANO(station)
  //attachInterrupt(digitalPinToInterrupt(CNT_PINN), ContactCallbackN, RISING);
  delay(0.05*ONE_SEC);
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
  //nh.loginfo("ready to publish contact information");
  //nh.loginfo("========================================");
  #if DEBUG
  Serial.println(F(""));
  Serial.println(F("setup"));
  delay(ONE_SEC);
  #endif
  delay(0.05*ONE_SEC);
  tTimer.every(0.5*CALLBACK_PERIOD, Refresh); // To prevent the "lost sync"
  //tTimer.every(CALLBACK_PERIOD, getCurrentTime);
  tTimer.every(CALLBACK_PERIOD, BTCom);
}



void loop() {
  // put your main code here, to run repeatedly:
  pulsecount();
  ChargingContact();
  getBT();
  tTimer.update();
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
  if(StartFlag) ROSPublishing("start");
  else if(ContactFlag) ROSPublishing("contact");
  else if(ChargingFlag) ROSPublishing("charging...");
  else ROSPublishing("searching station...");
}

void ChargingContact() {
  if(ContactFlag_P && !ContactFlag) {
    ContactFlag = true;
  }
  if(ContactFlag) {
    #if DEBUG
    Serial.println(F("contact"));
    #endif
    ROSPublishing("contact");
    #if DEBUG
    nh.loginfo("contact!!!!!!!!!!!");
    #endif
    while(recByte != OK){
      BTSerial->write(CONTACT);
      getBT();
      delay(0.3 * ONE_SEC);
      #if DEBUG
      nh.loginfo("inf loop before get OK from station(contact)");
      #endif
    } 
    delay(5*ONE_SEC);
    TimeChargingStart = TimeChargingCurrent;
    batteryOn();
    ContactFlag_P = false;
    Charging      = true;
    StartFlag     = false;
    ContactFlag   = false;
    recByte = '\0';
  }
}

void getCurrentTime() {
  if(Charging) TimeChargingCurrent = millis();
}

void ChargingFinish() {
  if(FinishFlag) batteryOff();
}

void getBT() {
  if(BTSerial->available()) {
    recByte = BTSerial->read();
  }
}

void batteryOn() {
  digitalWrite(RELAYN_PIN, BATTERY_ON);
  digitalWrite(RELAYP_PIN, BATTERY_ON);
}

void batteryOff() {
  digitalWrite(RELAYN_PIN, ARDUINO_ON);
  digitalWrite(RELAYP_PIN, ARDUINO_ON);
}

void pulsecount() {
  int width_p = 0;
  static int PWM_CNT_P=0;
  char temp[10];
  
  if(ProximityFlag) if(PWM_CNT_P <= PWM_threshold) width_p = pulseIn(CNT_PINP,HIGH,PWM_maxWidth);

  if(width_p > PWM_minWidth && width_p < PWM_maxWidth) PWM_CNT_P++;
  sprintf(temp,"%d",PWM_CNT_P);
  #if DEBUG
  nh.loginfo(temp);
  #endif
  if(PWM_CNT_P > PWM_threshold) {
    ContactFlag_P = true;
    PWM_CNT_P = 0;
  }


}

void BTCom() {
  if(ChargingSubMsg.indexOf("start") != -1 && !ProximityFlag) {
    while(recByte != OK) {
      BTSerial->write(PROXIMITY);  // if string exists
      delay(0.3*ONE_SEC);
      getBT();
      #if DEBUG
      nh.loginfo("inf loop before start subscribe");
      #endif
    }
    StartFlag     = true;
    ProximityFlag = true;
    FinishFlag    = false;
  }
  //else if(ChargingSubMsg.indexOf("charging") != -1) BTSerial->write('m');
  else if(ChargingSubMsg.indexOf("finish") != -1 && !FinishFlag){
    FinishFlag    = true;
    StartFlag     = false;
    ProximityFlag = false;
    ChargingFinish();
    while(recByte != OK) {
      BTSerial->write(FULLY_CHARGED);
      delay(0.3*ONE_SEC);
      getBT();
      #if DEBUG
      nh.loginfo("inf loop after finish subscribe");
      #endif
    }
    
  }
  else if(ChargingSubMsg.indexOf("charging") != -1) {
    BTSerial->write(CHARGING); // ing
    ChargingFlag = true;
  }
  ChargingSubMsg = "";
  recByte = '\0';
}

// EOF
