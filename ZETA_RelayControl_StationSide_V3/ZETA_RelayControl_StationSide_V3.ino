/*
         Author         Ahn semin
         Created        2020.05.07
         Last modified  2020.06.02 17:48
         Description    Relay switch control for Autonomous charging(Station side Arduino Nano)

*/

#define DEBUG     0
#define LED_PIN   13

#include "ZETA_RelayControl_StationSide.h"

char recByte      = '\0';
bool startFlag    = false;
bool chargingFlag = false;


void setup() {
  // put your setup code here, to run once:
#if DEBUG
  Serial.begin(SERIAL_SPEED);
  delay(ONE_SEC);
#endif
  pinMode(SWITCH_PINL, INPUT);
  pinMode(SWITCH_PINR, INPUT);
  pinMode(RELAYL_PIN, OUTPUT);
  pinMode(RELAYR_PIN, OUTPUT); // Relay switch control pin
  digitalWrite(RELAYL_PIN, HIGH);
  digitalWrite(RELAYR_PIN, HIGH);
  BTSerial.begin(SERIAL_SPEED);
  delay(ONE_SEC);
  tTimer.every(1.5 * ONE_SEC, LED_blink);
  delay(0.5*ONE_SEC);
#if DEBUG
  Serial.println("station setup");
#endif
}

void loop() {
  // put your main code here, to run repeatedly:
  getBT();
  checkContact();
  tTimer.update();
}

void chargingOn() {
  digitalWrite(RELAYL_PIN, CHARGER_ON);
  digitalWrite(RELAYR_PIN, CHARGER_ON);
  #if DEBUG
  Serial.println("charging on");
  #endif
}

void chargingOff() {
  digitalWrite(RELAYL_PIN, CHARGER_OFF);
  digitalWrite(RELAYR_PIN, CHARGER_OFF);
  chargingFlag = false;
  //BTSerial.write(RESET);  // send reset signal.
}

void getBT() {
  if (BTSerial.available()) {
    recByte = BTSerial.read();
    if (recByte == FULLY_CHARGED || recByte == RESET) { // if the battery is fully charge
      BTSerial.write(OK);
      chargingOff();
      delay(ONE_SEC);
    } else if (recByte == PROXIMITY) { // if the robot is close
      BTSerial.write(OK);
      delay(ONE_SEC);
      #if DEBUG
      Serial.println("proxi");
      #endif
    } else if (recByte == CHARGING) {
      chargingFlag = true;
    }
    #if DEBUG
    Serial.println(recByte);
    #endif
    if(recByte != OK) recByte = '\0';
  }
}

void LED_blink() {
  static bool state;
  if(state) {
    digitalWrite(LED_PIN, HIGH);
    state = false;
  } else if(!state) {
    digitalWrite(LED_PIN, LOW);
    state = true;
  }
}

void checkContact() {
  bool CONTACTL = digitalRead(SWITCH_PINL);
  bool CONTACTR = digitalRead(SWITCH_PINR);
  
  if(CONTACTL) BTSerial.write(CONTACTLEFT);
  else if(CONTACTR) BTSerial.write(CONTACTRIGHT);
  if(CONTACTL && CONTACTR && !chargingFlag) {
    #if DEBUG
    Serial.println("contact");
    #endif
    chargingOn();
    recByte = '\0';
    while(recByte != OK){
      BTSerial.write(CONTACT);
      getBT();
      delay(0.3 * ONE_SEC);
      #if DEBUG
      Serial.println("waiting BT...");
      #endif
      tTimer.update();
    }
  }
  
}

// EOF
