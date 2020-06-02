/*
         Author         Ahn semin
         Created        2020.05.07
         Last modified  2020.06.02 17:48
         Description    Relay switch control for Autonomous charging(Station side Arduino Nano)

*/

#define DEBUG                           0
#define LED_PIN   13

#include "ZETA_RelayControl_StationSide.h"

char recByte = '\0';
bool nc = true; // no contact detection flag

void setup() {
  // put your setup code here, to run once:
#if DEBUG
  Serial.begin(SERIAL_SPEED);
  delay(ONE_SEC);
#endif
  pinMode(PWM_PINP, OUTPUT);
  pinMode(PWM_PINN, OUTPUT);  // PWM generation pins
  pinMode(RELAYP_PIN, OUTPUT);
  pinMode(RELAYN_PIN, OUTPUT); // Relay switch control pin
  digitalWrite(RELAYP_PIN,HIGH);
  digitalWrite(RELAYN_PIN,HIGH);
  BTSerial->begin(SERIAL_SPEED);
  delay(ONE_SEC);

  tTimer.every(ONE_SEC, PWM_OUT); // generates periodic signal at every x msec
  tTimer.every(1.5 * ONE_SEC, LED_blink);
  delay(ONE_SEC);
#if DEBUG
  Serial.println("station setup");
#endif
}

void loop() {
  // put your main code here, to run repeatedly:
  getBT();
  tTimer.update();
}

void PWM_OUT() {
  if (nc) {
#if DEBUG
    //Serial.println("PWM");
#endif
    analogWrite(PWM_PINP, PWM_WIDTH * PWM_DUTY);
    analogWrite(PWM_PINN, PWM_WIDTH * PWM_DUTY);
  }
}


void chargingOn() {
  digitalWrite(RELAYP_PIN, CHARGER_ON);
  digitalWrite(RELAYN_PIN, CHARGER_ON);
  Serial.println("charging on");
}

void chargingOff() {
  digitalWrite(RELAYP_PIN, ARDUINO_ON);
  digitalWrite(RELAYN_PIN, ARDUINO_ON);
  delay(5 * ONE_SEC);
  BTSerial->write(RESET);  // send reset signal.
}

void getBT() {
  if (BTSerial->available()) {
    recByte = BTSerial->read();
    //Serial.println(recByte);
    if (recByte == CONTACT) { // if the contact are closed
      nc = false;
      chargingOn();
      BTSerial->write(OK);
      delay(0.3 * ONE_SEC);
    } else if (recByte == FULLY_CHARGED) { // if the battery is fully charged
      // write actions after fully charging(assuming the robot is away from the station already)
      BTSerial->write(OK);
      delay(5 * ONE_SEC);
      chargingOff();
    } else if (recByte == PROXIMITY) { // if the robot is close
      BTSerial->write(OK);
      nc = true;
      delay(0.3 * ONE_SEC);
    }
    #if DEBUG
    Serial.println(recByte);
    #endif
    recByte = '\0';
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

// EOF
