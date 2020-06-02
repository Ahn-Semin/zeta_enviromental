// robot side 
#define speeD   115200
#define SERIAL_SPEED speeD
#define BT_SPEED     apeeD

#define ONE_SEC 1000

HardwareSerial* BTSerial = &Serial1;
char recByte = '\0';
void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_SPEED);
  delay(0.5*ONE_SEC);
  BTSerial->begin(SERIAL_SPEED);
  delay(0.5*ONE_SEC);
  Serial.println("do");
}

void loop() {
  if(BTSerial->available()) {
    Serial.print(BTSerial->read());
  }
  // put your main code here, to run repeatedly:

}
