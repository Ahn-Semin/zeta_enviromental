#define HCHO_rxPIN     2
#define HCHO_txPIN     3
#define HCHO_DateLen   9
#define HCHO_MolWeight 30.031f
#define stdMolNum      22.24

//#include <ze08_ch2o.h>
#include <SoftwareSerial.h>

// Instantiate a serial port, whatever Stream you have
//SoftwareSerial ch2oSerial(4, SW_SERIAL_UNUSED_PIN); // RX, TX

// Instantiate a sensor connected to the previous port
//Ze08CH2O ch2o{&ch2oSerial};

byte HCHO_CMD_getData[HCHO_DateLen] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
byte HCHO_CMD_QnA[HCHO_DateLen]     = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
SoftwareSerial HCHO_Serial(HCHO_rxPIN, HCHO_txPIN);

uint32_t HCHO_ppb  = 0;
float HCHO_ugm3    = 0;
uint8_t HCHO_recData[HCHO_DateLen] = {0x00,};

void setup() {
  Serial.begin(115200);  // Console
  delay(1000);
  HCHO_init();
  
}

void loop() {
  if(Serial.available()){
    for(int i = 0; i < HCHO_DateLen; i++) {
      //HCHO_Serial.write(HCHO_CMD_getData[i]);
    }
  }
  delay(100);
  if(HCHO_Serial.available() == HCHO_DateLen) {
    HCHO_Serial.readBytes(HCHO_recData,HCHO_DateLen);
    Serial.println("");
    if(FucCheckSum(HCHO_recData, HCHO_DateLen) == HCHO_recData[8]){
      HCHO_ppb = HCHO_recData[4] * 256 + HCHO_recData[5];
      HCHO_ugm3 = HCHO_ppb * HCHO_MolWeight / stdMolNum;
      Serial.print("HCHO: ");Serial.print(HCHO_ugm3);Serial.println("[ug/m3]");  
    } else {
      for(int i = 0; i < HCHO_DateLen; i++) {
        Serial.print("0x");Serial.print(HCHO_recData[i]);Serial.print(" ");
      }
      Serial.println("");
    }
  } else {
    while(HCHO_Serial.available()) {
      Serial.print("0x");Serial.print(HCHO_Serial.read());Serial.print(" ");
    }
    Serial.println("");
  }
  HCHO_ugm3 = 0;
  delay(900);
}

uint8_t FucCheckSum(uint8_t* hcho, uint8_t hcho_len) {
  uint8_t j, tempq = 0;
  hcho+=1;
  for(j = 0; j < hcho_len - 2; j++) {
    tempq+= *hcho;
    hcho++;
  }
  tempq=(~tempq)+1;
  return tempq;
}

void HCHO_init() {
  HCHO_Serial.begin(9600);
  delay(1000);
  if(Serial.available()){
    for(int i = 0; i < HCHO_DateLen; i++) {
      //HCHO_Serial.write(HCHO_CMD_QnA[i]);
    }
  }
  delay(10);
}
