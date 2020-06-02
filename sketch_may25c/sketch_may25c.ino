 /* 
  *       Author         Ahn semin
  *       Created        2020.04.13
  *       Last modified  2020.05.04
  *       Description    H2O2 sensor(GMF-W-C-H2O2-500-G) w/ Arduino NANO 33 IoT
  */
#define DEBUG            0

#define SerialTxControl  12   //RS485 trans & rec control pin
#define RS485Transmit    HIGH
#define RS485Receive     LOW
#define RS485_baudrate   9600 

#define RecDataLen_Short        6
#define RecDataNoCRCLen_Short   4
#define RecDataLen_Long         7
#define RecDataNoCRCLen_Long    5
#define MAXIMUM_PPM             500
#define DataLen                 6 // 500.0 + NULL ch
#define BUFFER_SIZE             255
#define SMALL_BUFFER            30

#include <CRC16_modbus.h>
#include <Arduino.h>
#include <SoftwareSerial.h>


SoftwareSerial RS485_Serial(2,3);


String data_str;
String inner_str;
int str_len = 0;


// Time variables
uint64_t Time_getData_current   = 0; // time var for getData
uint64_t Time_getData_previous  = 0;

uint64_t Time_sendData_current   = 0; // time var for sending
uint64_t Time_sendData_previous  = 0;

uint16_t Time_getDataPeriod   = 500;
uint16_t Time_sendDataPeriod  = 15000;


// 
const int data_len          = DataLen - 1;
const char CMD_GET_DATA[]   = {0x00, 0x20, 0x00, 0x68, 0x00};
const char CMD_ZERO[]       = {0x01, 0x38, 0x00, 0x33, 0xC0};
const char CMD_INIT_CALIB[] = {0x01, 0x31, 0x00, 0x35, 0x90};
char CMD_SPAN[]             = {0x01, 0x39, 0x02, 0x00, 0x00, 0x00, 0x00};
bool RequestFlag            = false;
bool Checksum               = false;
bool DataShowFlag           = false;
bool ContinuousFlag         = false;
union {
  uint16_t  CRC_code;
  uint8_t   CRC_byte[2];
}CRC;

uint16_t H2O2_PPM = 0;
float H2O2_PPMf  = 0.0;
// variables for calibration
char rec_byte = '\0';

void setup() {
  delay(1000);
  Serial.begin(9600);
  delay(1000);
  while(Serial.available()) Serial.read();
  RS485_Serial.begin(9600SERIAL_8E1);
  delay(1000);
  if(RS485_Serial){
    while(RS485_Serial.available()) RS485_Serial.read();  
  }
  pinMode(SerialTxControl,OUTPUT);
  delay(1000);
  digitalWrite(SerialTxControl,RS485Receive);
  // check for the WiFi module:

  

}





void loop() {
  CMD_pulling();
  Time_getData_current = millis();
  if(Time_getData_current - Time_getData_previous > Time_getDataPeriod ){
    getH2O2();
    Time_getData_previous = Time_getData_current;
    Time_sendData_current = millis();
  }
}








void getH2O2() {
  char data[DataLen]                              = {'0',};
  char Rec_buffer[RecDataLen_Long]                = {0x00,};
  uint8_t buffer_sub_Short[RecDataNoCRCLen_Short] = {0x00,};
  uint8_t buffer_sub_Long[RecDataNoCRCLen_Long]   = {0x00,};
  int count = 0;
  if(*RS485_Serial) {
    digitalWrite(SerialTxControl, RS485Transmit);
    for(int i = 0; i < 5; i++) {
      RS485_Serial.write(CMD_GET_DATA[i]);
    }
    delay(6);
    digitalWrite(SerialTxControl, RS485Receive);
    delay(10);
    int num = RS485_Serial.available();
    String str = String(num);
    char chararr[10] = {'\0',};
    str.toCharArray(chararr,9);
    //Socket_CMD(chararr);
    delay(10);
    if(RS485_Serial.available() == RecDataLen_Short) {
      RS485_Serial.readBytes(Rec_buffer, RecDataLen_Short);
      for(int i = 0; i < RecDataNoCRCLen_Short; i++) buffer_sub_Short[i] = Rec_buffer[i];
      CRC.CRC_code = CRC16_MODBUS(buffer_sub_Short, RecDataNoCRCLen_Short);
      if(Rec_buffer[4] == CRC.CRC_byte[0] & Rec_buffer[5] == CRC.CRC_byte[1]) Checksum = true;
      if(Checksum) {
        H2O2_PPM = Rec_buffer[3];
        H2O2_PPMf = double(H2O2_PPM)/10;
        Checksum = false;
      }
    } else if(RS485_Serial.available() == RecDataLen_Long) {
        RS485_Serial.readBytes(Rec_buffer, RecDataLen_Long);
        for(int i = 0; i < RecDataNoCRCLen_Long; i++) buffer_sub_Long[i] = Rec_buffer[i];
        CRC.CRC_code = CRC16_MODBUS(buffer_sub_Long, RecDataNoCRCLen_Long);
        if(Rec_buffer[5] == CRC.CRC_byte[0] & Rec_buffer[6] == CRC.CRC_byte[1]) Checksum = true;
        if(Checksum) {
        H2O2_PPM = Rec_buffer[3] + Rec_buffer[4] << 8UL;
        H2O2_PPMf = double(H2O2_PPM)/10;
        Checksum = false;
      }
    } else {
      while(RS485_Serial.available()) RS485_Serial.read();
    }
    
    
    if(DataShowFlag) {
      if(!ContinuousFlag) DataShowFlag = false;
    }
    SocketSendFlag = true;  
  }
  
}

void CMD_pulling() {
  rec_byte = rec_socket;
  rec_socket = '\0';
  if(rec_byte == 'm') DataShowFlag = true;
  else if(rec_byte == 'z') SetZero();
  else if(rec_byte == 's') SetSpan();
  else if(rec_byte == 'i') InitCalib();
  else if(rec_byte == 'c') {
    ContinuousFlag = true;
    DataShowFlag   = true;
  }
  else if(rec_byte == 'q') ContinuousFlag = false;
  rec_byte = '\0';
}

void SetSpan() {
  float span_value = 0.0;
  String span_valueS;
  int span_valueI = 0;
  uint8_t numH = 0;
  uint8_t numL = 0;
  char span_valueC[5] = {'\0',};
  uint8_t CRC_buffer[5] = {0x01, 0x39, 0x02, '\0',};
  if(SocketSpan) {
    span_valueS = Socket();
    SocketSpan = false;
    span_value = span_valueS.toFloat();
  } else {
  }
  if(span_value > 500.0 || span_value < 0.09) {
    return;
  }

  span_valueI = int(span_value*10);   // 100.1 will be convert to 1001 & under 2 decimal point will be ignored
  numH = span_valueI / 100;  // for 1001, 10
  numL = span_valueI % 100;  // for 1001, 01
  CRC_buffer[3] = numL;
  CRC_buffer[4] = numH;
  CRC.CRC_code = CRC16_MODBUS(CRC_buffer,sizeof(CRC_buffer));
  for(int i = 0; i< sizeof(CRC_buffer); i++) {
    CMD_SPAN[i] = CRC_buffer[i];
  }
  CMD_SPAN[5] = CRC.CRC_byte[0];
  CMD_SPAN[6] = CRC.CRC_byte[1];
  digitalWrite(SerialTxControl, RS485Transmit);
  for(int i = 0; i < sizeof(CMD_SPAN); i++) RS485_Serial.write(CMD_SPAN[i]);
  delay(6);
  //Socket_CMD("set span done.");
}

void InitCalib() {
  digitalWrite(SerialTxControl, RS485Transmit);
  for(int i = 0; i < sizeof(CMD_INIT_CALIB); i++) RS485_Serial.write(CMD_INIT_CALIB[i]);
  delay(6);
  //Socket_CMD("init calib done.");
}

void SetZero() {
  digitalWrite(SerialTxControl,RS485Transmit);
  for(int i = 0; i < sizeof(CMD_ZERO); i++) RS485_Serial.write(CMD_ZERO[i]);
  delay(6);
  rec_byte = DataShowFlag = true;
  delay(100);
  //Socket_CMD("set zero done.");
  getH2O2();
}

// EOF
