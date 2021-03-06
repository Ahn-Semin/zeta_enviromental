 /* 
  *       Author         Ahn semin
  *       Created        2020.04.13
  *       Last modified  2020.04.21
  *       Description    H2O2 sensor(GMF-W-C-H2O2-500-G) w/ Arduino NANO 33 IoT
  */
#define DEBUG            1

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

#include <SPI.h>
#include <WiFiNINA.h>
#include <CRC16_modbus.h>

HardwareSerial* RS485_Serial = &Serial1;

// WIFI infomation
char ssid[] = "zeta_dev";        // SSID of WIFI
char pass[] = "12345678";     // Password of the WIFE
int status = WL_IDLE_STATUS;  // The Wifi radio's status

// DB infomation
//const char* server = "15.164.221.85"; // Zeta Server
//const int port = 5543;
const char* server = "192.168.0.29"; // Zeta Server
const int port = 5000;
char data_str[]="{\"command\":\"SENSOR_DATA\", \"sid\":\"XefiGyJeTYMpWNNSViBJW6pSI3yp7IKs\", \"data\":12345}"; // maximum 500.0
const int str_len = sizeof(data_str);

// Time variables
uint64_t Time_getData_current   = 0; // time var for getData
uint64_t Time_getData_previous  = 0;

uint64_t Time_sendData_current   = 0; // time var for sending
uint64_t Time_sendData_previous  = 0;

uint16_t Time_getDataPeriod   = 500;
uint16_t Time_sendDataPeriod  = 2500;

WiFiClient client;

// 
const int data_len          = DataLen - 1;
const char CMD_GET_DATA[]   = {0x00, 0x20, 0x00, 0x68, 0x00};
const char CMD_ZERO[]       = {0x01, 0x38, 0x00, 0x33, 0xC0};
const char CMD_INIT_CALIB[] = {0x01, 0x31, 0x00, 0x35, 0x90};
char CMD_SPAN[]             = {0x01, 0x39, 0x02, 0x00, 0x00, 0x00, 0x00};
bool RequestFlag            = false;
bool Checksum               = false;
bool DataShowFlag           = false;

union {
  uint16_t  CRC_code;
  uint8_t   CRC_byte[2];
}CRC;

uint16_t H2O2_PPM = 0;
float H2O2_PPMf  = 0.0;
// variables for calibration
char rec_byte = '\0';

void setup() {
  Serial.begin(9600,SERIAL_8E1);
  delay(100);
  RS485_Serial->begin(9600,SERIAL_8E1);
  delay(100);
  #if DEBUG
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  #endif
  pinMode(SerialTxControl,OUTPUT);
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    
    #if DEBUG
    Serial.println("Communication with WiFi module failed!");
    #endif
    
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    getWIFI();
    
    #if DEBUG
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    #endif
    
    status = WiFi.begin(ssid, pass);
    
    #if DEBUG
    Serial.println(status);
    #endif
    
    // wait 3 seconds for connection:
    delay(5000);
  }

  // you're connected now, so print out the data:
  
  #if DEBUG
  Serial.println("You're connected to the network");
  #endif
  
}









void loop() {
  Time_getData_current = millis();
  if(Time_getData_current - Time_getData_previous > Time_getDataPeriod ){
    getH2O2();
    Time_getData_previous = Time_getData_current;
    Time_sendData_current = millis();
  }
  UART_pulling();
  
  
  /*
  if(Time_sendData_current - Time_sendData_previous > Time_sendDataPeriod ) {
    if (!client.connect(server, port)) {
      #if DEBUG
      Serial.println("connection failed");
      #endif
    
      return;
    } else {
      #if DEBUG
      Serial.println(data_str);
      #endif
      client.write(data_str);
      String recevbline = client.readStringUntil('\r');
     
      #if DEBUG
      Serial.println(recevbline);
      #endif
    }
    client.flush();
    client.stop();
    Time_sendData_previous = Time_sendData_current;
  }*/
}








void getWIFI() {
  /*
  Serial.println("enter the WIFI SSID(ex: iptime): ");
  ssid = readKeyboard();
  Serial.println("enter the WIFI password: ");
  pass = readKeyboard();
  */
}






void getH2O2() {
  char data[DataLen]           = {'0',};
  char Rec_buffer[RecDataLen_Long] = {0x00,};
  uint8_t buffer_sub_Short[RecDataNoCRCLen_Short] = {0x00,};
  uint8_t buffer_sub_Long[RecDataNoCRCLen_Long]   = {0x00,};
  String temp;
  temp.reserve(DataLen);
    digitalWrite(SerialTxControl, RS485Transmit);
    for(int i = 0; i < 5; i++) {
      RS485_Serial->write(CMD_GET_DATA[i]);
    }
    delay(6);
    digitalWrite(SerialTxControl, RS485Receive);
    if(RS485_Serial->available() == RecDataLen_Short) {
      RS485_Serial->readBytes(Rec_buffer, RecDataLen_Short);
      for(int i = 0; i < RecDataNoCRCLen_Short; i++) buffer_sub_Short[i] = Rec_buffer[i];
      CRC.CRC_code = CRC16_MODBUS(buffer_sub_Short, RecDataNoCRCLen_Short);
      if(Rec_buffer[4] == CRC.CRC_byte[0] & Rec_buffer[5] == CRC.CRC_byte[1]) Checksum = true;
      if(Checksum) {
        H2O2_PPM = Rec_buffer[3];
        H2O2_PPMf = double(H2O2_PPM)/10;
        Checksum = false;
      }
    } else if(RS485_Serial->available() == RecDataLen_Long) {
      RS485_Serial->readBytes(Rec_buffer, RecDataLen_Long);
      for(int i = 0; i < RecDataNoCRCLen_Long; i++) buffer_sub_Long[i] = Rec_buffer[i];
      CRC.CRC_code = CRC16_MODBUS(buffer_sub_Long, RecDataNoCRCLen_Long);
      if(Rec_buffer[5] == CRC.CRC_byte[0] & Rec_buffer[6] == CRC.CRC_byte[1]) Checksum = true;
      if(Checksum) {
        H2O2_PPM = Rec_buffer[3] + Rec_buffer[4] << 8UL;
        H2O2_PPMf = double(H2O2_PPM)/10;
        Checksum = false;
      }
    } else {
      while(RS485_Serial->available()) RS485_Serial->read();
    }
    
    temp = String(H2O2_PPMf);
    temp.toCharArray(data, DataLen);
    for(int i=data_len; i>0; i--) {
      if(data[data_len-i] == '\0') data[data_len-i] = '0';
      data_str[str_len-i-2] = data[data_len-i];  // 2 is NULL and '}'
    }
    if(DataShowFlag) {
      Serial.print(H2O2_PPMf); Serial.println(" PPM");
      DataShowFlag = false;
    }
}

void UART_pulling() {
  if(Serial.available()) rec_byte = Serial.read(); 
  if(rec_byte == 'm') DataShowFlag = true;
  else if(rec_byte == 'z') SetZero();
  else if(rec_byte == 's') SetSpan();
  else if(rec_byte == 'i') InitCalib();
  rec_byte = '\0';
}

void SetSpan() {
  char rec_buffer[BUFFER_SIZE] = {'\0',};
  String span_valueS;
  int span_valueI = 0;
  int digit = 0;
  Serial.print("enter the span value in PPM(ex: 100.1):");
  do{
    if(Serial.available()) {
      rec_buffer[digit] = Serial.read();
    }
  }while(rec_buffer[digit++] != '\n');
  if(digit>DataLen) return;
  span_valueS = String(rec_buffer);
  
  digitalWrite(SerialTxControl, RS485Transmit);
  for(int i = 0; i < 5; i++) {
      //RS485_Serial->write(CMD_GET_DATA[i]);
  }
    delay(6);
}

void InitCalib() {
  
}

void SetZero() {
  Serial.println("set the zero on the current measurement value");
  for(int i = 0; i < sizeof(CMD_ZERO); i++) {
    RS485_Serial->write(CMD_ZERO[i]);
  }
  delay(6);
  rec_byte = DataShowFlag = true;
  getH2O2();
}

// EOF
