 /* 
  *       Author         Ahn semin
  *       Created        2020.03.24
  *       Last modified  2020.03.26
  */
#include <Timer.h> // https://github.com/JChristensen/Timer
#include <mthread.h> // https://github.com/jlamothe/mthread
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <stdio.h>
  // SoftwareSerial.h 
  // Not all pins on the Mega and Mega 2560 support change interrupts,
  // so only the following can be used for RX: 10, 11, 12, 13, 14, 15,
  // 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66),
  // A13 (67), A14 (68), A15 (69).
  // Caution: Only one software serial port can listen at a time.
  // data that arrives for other ports will be discarded.
#include <PMS.h>  // Matter sensor

#include <DHT_U.h>
#include <DHT.h>  // Humidity & temp. sensor

#include <ZE07CO_Sensor.h> // CO sensor


#define Number_of_Thread 9  // 8 for sensors, 1 for loop

// pin config
#define Dust_rxPIN    19  // must use 3.3V level shifter
#define Dust_txPIN    19
#define CO2_rxPIN     15
#define CO2_txPIN     14
#define HCHO_rxPIN    0
#define HCHO_txPIN    0
#define CO_rxPIN      0
#define CO_txPIN      0
#define CO_dacPIN     A0
#define NO2_rxPIN     0 // must use 3.3V level shifter
#define NO2_txPIN     0
#define Rn_rxPIN      17
#define Rn_txPIN      18
#define VOC_rxPIN     0
#define VOC_txPIN     0

#define DHT_PIN       0
#define DHTTYPE       DHT22

// other constants
#define CO2               0
#define NO2               1
#define CO2_buffersize   10
#define CO2_len           4
#define NO2_len           255
uint32_t cnt = 0; // for debug

/* Thread class for receive UART */
class FooThread : public Thread{
    public: FooThread(int id);
    protected: bool loop();
    private: int id;
};

FooThread::FooThread(int id){
  this->id = id;
}

////////////////////////////////////////////////////////////////////////////////
// Dust
//SoftwareSerial Dust_Serial(Dust_rxPIN,Dust_txPIN);  // if available, use hardware serial
HardwareSerial *Dust_Serial = &Serial1;
PMS pms(Serial1);
PMS::DATA Dust_data; // contains 3 uint16_t data(PM_SP_UG_1_0, PM_SP_UG_2_5, PM_SP_UG_10_0)

uint16_t Dust_PM2_5_ugm3 = 0;
uint16_t Dust_PM10_ugm3  = 0;

bool Dust_rec_flag      = false;
bool Dust_request_flag  = false;
////////////////////////////////////////////////////////////////////////////////
//CO2
HardwareSerial* CO2_Serial = &Serial3;
uint16_t CO2_ppm = 0;
char CO2_rec_data[11];

bool CO2_request_flag = false;
bool CO2_rec_flag = false;
////////////////////////////////////////////////////////////////////////////////
//HCHO
/*
 * code is here
 */

////////////////////////////////////////////////////////////////////////////////
// CO

float CO_ppm = 1.0;
bool CO_rec_flag = false;
bool CO_request_flag = false;
////////////////////////////////////////////////////////////////////////////////
// NO2
/*
 * code is here
 */

////////////////////////////////////////////////////////////////////////////////
// Radon
//SoftwareSerial Rn_Serial(Rn_rxPIN,Rn_txPIN);  
HardwareSerial* Rn_Serial = &Serial2;

/* Command */
enum {
    cmd_GAMMA_RESULT_QUERY = 0x44, // D, Read measuring value(10min avg, 1min update)
    cmd_GAMMA_RESULT_QUERY_1MIN = 0x4D, // M, Read measuring value(1min avg, 1min update)
    cmd_GAMMA_PROC_TIME_QUERY = 0x54, // T, Read measuring time
    cmd_GAMMA_MEAS_QUERY = 0x53, // S, Read status
    cmd_GAMMA_FW_VERSION_QUERY = 0x46, // F, Read firmware version
    cmd_GAMMA_VIB_STATUS_QUERY = 0x56, // V, Response vibration status
    cmd_GAMMA_RESET = 0x52, // R, Reset
    cmd_GAMMA_AUTO_SEND = 0x55, // U, Set Auto_send status
    cmd_GAMMA_ALL_QUERY = 0x41, // A, Read all data
};

Timer tTimer;
char Rn_rec_data[20] =  {'\0',}; // Array for received command
String Rn_rec_str = "";
bool Rn_request_flag = false; // enable or disable send command automatically
float Rn_uSv = 0.0;
float Rn_uSv_pre = 0.0;
bool Rn_rec_flag = false;
char cmd_M[] = {0x02, cmd_GAMMA_RESULT_QUERY_1MIN, ':'};
const String cmd_M_str = String(cmd_M);
char temp_data[20] = {'\0',};
String temp_str;
////////////////////////////////////////////////////////////////////////////////
// TVOCs
/*
 * code is here
 */
 
////////////////////////////////////////////////////////////////////////////////
// DHT
DHT dht( DHT_PIN, DHTTYPE );
 
////////////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("arduino wake up");

  main_thread_list->add_thread(new FooThread(1));
  Dust_Serial->begin(9600);
  pms.passiveMode();
  tTimer.every(1000,Dust_callback);
  
  main_thread_list->add_thread(new FooThread(2));
  Rn_Serial->begin(9600);
  /*Gamma_INIT();
  tTimer.every(1000, Rn_callback); // Request to Gamma Sensor
  */ // do after logic level converter setup!!
  
  main_thread_list->add_thread(new FooThread(3));
  pinMode(CO_dacPIN,INPUT);
  tTimer.every(1000, CO_callback);
  
  main_thread_list->add_thread(new FooThread(4));
  CO2_Serial->begin(38400);
  tTimer.every(1000,CO2_callback);

  main_thread_list->add_thread(new FooThread(5)); // thread for publishing date to ROS. change the number of thread
  tTimer.every(1000, Publish_data);
  
  main_thread_list->add_thread(new FooThread(6)); // thread for updating timers(must set as the last thread)
  tTimer.every(3000, Show_data);
  
  Serial.println("setup finish");
  Serial.println("====================================");

}

////////////////////loop is here!!!!!!!!!!!!!!!///////////////////
bool FooThread::loop(){
  switch(id){
    case 1 :
      if(Dust_request_flag) {
        Dust_RecUartData();
      }
      break;
    case 2 :
      if(Rn_request_flag) {
        Rn_RecUartData();
      }
      break;
    case 3 :
      if(CO_request_flag) {
        CO_ppm = ((float)analogRead(CO_dacPIN) * 5 / 1024.0 - 0.4 ) * 500 / 1.6;
        CO_rec_flag = true;
      }
      break;
    case 4 :
      if(CO2_request_flag) {
        CO2_RecUartData();
        CO2_request_flag = false;
      }
      break;
    case 5 :
      break;
    case 6 :
      tTimer.update();
      break;
  }
  /*if(Dust_rec_flag) {
    Serial.print("dust(PM-2.5): ");Serial.println(Dust_PM2_5_ugm3);
    Serial.print("dust(PM-10): ");Serial.println(Dust_PM10_ugm3);
    Dust_rec_flag = false;
  }
  if(Rn_rec_flag) {
    Serial.print("Rn_uSv : "); Serial.println(Rn_uSv);
    Serial.flush();
    Rn_rec_flag = false;
  }
  if(CO_rec_flag) {
    Serial.print("CO_ppm : "); Serial.println(CO_ppm);
    Serial.flush();
    CO_rec_flag = false;
    //Serial.println("===========================");
  }
  if(CO2_rec_flag) {
    Serial.print("CO2_ppm : "); Serial.println(CO2_ppm);
    Serial.flush();
    CO_rec_flag = false;
    Serial.println("===========================");
  }*/
  return true;
}

////////////////////loop end!!!!!!!!!!!!!!!/////////////////////
/*                                    Other functions                                    */
// Dust
void Dust_callback() {
  pms.requestRead();
  Dust_request_flag = true;
}

void Dust_RecUartData() {
  if(pms.readUntil(Dust_data,100)){
    Dust_PM2_5_ugm3 = Dust_data.PM_SP_UG_2_5;
    Dust_PM10_ugm3 = Dust_data.PM_SP_UG_10_0;
    Dust_rec_flag = true;
  }
 
  //Serial.println("dust rec data");
}



//CO2
void CO2_callback() {
  CO2_request_flag = true;
}

void CO2_RecUartData() {
  int rec_size = CO2_Serial->available();
  if(rec_size){
    for(int i = 0; i < rec_size; i++) {
      CO2_rec_data[i] = CO2_Serial->read();
    }
    CO2_ppm = Byte2int(CO2_rec_data);
    CO2_rec_flag = true;
    CO2_request_flag = false;
  }
}

uint16_t Byte2int(char* Byte){
  uint8_t len = CO2_len;
  uint8_t index = 0;
  char temp[CO2_buffersize] = {'\0',};
  for(int i=0; i<len; i++) {
    temp[i] = Byte[index++];
  }
  return atoi(temp);
}

// CO
void CO_callback() {
  CO_request_flag = true;
}

void CO_RecUartData() {
  //Serial.println("CO rec data");
  /*if(CO_Sensor.available(10)) { // wait until 1ms until get data  if wait too long -> overflow
    CO_ppm = CO_Sensor.uartReadPPM();
    CO_rec_flag = true;
  }*/
  //Serial.println("CO rec data");
}

// Radon


/* Gamma Sensor Initialize */
void Gamma_INIT() {
  Read_FW(); // Read FW version
  Reset(); // Reset
  // Add Thread 1 for UART Response
  Serial.println("GDK101 Init.");
}

/* Meawurement Reset */
void Reset(){
  byte send_data[6] = {0x02, cmd_GAMMA_RESET, ':', '1', 0x0D, 0x0A};
  Rn_Serial->write(send_data, 6);
  Serial.println("GDK101 Reset.");
  delay(100);
}

/* Read Firmware */
void Read_FW(){
  byte send_data[6] = {0x02, cmd_GAMMA_FW_VERSION_QUERY, ':', '?', 0x0D, 0x0A};
  Rn_Serial->write(send_data, 6);
  delay(100);
}

/* Read all data (automatically) */
void Rn_callback(){
  //Serial.println("Radon call back");
  byte send_data[6] = {0x02, cmd_GAMMA_RESULT_QUERY_1MIN, 0x3A, 0x3F, 0x0D, 0x0A};
  //byte send_data[6] = {0x02, cmd_GAMMA_ALL_QUERY, 0x3A, 0x3F, 0x0D, 0x0A};
  //while(Rn_Serial.available()){Rn_Serial.read();}
  Rn_Serial->write(send_data, 6);
  Rn_request_flag = true;
  //Serial.println("Radon call back");
}

void Rn_RecUartData(){
  //Serial.println("Radon rec data");
  int rec_size = Rn_Serial->available();
  if (rec_size) {
    Serial.println("Radon rec data enter");
    for(int i = 0; i < rec_size ; i++) {
      Rn_rec_data[i] = Rn_Serial->read();
      temp_data[i] = Rn_rec_data[i];
    }
    Rn_rec_data[rec_size] = '\0';
    String temp = String(Rn_rec_data);
    if(temp.indexOf('.') == -1) //Serial.print(temp);
    if(temp.indexOf(':') != -1) temp.remove(0,temp.indexOf(':')+1);
    //Serial.println("");
    //Serial.print(temp); // why 2 times serial available???? -> remains trash data in uart res.
    //Serial.println("");
    if(temp.startsWith("0") || temp.startsWith("1")){
      Rn_uSv_pre = Rn_uSv;
      Rn_uSv = temp.toFloat();
      if(Rn_uSv - Rn_uSv > 1.0) Rn_uSv = Rn_uSv_pre;  // to ignore noised value
    }
    Rn_rec_flag = true;
  }
  //Serial.println("Radon rec data");
}

void Publish_data() {
  // do something
  // send data at least 3 min after
}

void Show_data() {
  Serial.print("PM-2.5(ug/m3): ");Serial.println(Dust_PM2_5_ugm3);
  Serial.print("PM-10(ug/m3) : ");Serial.println(Dust_PM10_ugm3);
  Serial.flush();
  Serial.print("Radon(uSv)   : "); Serial.println(Rn_uSv);
  Serial.flush();
  Serial.print("CO2(ppm)     : "); Serial.println(CO2_ppm);
  Serial.flush();
  Serial.print("CO(ppm)      : "); Serial.println(CO_ppm);
  Serial.flush();
  Serial.println("====================================");
}
// EOF
