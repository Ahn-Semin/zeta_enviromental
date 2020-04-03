 /* 
  *       Author         Ahn semin
  *       Created        2020.03.24
  *       Last modified  2020.03.31
  */
#include <Timer.h> // https://github.com/JChristensen/Timer
#include <mthread.h> // https://github.com/jlamothe/mthread
#include <SoftwareSerial.h>
#include <stdlib.h>
#include <stdio.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/String.h>
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
#define HCHO_rxPIN    13
#define HCHO_txPIN    12
//#define CO_rxPIN      0
//#define CO_txPIN      0
#define CO_dacPIN     A0
#define NO2_rxPIN     0 // must use 3.3V level shifter
#define NO2_txPIN     0
#define Rn_rxPIN      17
#define Rn_txPIN      18
#define VOC_rxPIN     0
#define VOC_txPIN     0

#define DHT_PIN       2
#define DHTTYPE       DHT22

// other constants
#define CO2               0
#define NO2               1
#define CO2_buffersize    10
#define CO2_len           4
#define NO2_len           255
#define HCHO_len          9
#define ADC_span          (float)1024.0
#define ADC_ref           5
#define MAX_msg_size      60

uint32_t cnt = 0; // for debug

////////////////////////////////////////////////////////////////////////////////
// ROS

ros::NodeHandle  nh;
std_msgs::String air_msg;

ros::Publisher air_pub("air", &air_msg);
// ros::Subscriber<std_msgs::String> led_sub("air_op", airOpCallback);

bool ROS_publish_flag = false;
const uint32_t Time_toPublish = 180;
uint32_t Time_wakeup          = 0;
uint32_t Time_current         = 0;
bool Warm_up_finish_flag      = false;
////////////////////////////////////////////////////////////////////////////////

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

bool Dust_rec_flag       = false;
bool Dust_request_flag   = false;

////////////////////////////////////////////////////////////////////////////////
//CO2
HardwareSerial* CO2_Serial = &Serial3;
uint16_t CO2_ppm           = 0;
char CO2_rec_data[11]      = {'\0',};

bool CO2_request_flag      = false;
bool CO2_rec_flag          = false;

////////////////////////////////////////////////////////////////////////////////
// HCHO
SoftwareSerial HCHO_Serial(HCHO_rxPIN,HCHO_txPIN);
float HCHO_ppb              = 1;
int HCHO_rec_data[HCHO_len] = {0,};

bool HCHO_request_flag      = false;
bool HCHO_rec_flag          = false;

////////////////////////////////////////////////////////////////////////////////
// CO

float CO_ppb               = 1.0;
float CO_zero              = 0.0;
const float CO_full_range  = 500;
const float CO_Vout_span   = 1.6;
const int CO_window_size   = 10;
bool CO_rec_flag           = false;
bool CO_request_flag       = false;
bool CO_filter_flag        = false;

////////////////////////////////////////////////////////////////////////////////
// NO2
uint16_t NO2_ppb = 1;

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
float Rn_nSv = 0.0;
float Rn_nSv_pre = 0.0;
bool Rn_rec_flag = false;
char cmd_M[] = {0x02, cmd_GAMMA_RESULT_QUERY_1MIN, ':'};
const String cmd_M_str = String(cmd_M);
char temp_data[20] = {'\0',};
String temp_str;
////////////////////////////////////////////////////////////////////////////////
// TVOCs
uint16_t TVOCs_ugm3 = 0;
 
////////////////////////////////////////////////////////////////////////////////
// DHT
DHT dht( DHT_PIN, DHTTYPE ); 
float temp_celcius = 0.0;
float hum_RHp = 0.0;

////////////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(air_pub);

  Time_wakeup = millis();
  //Serial.begin(9600);
  Serial.println("arduino wake up");

  main_thread_list->add_thread(new FooThread(1));
  Dust_Serial->begin(9600);
  pms.passiveMode();
  tTimer.every(1000,Dust_callback);
  
  main_thread_list->add_thread(new FooThread(2));
  //Rn_Serial->begin(9600);
  /*Gamma_INIT();
  tTimer.every(1000, Rn_callback); // Request to Gamma Sensor
  */ // do after logic level converter setup!!
  HCHO_Serial.begin(9600);
  tTimer.every(1000, HCHO_callback);
  dht.begin();
  delay(2000);
  
  main_thread_list->add_thread(new FooThread(3));
  pinMode(CO_dacPIN,INPUT);
  tTimer.every(1000, CO_callback);
  
  main_thread_list->add_thread(new FooThread(4));
  CO2_Serial->begin(38400);
  tTimer.every(1000,CO2_callback);

  main_thread_list->add_thread(new FooThread(5)); // thread for publishing date to ROS. change the number of thread
  tTimer.every(1000, Publish_callback);
  
  main_thread_list->add_thread(new FooThread(6)); // thread for updating timers(must set as the last thread)
  //tTimer.every(3000, Show_data);
  
  Serial.println("setup finish");
  Serial.println("====================================");

}

////////////////////loop is here!!!!!!!!!!!!!!!///////////////////
bool FooThread::loop(){
  switch(id){
    case 1 :
      if(Dust_request_flag) {
        Dust_RecUartData();
        Dust_request_flag = false;
      }
      break;
    case 2 :
      if(Rn_request_flag) {
        Rn_RecUartData();
        Rn_request_flag = false;
      }
      if(HCHO_request_flag) {
        HCHO_RecUartData();
        HCHO_request_flag = false;
        //Serial.println(HCHO_ppb);
      }
      break;
    case 3 :
      CO_ADC();
      break;
    case 4 :
      if(CO2_request_flag) {
        CO2_RecUartData();
        CO2_request_flag = false;
      }
      break;
    case 5 :
      if(( nh.connected() & ROS_publish_flag )) {
        Publish_data();
        ROS_publish_flag = false;
        }
      else{
        ;
      }
      nh.spinOnce();
    // publish
      break;
    case 6 :
      tTimer.update();
      break;
  }
  
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


////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
// HCHO
void HCHO_callback() {
  HCHO_request_flag = true;  
}

void HCHO_RecUartData() {
  if(HCHO_Serial.available()>0){
    for(int i = 0; i < HCHO_len; i++){
      HCHO_rec_data[i] = HCHO_Serial.read();
    }
    HCHO_ppb = (HCHO_rec_data[4] * 256 + HCHO_rec_data[5]);
  }
}

// CO
void CO_callback() {
  if(Warm_up_finish_flag) CO_zero_calib();
  CO_request_flag = true;
}

void CO_zero_calib() {
  delay(300);
  for(int i = 0;i<10;i++) {
    CO_zero += (float)analogRead(A0) * ADC_ref / ADC_span;
    delay(100);
  }
  CO_zero = CO_zero / 10.0;
  //nh.loginfo("CO init finish");
  if(CO_zero > 0.44 || CO_zero < 0.39) CO_zero = 0.411;
}

void CO_RecUartData() {
  //Serial.println("CO rec data");
  /*if(CO_Sensor.available(10)) { // wait until 1ms until get data  if wait too long -> overflow
    CO_ppm = CO_Sensor.uartReadPPM();
    CO_rec_flag = true;
  }*/
  //Serial.println("CO rec data");
}

void CO_ADC() {
  static float CO_ppb_arr[10] = {0.0,};
  static int cnt = 0;
  float sum = 0.0;
  if(CO_request_flag) {
    if(cnt==CO_window_size) {
      for(int i=0; i<CO_window_size; i++) {
        sum+= CO_ppb_arr[i];
      }
      CO_ppb = sum / 10.0; // smoothing
      cnt = 0;
    } else {
      CO_ppb_arr[cnt++] = ((float)analogRead(A0) * ADC_ref / ADC_span - CO_zero) * CO_full_range / CO_Vout_span;  
    }
    
    CO_rec_flag = true;
  }
  temp_celcius = dht.readTemperature();
  hum_RHp =  dht.readHumidity();
}

////////////////////////////////////////////////////////////////////////////////
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
    //Serial.println("Radon rec data enter");
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
      Rn_nSv_pre = Rn_nSv;
      Rn_nSv = temp.toFloat();
      if(Rn_nSv - Rn_nSv > 1.0) Rn_nSv = Rn_nSv_pre;  // to ignore noised value
    }
    Rn_rec_flag = true;
  }
  //Serial.println("Radon rec data");
}

////////////////////////////////////////////////////////////////////////////////
// ROS
void Publish_callback() {
  if(Warm_up_finish_flag) ROS_publish_flag = true;
  else Time_current = millis();
  //if(Time_current - Time_wakeup >= Time_toPublish*1000) {
  if(Time_current - Time_wakeup >= 5000) {  // for debugging hard coding -> delete this line
    Warm_up_finish_flag = true;
  }
}

void Publish_data() {
  char send_msg[MAX_msg_size] = "";
  sprintf(send_msg, "%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|",
  Dust_PM2_5_ugm3,    Dust_PM10_ugm3,     CO2_ppm,
  (int)HCHO_ppb,      (int)CO_ppb,        NO2_ppb,
  (int)(Rn_nSv*1000), TVOCs_ugm3,         (int)temp_celcius,
  (int)hum_RHp);  // Rn is nSv unit 
  nh.loginfo(send_msg);
  air_msg.data = send_msg;
  air_pub.publish(&air_msg);
}

////////////////////////////////////////////////////////////////////////////////
// Serial
void Show_data() {
  Serial.print("PM-2.5(ug/m3): ");Serial.println(Dust_PM2_5_ugm3);
  Serial.print("PM-10(ug/m3) : ");Serial.println(Dust_PM10_ugm3);
  Serial.flush();
  Serial.print("Radon(uSv)   : "); Serial.println(Rn_nSv);
  Serial.flush();
  Serial.print("CO2(ppm)     : "); Serial.println(CO2_ppm);
  Serial.flush();
  Serial.print("CO(ppm)      : "); Serial.println(CO_ppb);
  Serial.flush();
  Serial.println("====================================");
}
// EOF
