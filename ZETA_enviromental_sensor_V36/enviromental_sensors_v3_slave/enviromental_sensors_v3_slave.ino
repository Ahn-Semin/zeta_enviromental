 /* 
  *       Author         Ahn semin
  *       Created        2020.03.24
  *       Last modified  2020.05.19 16:18
  *       Description    enviromental sensors slave
  */
  
#include "enviromental_sensors.h"

Timer tTimer;

////////////////////////////////////////////////////////////////////////////////
// NO2
HardwareSerial* NO2_Serial = &Serial1;
int NO2_ppb = -1;
char NO2_rec_data[100] = {'\0',};
const int nAverage = 300;

bool NO2_request_flag = false;
bool NO2_rec_flag = false;

const char NO2_CMD_SET_AVERAGE  = 'A';
const char NO2_CMD_CONTINUOUS   = 'c';
const String NO2_HWID           = "093019020151"; // must set sensor HWID to it or not works
////////////////////////////////////////////////////////////////////////////////
// Radon
HardwareSerial* Rn_Serial = &Serial2;

char Rn_rec_data[20] =  {'\0',}; // Array for received command
bool Rn_request_flag = false; // enable or disable send command automatically
float Rn_uSv = 0.0;
float Rn_uSv_pre = 0.0;
float Rn_mBqm3     = 0.0;
bool Rn_rec_flag = false;

////////////////////////////////////////////////////////////////////////////////
// DHT
DHT dht( DHT_PIN, DHTTYPE ); 
float temp_celcius  = 0.0;
float hum_RHp       = 0.0;

bool DHT_request_flag = false;
bool DHT_rec_flag     = false;
////////////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  int i = 1;
  Serial.begin(Serial_Speed); // debug
  
  main_thread_list->add_thread(new FooThread(i++));
  NO2_Serial->begin(NO2_Serial_Speed);
  delay(1000);
  NO2_init();
  tTimer.every(Callback_period, NO2_callback);

  main_thread_list->add_thread(new FooThread(i++)); 
  Rn_Serial->begin(Rn_Serial_Speed);
  Gamma_INIT();
  tTimer.every(Callback_period, Rn_callback); // Request to Gamma Sensor
  delay(1000);
  // do after logic level converter setup!!

  main_thread_list->add_thread(new FooThread(i++)); 
  dht.begin();
  delay(2000);
  tTimer.every(Callback_period, DHT_callback);

  // I2C request & rec are interrupt -> do not need thread
  Wire.begin(SLAVE_arduino);
  Wire.onRequest(I2C_SendData);
  
  main_thread_list->add_thread(new FooThread(i++));
  Serial.println("Slave setup finish");
}

////////////////////loop is here!!!!!!!!!!!!!!!///////////////////
bool FooThread::loop(){
  switch(id){
    case 1 :
      if(NO2_request_flag) {
        NO2_RecUartData();
        NO2_request_flag = false;
      }
      break;
    case 2 :
      if(Rn_request_flag) {
        Rn_RecUartData();
        Rn_request_flag = false;
      }
      break;
    case 3 :
      if(DHT_request_flag) {
        DHT_RecData();
        DHT_request_flag = false;
      }
      break;
    case 4 :
      tTimer.update();
      break;
  }
  
  return true;
}

////////////////////loop end!!!!!!!!!!!!!!!/////////////////////
/*                                    Other functions                                    */
////////////////////////////////////////////////////////////////////////////////
// NO2
void NO2_init() {
  Serial.flush();
  NO2_Serial->flush();
  delay(1000);
  NO2_Serial->write('A');
  delay(1000);
  NO2_Serial->print(nAverage);
  delay(1000);
  NO2_Serial->write('\r');
  delay(1000);
  NO2_Serial->write('c');
  
}

void NO2_callback() {
  NO2_request_flag = true;
}

void NO2_RecUartData() {
  String temp_str;
  String sub_str;
  temp_str.reserve(100);
  sub_str.reserve(15);
  int cIndex = 0;
  int cIndex_pre = 0;
  int i = 0;
  if(NO2_Serial->available() > 62) {
    while(NO2_Serial->available()) {
      NO2_rec_data[i++] = NO2_Serial->read();
      NO2_Serial->flush();
    }
    temp_str = String(NO2_rec_data);
    //Serial.print(temp_str);
    cIndex = temp_str.indexOf(',');
    sub_str = temp_str.substring(0,cIndex);
    if(!sub_str.equals(NO2_HWID)) {
      Serial.println("return"); return;
    }
    temp_str = temp_str.substring(cIndex+1);
    //Serial.println(temp_str);
    cIndex = temp_str.indexOf(',');
    temp_str = temp_str.substring(0, cIndex);
    NO2_ppb = temp_str.toInt();
  }
  
}

////////////////////////////////////////////////////////////////////////////////
// Radon
/* Gamma Sensor Initialize */
void Gamma_INIT() {
  Read_FW(); // Read FW version
  Reset(); // Reset
}

/* Meawurement Reset */
void Reset(){
  byte send_data[6] = {0x02, cmd_GAMMA_RESET, ':', '1', 0x0D, 0x0A};
  Rn_Serial->write(send_data, 6);
  delay(100);
}
//
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
}

void Rn_RecUartData() {
  int i = 0;
  String temp;
  temp.reserve(30);
  if(Rn_Serial->available() > 5) {
    while(Rn_Serial->available()) Rn_rec_data[i++] = Rn_Serial->read();
    Rn_rec_data[i] = '\0';
    temp = String(Rn_rec_data);
    if(temp.indexOf('.') != -1) ; {// why??????????????????
    if(temp.indexOf(':') != -1) temp.remove(0,temp.indexOf(':')+1);
    if(temp.startsWith("0") || temp.startsWith("1") || temp.startsWith("2")) {
      Rn_uSv_pre = Rn_uSv;
      Rn_uSv = temp.toFloat();
      if(Rn_uSv - Rn_uSv_pre > 1.0) ;
      else  Rn_mBqm3 = Rn_uSv * Nano2Milli * HourPerYear * mSvyToBqm3 * 1000;  
    }
    Rn_rec_flag = true;
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
// DHT
void DHT_callback() {
  DHT_request_flag = true;
}

void DHT_RecData() {
  temp_celcius  = dht.readTemperature();
  hum_RHp       = dht.readHumidity();
}

////////////////////////////////////////////////////////////////////////////////
// I2C
void I2C_SendData() {
  char send_data[MAX_I2C_buffersize] = {'\0',};
  sprintf(send_data, "%d|%d|%d|%d|", NO2_ppb, (int)Rn_mBqm3, (int)temp_celcius, (int)hum_RHp);
  Wire.write(send_data);
}
////////////////////////////////////////////////////////////////////////////////
// EOF
