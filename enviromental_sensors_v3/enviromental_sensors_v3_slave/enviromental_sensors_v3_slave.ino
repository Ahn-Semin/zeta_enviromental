 /* 
  *       Author         Ahn semin
  *       Created        2020.03.24
  *       Last modified  2020.03.31
  *       Description    enviromental sensors slave
  */
  
#include <enviromental_sensors.h>

Timer tTimer;

////////////////////////////////////////////////////////////////////////////////
// NO2
HardwareSerial* NO2_Serial = &Serial1;
uint16_t NO2_ppb = 0;

bool NO2_request_flag = false;
bool NO2_rec_flag = false;
////////////////////////////////////////////////////////////////////////////////
// Radon
HardwareSerial* Rn_Serial = &Serial2;

char Rn_rec_data[20] =  {'\0',}; // Array for received command
bool Rn_request_flag = false; // enable or disable send command automatically
float Rn_nSv = 0.0;
float Rn_nSv_pre = 0.0;
bool Rn_rec_flag = false;
char temp_data[20] = {'\0',};

////////////////////////////////////////////////////////////////////////////////
// TVOCs
uint16_t TVOCs_ugm3         = 0;

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
  Serial.begin(9600); // debug
  
  main_thread_list->add_thread(new FooThread(i++)); 
  tTimer.every(Callback_period, NO2_callback);

  main_thread_list->add_thread(new FooThread(i++)); 
  Rn_Serial->begin(Rn_Serial_Speed);
  Gamma_INIT();
  tTimer.every(Callback_period, Rn_callback); // Request to Gamma Sensor
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
void NO2_callback() {
}

void NO2_RecUartData() {
  
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
  int rec_size = Rn_Serial->available();
  if (rec_size) {
    for(int i = 0; i < rec_size ; i++) {
      Rn_rec_data[i] = Rn_Serial->read();
      temp_data[i] = Rn_rec_data[i];
    }
    Rn_rec_data[rec_size] = '\0';
    String temp = String(Rn_rec_data);
    if(temp.indexOf('.') != -1) ; {
      if(temp.indexOf(':') != -1) temp.remove(0,temp.indexOf(':')+1);
      if(temp.startsWith("0") || temp.startsWith("1")) {
        Rn_nSv_pre = Rn_nSv;
        Rn_nSv = temp.toFloat();
        if(Rn_nSv - Rn_nSv > 1.0) Rn_nSv = Rn_nSv_pre;  // ignore noised value
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
  sprintf(send_data, "%d|%d|%d|%d|", NO2_ppb, (int)Rn_nSv*1000, (int)temp_celcius, (int)hum_RHp);
  Wire.write(send_data);
}
////////////////////////////////////////////////////////////////////////////////
// EOF
