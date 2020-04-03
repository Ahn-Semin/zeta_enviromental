 /* 
  *       Author         Ahn semin
  *       Created        2020.03.24
  *       Last modified  2020.04.01
  *       Description    enviromental sensors master
  */
#include <enviromental_sensors.h>
////////////////////////////////////////////////////////////////////////////////
// ROS

ros::NodeHandle  nh;
std_msgs::String air_msg;

ros::Publisher air_pub("air", &air_msg);

bool ROS_publish_flag = false;
const uint32_t Time_toPublish = 10;        // sec, time before publish sensor data to NUC
uint32_t Time_wakeup          = 0;
uint32_t Time_current         = 0;
bool Warm_up_finish_flag      = false;
////////////////////////////////////////////////////////////////////////////////


Timer tTimer;

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
HardwareSerial* CO2_Serial = &Serial2;
uint16_t CO2_ppm           = 0;
char CO2_rec_data[11]      = {'\0',};

bool CO2_request_flag      = false;
bool CO2_rec_flag          = false;

////////////////////////////////////////////////////////////////////////////////
// HCHO
HardwareSerial* HCHO_Serial = &Serial3;
float HCHO_ppb              = 0;
float HCHO_ugm3             = 0;
float HCHO_zero             = 0;
int HCHO_rec_data[HCHO_len] = {0,};
const int HCHO_window_size  = 60;
uint32_t HCHO_Time_set      = 30000;  // ms, zero calib after this time

bool HCHO_request_flag      = false;
bool HCHO_rec_flag          = false;
bool HCHO_zero_flag         = false;

////////////////////////////////////////////////////////////////////////////////
// CO

float CO_ppb               = 0.0;
float CO_zero              = 0.0;
const float CO_full_range  = 500;
const float CO_Vout_span   = 1.6;
const int CO_window_size   = 10;
uint32_t CO_Time_set       = 30000;  // ms, zero calib after this time
bool CO_rec_flag           = false;
bool CO_request_flag       = false;
bool CO_filter_flag        = false;
bool CO_ZC_flag            = false;

////////////////////////////////////////////////////////////////////////////////
// NO2
uint16_t NO2_ppb = 0;

////////////////////////////////////////////////////////////////////////////////
// Radon
float Rn_mBqm3 = 0.0;

////////////////////////////////////////////////////////////////////////////////
// TVOCs
CCS811 CCS811;
uint16_t TVOCs_ugm3         = 0;

////////////////////////////////////////////////////////////////////////////////
// DHT
float temp_celcius = 0.0;
float hum_RHp = 0.0;

////////////////////////////////////////////////////////////////////////////////
// I2C
char I2C_rec_data[MAX_I2C_buffersize] = {'\0',};

bool I2C_request_flag = false;
////////////////////////////////////////////////////////////////////////////////
void setup() {
  int i = 1;
  nh.getHardware()->setBaud(Serial_Speed);
  nh.initNode();
  nh.advertise(air_pub);
  
  Time_wakeup = millis();
  nh.loginfo("arduino wake up");

  main_thread_list->add_thread(new FooThread(i++));     
  Dust_Serial->begin(Dust_Serial_Speed);
  pms.passiveMode();
  tTimer.every(Callback_period,Dust_callback);

  main_thread_list->add_thread(new FooThread(i++));
  CO2_Serial->begin(CO2_Serial_Speed);
  tTimer.every(Callback_period,CO2_callback);
  
  main_thread_list->add_thread(new FooThread(i++));
  HCHO_Serial->begin(HCHO_Serial_Speed);
  tTimer.every(Callback_period, HCHO_callback);

  main_thread_list->add_thread(new FooThread(i++));
  pinMode(CO_dacPIN,INPUT);
  tTimer.every(Callback_period, CO_callback);

  main_thread_list->add_thread(new FooThread(i++));
  delay(1000); // waiting for slave arduino initialization
  Wire.begin(SLAVE_arduino);
  //if(!ssenseCCS811.begin(uint8_t(I2C_CCS811_ADDRESS), uint8_t(CCS811_WAKE_PIN), driveMode_1sec)) 
  //  nh.loginfo("TVOCs sensor initialization failed.");
  tTimer.every(Callback_period, I2C_callback);
  
  main_thread_list->add_thread(new FooThread(i++)); // thread for publishing date to ROS. change the number of thread
  tTimer.every(Callback_period, Publish_callback);

  main_thread_list->add_thread(new FooThread(i++)); // thread for updating timers(must set as the last thread)
  //tTimer.every(3 * Callback_period, Show_data);   // Serial -> debugging only
  nh.loginfo("       Master setup finish");
  nh.loginfo("====================================");

}

////////////////////loop is here!!!!!!!!!!!!!!!///////////////////
bool FooThread::loop(){
  Time_current = millis();
  switch(id){
    case 1 :
      if(Dust_request_flag) {
        Dust_RecUartData();
        Dust_request_flag = false;
      }
      break;
    case 2 :
      
      if(CO2_request_flag) {
        CO2_RecUartData();
        CO2_request_flag = false;
      }
      
      break;
    case 3 :
      if(HCHO_request_flag) {
        HCHO_RecUartData();
        HCHO_request_flag = false;
      }
      break;
    case 4 :
      if(CO_request_flag) {
        CO_ADC();
        CO_request_flag = false;  
      }
      break;
    case 5 :  // loop for I2C (slave arduino & TVOCs)
      if(I2C_request_flag) {
        //TVOCs_RecData();
        I2C_RecData();
        I2C_request_flag = false;
      }
      break;
    case 6 :
      if(( nh.connected() & ROS_publish_flag )) {
        Publish_data();
        ROS_publish_flag = false;
      }
      else ;
      nh.spinOnce();
    case 7 :
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
  if(HCHO_Serial->available()>0){
    for(int i = 0; i < HCHO_len; i++){
      HCHO_rec_data[i] = HCHO_Serial->read();
    }
    if(Time_current - Time_wakeup > HCHO_Time_set & !HCHO_zero_flag) HCHO_zero_calib();
    HCHO_ppb = (HCHO_rec_data[4] * 256 + HCHO_rec_data[5]) - HCHO_zero + 40;  // 40 is typical HCHO concentration in office enviroment
    HCHO_ugm3 = HCHO_ppb * HCHO_molweight / LiterPerMol;
    if(HCHO_ppb < ZERO_tol) HCHO_ppb = 0.0; // set lower limit
    if(Time_current - Time_wakeup < HCHO_Time_set & !HCHO_zero_flag) HCHO_ppb = -1;
  }
}

void HCHO_zero_calib() {
  static float sum;
  static int cnt;
  sum+= HCHO_ppb;
  if(cnt++>HCHO_window_size - 1) {
    HCHO_zero = sum/(float)HCHO_window_size; 
    HCHO_zero_flag = true;
  }
}

// CO
void CO_callback() {
  if(Time_current - Time_wakeup > CO_Time_set & !CO_ZC_flag) CO_zero_calib();
  CO_request_flag = true;
}

void CO_zero_calib() {
  for(int i = 0;i<10;i++) {
    CO_zero += (float)analogRead(CO_dacPIN) * ADC_ref / ADC_span;
    delay(100);
  }
  CO_zero = CO_zero / 10.0;
  CO_ZC_flag = true;
}

void CO_ADC() {
  static float CO_ppb_arr[CO_window_size];
  static int cnt;
  float sum = 0.0;
  if(cnt==CO_window_size-1) {
    for(int i=0; i<CO_window_size; i++) {
      sum+= CO_ppb_arr[i];
    }
    CO_ppb = sum / (float)CO_window_size * 1000; // smoothing
    if(CO_ppb < ZERO_tol) CO_ppb = 0.0;
    cnt = 0;
  } else {
    CO_ppb_arr[cnt++] = ((float)analogRead(A0) * ADC_ref / ADC_span - CO_zero) * CO_full_range / CO_Vout_span;
  }
  if(Time_current - Time_wakeup < CO_Time_set & !CO_ZC_flag) CO_ppb = -1;
  CO_rec_flag = true;
}

//
// TVOCs  & others
void I2C_callback() {
  I2C_request_flag = true;
}

void TVOCs_RecData() {
  if(temp_celcius > ZERO_tol) CCS811.setEnvironmentalData((float)(temp_celcius), (float)(hum_RHp));
  else CCS811.setEnvironmentalData(TEMP_ref, HUM_ref);
  if (CCS811.checkDataAndUpdate()) TVOCs_ugm3 = CCS811.gettVOC();
}

void I2C_RecData() {
  String rec_data_str;
  Wire.requestFrom(SLAVE_arduino,MAX_I2C_buffersize);
  for(int i = 0; i<MAX_I2C_buffersize; i++) {
    I2C_rec_data[i] = Wire.read();
  }
  rec_data_str = String(I2C_rec_data);
  Split(rec_data_str,'|');
  //nh.loginfo(I2C_rec_data);
}

void Split(String sData, char cSeparator) {  
  int nGetIndex = 0 ;
  String sTemp = "";
  String sCopy = sData;
  
  for(int i = 0; i<4; i++) {
    nGetIndex = sCopy.indexOf(cSeparator);
    sTemp = sCopy.substring(0, nGetIndex);
    sCopy = sCopy.substring(nGetIndex + 1);
    if(i==0) NO2_ppb = sTemp.toInt();
    else if(i==1) Rn_mBqm3 = sTemp.toInt();
    else if(i==2) temp_celcius = sTemp.toInt();
    else if(i==3) hum_RHp = sTemp.toInt();
  }
}

////////////////////////////////////////////////////////////////////////////////
// ROS
void Publish_callback() {
  ROS_publish_flag = true;
  if(Time_current - Time_wakeup >= Time_toPublish*1000 && !Warm_up_finish_flag) {
    Warm_up_finish_flag = true;
  }
  
}

void Publish_data() {
  char send_msg[MAX_msg_size] = "";
  if(Warm_up_finish_flag) {
    sprintf(send_msg, "%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|",
    Dust_PM2_5_ugm3,    Dust_PM10_ugm3,     CO2_ppm,
    (int)HCHO_ppb,      (int)CO_ppb,        NO2_ppb,
    (int)Rn_mBqm3,        TVOCs_ugm3,         (int)temp_celcius,
    (int)hum_RHp);  // Rn is nSv unit
  }
  else {
    sprintf(send_msg, "%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|",
    TRASH,    TRASH,     TRASH,
    TRASH,    TRASH,     TRASH,
    TRASH,    TRASH,     TRASH,
    TRASH);  // Rn is nSv unit
  }
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
  //Serial.print("Radon(uSv)   : "); Serial.println(Rn_nSv);
  Serial.flush();
  Serial.print("CO2(ppm)     : "); Serial.println(CO2_ppm);
  Serial.flush();
  Serial.print("CO(ppm)      : "); Serial.println(CO_ppb);
  Serial.flush();
  Serial.println("====================================");
}
// EOF
