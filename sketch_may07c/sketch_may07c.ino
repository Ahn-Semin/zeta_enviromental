// -*- mode: c++ -*-
//#include <IMU.h>
#include <mthread.h>
#include <Timer.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/String.h>

#include <MPU9250.h>


#define __DEBUG
#define _DEBUG

// Define commands

#define SEND_IMUDATA          0x10
#define SEND_IMUDATA_ONCE     0x11
#define SEND_IMUDATA_CONT     0x12
#define STOP_IMUDATA_CONT     0x13
#define SEND_IMUCALIGYRO      0x14

#define SEND_USDATA           0x20
#define SEND_USDATA_ONCE      0x21
#define SEND_USDATA_CONT      0x22
#define STOP_USDATA_CONT      0x23

#define LEFTFRONT_EN          22
#define LEFTFRONT_PW          23
#define RIGHTFRONT_EN         30
#define RIGHTFRONT_PW         31
#define LEFTREAR_EN           24
#define LEFTREAR_PW           25
#define RIGHTREAR_EN          32
#define RIGHTREAR_PW          33
#define REARLEFT_EN           26
#define REARLEFT_PW           27
#define REARCENTER_EN         34
#define REARCENTER_PW         35
#define REARRIGHT_EN          28
#define REARRIGHT_PW          29

#define LF                    0
#define RF                    1
#define LR                    2
#define RR                    3
#define R_L                   4
#define R_C                   5
#define R_R                   6

cMPU9250 SEN;  



//cIMU imu_;

union INTVAL {
  char cval[4];
  int ival;
} int_val;

union UINTVAL {
  char cval[4];
  uint32_t uival;
} uint_val;

union SHORTVAL {
  char cval[2];
  short sval;
} short_val;

union WNTVAL {
  char cval[2];
  uint16_t wval;
} word_val;

union FLOATVAL {
  char cval[4];
  float fval;
} float_val;

byte startcnt = 0;
byte b_StartFlag = 0;
byte rsize = 0;



char RData[50]={0};
char WData[65] = {0};

//-------- IMU ---------------

bool bConnected;
const float G = -9.807f;
const float Deg2Rad = 3.14159265359f/180.0f;

float AccelFactor;
float GyroFactor;
float MagFactor[3];

uint16_t IMULoopFreq = 50;    // Hz
unsigned long IMUStartTime;
unsigned long IMUEndTime;
unsigned long IMULoopTime;

bool b_IMUSendCont = false;



//#define ACCEL_FACTOR          -0.000598  // 2.0 * -9.8 / 32768
//#define GYRO_FACTOR           0.000133  // pi / (131 * 180)
//#define MAG_FACTOR            6e-7

/*int16_t angle[3];
float   rpy[3];
float   quat[4];
int16_t gyroData[3];
int16_t gyroRaw[3];
int16_t accData[3];
int16_t accRaw[3];
int16_t magData[3];
int16_t magRaw[3];

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;



float aRes;
float gRes;
float mRes;

uint32_t update_hz;
uint32_t update_us;*/
 

//20200507


Timer tTimer;
int tt1 = 0;

bool t1flag = false;
bool t2flag = true;

ros::Time stamp_now;

// Our custom Thread:


ros::Time rosNow(void);


void test1exe();

class FooThread : public Thread
{
public:
    FooThread(int id);
protected:
    bool loop();
private:
    int id;
};


FooThread::FooThread(int id)
{
    this->id = id;
}




ros::NodeHandle  nh;


sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu", &imu_msg);

sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("/imu/mag", &mag_msg);

std_msgs::String tempstr;
ros::Publisher temppub("temppub",&tempstr);


void test1exe(){
  digitalWrite(PJ1, HIGH);
  nh.loginfo("test1exe start");
  SEN.update();
  SaveIMUData();
  tempstr.data = "tempstr";
  temppub.publish(&tempstr);
  nh.spinOnce();
  stamp_now = rosNow();
  imu_msg.header.stamp = stamp_now;
  imu_pub.publish(&imu_msg);
  mag_msg.header.stamp    = stamp_now;
  mag_pub.publish(&mag_msg);
  //printRawData();
  //printOrient();
  digitalWrite(PJ1, LOW);
  nh.loginfo("test1exe end");
}


void setup() {

  uint32_t i;
  uint32_t pre_time;
  int ii = 1;
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(mag_pub);
  nh.advertise(temppub);
  Serial.begin(115200);
  delay(1000);
  Serial.println("testsetste");
  
  delay(1000);
  nh.loginfo("arduino connected");
  nh.loginfo("setup");
/*
  // Serial 0
  pinMode(PE1, OUTPUT);
  pinMode(PE0, INPUT);

  // Serial 1
  pinMode(PD3, OUTPUT);
  pinMode(PD2, INPUT);
*/  

 
  delay(500);
 

  //delay(1000);

 
  //cMPU9250 SEN;
  bConnected = SEN.begin();

  AccelFactor = SEN.aRes*G;
  GyroFactor = SEN.gRes*Deg2Rad;
  for(int i=0; i<3; i++)
    MagFactor[i] = SEN.mRes*SEN.AK8963_ASA[i];
 
  b_StartFlag = 0;
  startcnt = 0;
  rsize = 0;
 
  pinMode(PJ1, OUTPUT);
  pinMode(PJ0, OUTPUT);

  pinMode(PH1, OUTPUT);
  pinMode(PH0, OUTPUT);

  IMULoopTime = 1000000/IMULoopFreq;
 
  digitalWrite(PJ0, LOW);
  digitalWrite(PJ1, LOW);

  digitalWrite(PH0, LOW);
  digitalWrite(PH1, LOW);

  setLedToggle(PJ0);
  delay(500);
  setLedToggle(PJ0);
  delay(500);
  setLedToggle(PJ0);
  delay(500);
  setLedToggle(PJ0);
  delay(500);    
 
  // Create five threads and add them to the main ThreadList:
  main_thread_list->add_thread(new FooThread(ii++));
  main_thread_list->add_thread(new FooThread(ii++));
  
  tTimer.every(1000,test1);
  tTimer.every(1000,test2);
  
  main_thread_list->add_thread(new FooThread(ii++));
 


}



ros::Time rosNow()
{
  return nh.now();
}


// Our custom Thread:









void test1(){
  t1flag = true;
  nh.loginfo("test1");
}

void test2(){
  t2flag = true;
  nh.loginfo("test2");
}







/*#
void InitSonar(void)
{
  pinMode(LEFTFRONT_PW, INPUT);
  pinMode(RIGHTFRONT_PW, INPUT);
  pinMode(LEFTREAR_PW, INPUT);
  pinMode(RIGHTREAR_PW, INPUT);
  pinMode(REARLEFT_PW, INPUT);
  pinMode(REARCENTER_PW, INPUT);
  pinMode(REARRIGHT_PW, INPUT);
 
  pinMode(LEFTFRONT_EN, OUTPUT);
  pinMode(RIGHTFRONT_EN, OUTPUT);
  pinMode(LEFTREAR_EN, OUTPUT);
  pinMode(RIGHTREAR_EN, OUTPUT);
  pinMode(REARLEFT_EN, OUTPUT);
  pinMode(REARCENTER_EN, OUTPUT);
  pinMode(REARRIGHT_EN, OUTPUT);

  digitalWrite(LEFTFRONT_EN, LOW);
  digitalWrite(RIGHTFRONT_EN, LOW);
  digitalWrite(LEFTREAR_EN, LOW);
  digitalWrite(RIGHTREAR_EN, LOW);
  digitalWrite(REARLEFT_EN, LOW);
  digitalWrite(REARCENTER_EN, LOW);
  digitalWrite(REARRIGHT_EN, LOW);
 
}*/

/*#
int ReadSonar(int snum)
{
  uint32_t st, et;
 
    digitalWrite(sonar_enable[snum], HIGH);  
   
    delay(1);

#ifdef ___DEBUG
    st = micros();
#endif
   
    _max_time = micros() + MAXDELAYTIME;
    while(!digitalRead(sonar_pwm[snum])) {
      if(micros() > _max_time) {    // start timeout : _max_time
        duration[snum] = 0;
        return -1;
      }
    }

#ifdef ___DEBUG
    et = micros();

    Serial.print("start run time diff(");
    Serial.print(snum+1);
    Serial.print("): ");
    Serial.println(et - st);
   
    Serial.print("start max time : "); Serial.println(_max_time);
#endif    
   
    start_time[snum] = micros();

    _max_time = micros() + MAXDELAYTIME;


#ifdef ___DEBUG
    st = micros();
#endif
   
    while(digitalRead(sonar_pwm[snum])) {
      if(micros() > _max_time) {    // end timeout : _max_time
        duration[snum] = 0;
        return -2;
      }
    }
    end_time[snum] = micros();

#ifdef ___DEBUG
    et = micros();

    Serial.print("end run time diff(");
    Serial.print(snum+1);
    Serial.print("): ");
    Serial.println(et - st);
#endif    

    duration[snum] = end_time[snum] - start_time[snum];

#ifdef ___DEBUG
    Serial.print("duration : "); Serial.println(duration[snum]);
#endif    

    USFilter1(snum);
   
    distance[snum] = duration[snum]*2.54/147.0;

    digitalWrite(sonar_enable[snum], LOW);  

    return 1;

#if 0
  if(sonarReadCnt == 1) {
    digitalWrite(LEFTFRONT_EN, LOW);  
    digitalWrite(RIGHTREAR_EN, LOW);  
    digitalWrite(REARLEFT_EN, LOW);

    //delay(2);

    while(!digitalRead(LEFTFRONT_PW)) ;
    start_time[LF] = micros();

    while(!digitalRead(RIGHTREAR_PW)) ;
    start_time[RR] = micros();

    while(!digitalRead(REARLEFT_PW)) ;
    start_time[R_L] = micros();

    //----------------------------------

    while(digitalRead(LEFTFRONT_PW)) ;
    end_time[LF] = micros();

    while(digitalRead(RIGHTREAR_PW)) ;
    end_time[RR] = micros();

    while(digitalRead(REARLEFT_PW)) ;
    end_time[R_L] = micros();

    duration[LF] = end_time[LF] - start_time[LF];
    duration[LF] = end_time[LF] - start_time[LF];
  }
#endif
   
}

void USFilter1(int snum)
{
  int i;
 
  USRawData[snum][0] = duration[snum];

  USRawDSum = 0;

  for(i=0; i<USFILTERNUM; i++)
    USRawDSum += USRawData[snum][i];

  USFilteredData[snum] = USRawDSum/USFILTERNUM;

  for(i=USFILTERNUM-1; i>0; i--)
    USRawData[snum][i] = USRawData[snum][i-1];

  distance[snum] = USFilteredData[snum]*2.54/147.0;

}*/


void SaveIMUData(void)
{
  nh.loginfo("SaveIMU start");
  imu_msg.angular_velocity.x = SEN.gyroADC[0] * GyroFactor;
 
  imu_msg.angular_velocity.y = SEN.gyroADC[1] * GyroFactor;
 
  imu_msg.angular_velocity.z = SEN.gyroADC[2] * GyroFactor;
 
  imu_msg.linear_acceleration.x = SEN.accADC[0] * AccelFactor;
 
  imu_msg.linear_acceleration.y = SEN.accADC[1] * AccelFactor;

  imu_msg.linear_acceleration.z = SEN.accADC[2] * AccelFactor;
 
  mag_msg.magnetic_field.x = SEN.magADC[0] * MagFactor[0];

  mag_msg.magnetic_field.y = SEN.magADC[1] * MagFactor[1];
 
  mag_msg.magnetic_field.z = SEN.magADC[2] * MagFactor[2];
 
  imu_msg.orientation.w = SEN.quat[0];

  imu_msg.orientation.x = SEN.quat[1];

  imu_msg.orientation.y = SEN.quat[2];

  imu_msg.orientation.z = SEN.quat[3];
  nh.loginfo("SaveIMU finish");
  //Temperature = SEN.temperature;
}


void printRawData()
{
  // Print acceleration values in milligs!
  /*Serial.print("ax = "); Serial.print(SEN.ax, 2);
  Serial.print(" ay = "); Serial.print(SEN.ay, 2);
  Serial.print(" az = "); Serial.print(SEN.az, 2); Serial.println(" mg");
  // Print gyro values in degree/sec
  Serial.print("gx = "); Serial.print(SEN.gx, 2);
  Serial.print(" gy = "); Serial.print(SEN.gy, 2);
  Serial.print(" gz = "); Serial.print(SEN.gz, 2); Serial.println(" deg/s");
  // Print mag values in degree/sec
  Serial.print("mx = "); Serial.print(SEN.mx, 2);
  Serial.print(" my = "); Serial.print(SEN.my, 2);
  Serial.print(" mz = "); Serial.print(SEN.mz, 2); Serial.println(" mG");
  */
  /*
  Serial.print("q0 = "); Serial.print(SEN.quat[0], 6);
  Serial.print(" qx = "); Serial.print(SEN.quat[1], 6);
  Serial.print(" qy = "); Serial.print(SEN.quat[2], 6);
  Serial.print(" qz = "); Serial.println(SEN.quat[3], 6);
  */
  /*Serial.print("angle0 = "); Serial.print(SEN.angle[0]);
  Serial.print(" angle1 = "); Serial.print(SEN.angle[1]);
  Serial.print(" angle2 = "); Serial.println(SEN.angle[2]);*/

  /*Serial.print("ax = "); Serial.print(SEN.accADC[0] * ACCEL_FACTOR, 2);
  Serial.print(" ay = "); Serial.print(SEN.accADC[1] * ACCEL_FACTOR, 2);
  Serial.print(" az = "); Serial.print(SEN.accADC[2] * ACCEL_FACTOR, 2); Serial.println(" mg");
  // Print gyro values in degree/sec
  Serial.print("gx = "); Serial.print(SEN.gyroADC[0] * GYRO_FACTOR, 2);
  Serial.print(" gy = "); Serial.print(SEN.gyroADC[1] * GYRO_FACTOR, 2);
  Serial.print(" gz = "); Serial.print(SEN.gyroADC[2] * GYRO_FACTOR, 2); Serial.println(" deg/s");
  // Print mag values in degree/sec
  Serial.print("mx = "); Serial.print(SEN.magADC[0] * MAG_FACTOR, 2);
  Serial.print(" my = "); Serial.print(SEN.magADC[1] * MAG_FACTOR, 2);
  Serial.print(" mz = "); Serial.print(SEN.magADC[2] * MAG_FACTOR, 2); Serial.println(" mG");
 
  Serial.print("q0 = "); Serial.print(SEN.quat[0], 2);
  Serial.print(" qx = "); Serial.print(SEN.quat[1], 2);
  Serial.print(" qy = "); Serial.print(SEN.quat[2], 2);
  Serial.print(" qz = "); Serial.println(SEN.quat[3], 2);*/
 
}

void printOrient(void)
{
  double theta;
  //Serial.print("q0 = "); Serial.print(SEN.quat[0], 2);
  //Serial.print(" qx = "); Serial.print(SEN.quat[1], 2);
  //Serial.print(" qy = "); Serial.print(SEN.quat[2], 2);
  //Serial.print(" qz = "); Serial.println(SEN.quat[3], 2);
 
  theta = atan2f(SEN.quat[1] * SEN.quat[2] + SEN.quat[0] * SEN.quat[3],
    0.5f - SEN.quat[2] * SEN.quat[2] - SEN.quat[3] * SEN.quat[3]);

  Serial.print(" theta = "); Serial.println(theta, 8);
}
   
void setLedToggle(uint8_t pin)
{
  digitalWrite(pin, !digitalRead(pin));
}


void calibrationGyro()
{
  uint32_t pre_time;
  uint32_t t_time;

  const uint8_t led_ros_connect = 3;

  SEN.gyro_cali_start();
 
  t_time   = millis();
  pre_time = millis();

  while(!SEN.gyro_cali_get_done())
  {
    SEN.update();

    if (millis()-pre_time > 5000)
    {
      break;
    }
    if (millis()-t_time > 100)
    {
      t_time = millis();
      setLedToggle(led_ros_connect);
    }
  }
}




/*#
void ReadUSSensor()
{
  int res;
 
  res = ReadSonar(LF);
  if(res<0) {
    distance[LF] = res;
  }
   
  res = ReadSonar(RR);
  if(res<0)
    distance[RR] = res;
   
  res = ReadSonar(R_R);
  if(res<0)
    distance[R_R] = res;

#ifdef _DEBUG
  //if(distance[LF] >= 0)
    Serial.print(distance[LF]);
  //else
  //  Serial.print(distance[LF]);
 
  Serial.print(",");

  //if(distance[RR] >= 0)
    Serial.print(distance[RR]);    
  Serial.print(",");

  //if(distance[R_R] >= 0)
    Serial.println(distance[R_R]);
  Serial.print(",");
#endif

  res = ReadSonar(LR);
  if(res<0)
    distance[LR] = res;
   
  res = ReadSonar(RF);
  if(res<0)
    distance[RF] = res;
   
  res = ReadSonar(R_L);
  if(res<0)
    distance[R_L] = res;

#ifdef _DEBUG  
  //if(distance[LR] >= 0)
    Serial.print(distance[LR]);    
  Serial.print(",");

  //if(distance[RF] >= 0)
    Serial.print(distance[RF]);
  Serial.print(",");

  //if(distance[R_L] >= 0)
    Serial.println(distance[R_L]);
  Serial.print(",");
#endif

  res = ReadSonar(R_C);
  if(res<0)
    distance[R_C] = res;

#ifdef _DEBUG  
  //if(distance[R_C] >= 0)
    Serial.println(distance[R_C]);
#endif
 
}
*/

/*
void loop() {
  digitalWrite(PJ1, HIGH);

#if 0
  ReadSonar(LF);
  Serial.print("LF:");
  Serial.println(distance[Lf]);

  ReadSonar(LR);
  Serial.print("LR:");
  Serial.println(distance[LR]);

  ReadSonar(RF);
  Serial.print("RF:");
  Serial.println(distance[RF]);

  ReadSonar(RR);
  Serial.print("RR:");
  Serial.println(distance[RR]);

  ReadSonar(R_L);
  Serial.print("R_L:");
  Serial.println(distance[R_L]);

  ReadSonar(R_C);
  Serial.print("R_C:");
  Serial.println(distance[R_C]);

  ReadSonar(R_R);
  Serial.print("R_R:");
  Serial.println(distance[R_R]);
#endif

#if 0  
  ReadSonar(LF);
  ReadSonar(RR);
  ReadSonar(R_R);
 
  Serial.print("LF:");
  Serial.print(distance[LF]);
  Serial.print(" ");
  Serial.print("RR:");
  Serial.print(distance[RR]);
  Serial.print(" ");
  Serial.print("R_R:");
  Serial.println(distance[R_R]);
  Serial.print(" ");

  ReadSonar(LR);
  ReadSonar(RF);
  ReadSonar(R_L);
  Serial.print("LR:");
  Serial.print(distance[LR]);
  Serial.print(" ");
  Serial.print("RF:");
  Serial.print(distance[RF]);
  Serial.print("R_L:");
  Serial.println(distance[R_L]);
  Serial.print(" ");

  ReadSonar(R_C);
  Serial.print("R_C:");
  Serial.println(distance[R_C]);
#endif

  if( true) {

      digitalWrite(PJ1, HIGH);
     
      SEN.update();

      //SendIMUData();

      printRawData();
      //printOrient();

      digitalWrite(PJ1, LOW);

    }  
 

#if 0
  if(b_USSendCont == true) {
    USEndTime = micros();
    if((USEndTime - USStartTime) > USLoopTime) {
      digitalWrite(PH1, HIGH);

        ReadUSSensor();

        SendUSData();
       
      digitalWrite(PH1, LOW);

      USStartTime = micros();
    }
  }
#endif

 
#if 0  
  digitalWrite(PJ1, LOW);

  SEN.update();

  //printRawData();
  printOrient();

  Serial1.println("Send data...");
#endif
 
  delay(20);
  //delay(500);

}
*/

bool FooThread::loop()
{

  switch(id){
    case 1 :
      if(t1flag){
      test1exe();
      t1flag = false;
      
      }
           //tTimer.update();
      nh.loginfo("id1");
      break;    
     
    case 2 :
      if(t2flag){
      //Serial.print("FooThread ");
      //Serial.println(" called.");
      
      t2flag = false;
      }           //tTimer.update();
      nh.loginfo("id2");
      
      break;  

    case 3 :
      nh.loginfo("id3 start");
      tTimer.update();
      nh.loginfo("id3 finish");
      break;
  }
    return true;
 
}
