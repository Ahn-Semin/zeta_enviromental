/* 
  *       Author         Ahn semin
  *       Created        2020.03.24
  *       Last modified  2020.03.31
  *       Description    enviromental sensors header
  */
 
#ifndef _ENVIROMENTAL_SENSORS_H_
#define _ENVIROMENTAL_SENSORS_H_
#include <Timer.h> // https://github.com/JChristensen/Timer
#include <mthread.h> // https://github.com/jlamothe/mthread
#include <Wire.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <PMS.h>  // Matter sensor
#include <ZE07CO_Sensor.h> // CO sensor
#include <sSense-CCS811.h>  // TVOCs sensor

#include <DHT_U.h>
#include <DHT.h>  // Humidity & temp. sensor




#define Number_of_Thread 0  // 4 for sensors, 1 for ROS publishing, 1 for loop, 1 for I2C

// pin config
#define Dust_rxPIN    19  // must use with 3.3V level shifter
#define Dust_txPIN    18
#define CO2_rxPIN     17
#define CO2_txPIN     16
#define HCHO_rxPIN    15
#define HCHO_txPIN    14
//#define CO_rxPIN      0
//#define CO_txPIN      0
#define CO_dacPIN     A0
#define VOC_rxPIN     0
#define VOC_txPIN     0

#define NO2_rxPIN     19 // must use with 3.3V level shifter
#define NO2_txPIN     18
#define Rn_rxPIN      17
#define Rn_txPIN      16
#define DHT_PIN       2
#define DHTTYPE       DHT22

// other constants
#define Serial_Speed              9600
#define Dust_Serial_Speed         9600
#define CO2_Serial_Speed          38400
#define HCHO_Serial_Speed         9600
#define NO2_Serial_Speed          9600
#define Rn_Serial_Speed           9600
#define Callback_period           1000  // ms

#define SLAVE_arduino             0x01  // do not set this address as same with TVOCs sensor's address
#define MAX_I2C_buffersize        20

#define MAX_msg_size              60

#define CO2                       0
#define NO2                       1

#define CO2_buffersize            10
#define CO2_len                   4
#define NO2_len                   255
#define HCHO_len                  9
#define ADC_span                  (float)1024.0
#define ADC_ref                   5


#define TEMP_ref                  21.0
#define HUM_ref                   50.0
#define ZERO_tol                  0.01
#define HCHO_molweight            30.031
#define LiterPerMol               22.4
#define HourPerYear               8760
#define Nano2Milli                0.000001
#define mSvyToBqm3                58.824
#define TRASH                     -1


/* Radon Command */
enum {
    cmd_GAMMA_RESULT_QUERY      = 0x44, // D, Read measuring value(10min avg, 1min update)
    cmd_GAMMA_RESULT_QUERY_1MIN = 0x4D, // M, Read measuring value(1min avg, 1min update)
    cmd_GAMMA_PROC_TIME_QUERY   = 0x54, // T, Read measuring time
    cmd_GAMMA_MEAS_QUERY        = 0x53, // S, Read status
    cmd_GAMMA_FW_VERSION_QUERY  = 0x46, // F, Read firmware version
    cmd_GAMMA_VIB_STATUS_QUERY  = 0x56, // V, Response vibration status
    cmd_GAMMA_RESET             = 0x52, // R, Reset
    cmd_GAMMA_AUTO_SEND         = 0x55, // U, Set Auto_send status
    cmd_GAMMA_ALL_QUERY         = 0x41, // A, Read all data
};

/* Thread class for receive UART */
class FooThread : public Thread{
    public: FooThread(int id);
    protected: bool loop();
    private: int id;
};

FooThread::FooThread(int id){
  this->id = id;
}

#endif
// EOF