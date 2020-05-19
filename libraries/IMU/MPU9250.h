//----------------------------------------------------------------------------
//    프로그램명 	: MPU6050
//
//    만든이     	: Jae Yong Seo ( sjyong@humanoidsystem.com )
//
//    최종 수정  	: 2019.05.25
//
//    파일명     	: MPU6050.h
//----------------------------------------------------------------------------
#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <inttypes.h>
//#include <Arduino.h>
#include <Wire.h>    // I2C library
#include <SPI.h>

#include "Define.h"
#include "MPU9250_REGS.h"
#include "MadgwickAHRS.h"

#define ACC_1G     512
#define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0) //16.4 LSB = 1 deg/s

#define FILTER_NUM    3

class cMPU9250
{
//	const uint8_t MPU9250_ADDRESS {0x68};  // Device address when ADO = 0
//	const uint8_t AK8963_ADDRESS {0x0C};   //  Address of magnetometer

//	const uint8_t MPU9250_WHOAMI_DEFAULT_VALUE {0x71}; // 0x68????
//	const uint8_t AK8963_WHOAMI_DEFAULT_VALUE {0x48};
	
public:
	bool     bConnected;

	int16_t angle[3];
	float   rpy[3];
	float   quat[4];

	int16_t  gyroADC[3];
	int16_t  gyroRAW[3];
	int16_t  gyroZero[3];

	int16_t  accADC[3];
	int16_t  accRAW[3];
	int16_t  accZero[3];

	int16_t  magADC[3];
	int16_t  magRAW[3];
	int16_t  magZero[3];
	
	int16_t gyroData[3];
	int16_t gyroRaw[3];
	int16_t accData[3];
	int16_t accRaw[3];
	int16_t magData[3];
	int16_t magRaw[3];

	float ax, ay, az;
	float gx, gy, gz;
	float mx, my, mz;

	int16_t  accSmooth[3];

	uint16_t calibratingG;
	uint16_t calibratingA;
	uint16_t calibratingM;
	
	int32_t update_hz;
	uint32_t update_us;


	float AK8963_ASA[3];
	float   SelfTest[6];    // holds results of gyro and accelerometer self test
	float gyroBias[3] = {0, 0, 0} , accelBias[3] = {0, 0, 0};	// Bias corrections for gyro and accelerometer
	
	Madgwick filter;

	// Set initial input parameters
	enum Ascale {
	  AFS_2G = 0,
	  AFS_4G,
	  AFS_8G,
	  AFS_16G
	};

	enum Gscale {
	  GFS_250DPS = 0,
	  GFS_500DPS,
	  GFS_1000DPS,
	  GFS_2000DPS
	};

	enum Mscale {
	  MFS_14BITS = 0, // 0.6 mG per LSB
	  MFS_16BITS      // 0.15 mG per LSB
	};
	
	// Specify sensor full scale
	// Specify sensor full scale
	uint8_t Gscale = GFS_2000DPS;
	uint8_t Ascale = AFS_8G;
	uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
	uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

	int16_t tempCount;      // temperature raw count output
	float   temperature;    // Stores the real internal chip temperature in degrees Celsius

public:
	cMPU9250();

	bool begin( void );
	
	void initMPU9250();
	void MPU9250Init( void );
	bool isConnectedMPU9250();
	void MPU9250SelfTest(float * destination);
	void calibrateMPU9250(float * dest1, float * dest2);
	
	bool isConnectedAK8963();
	void initAK8963(void);
	void gyro_init( void );
	void gyro_get_adc( void );
	void gyro_common();
	void gyro_cali_start();
	bool gyro_cali_get_done();
	void getGres();
	

	void acc_init( void );
	void acc_get_adc( void );
	void acc_common();
	void acc_cali_start();
	bool acc_cali_get_done();
	void getAres();

	void mag_init( void );
	void mag_get_adc( void );
	void mag_common();
	void mag_cali_start();
	bool mag_cali_get_done();
	void getMres();
	
	uint16_t update( void);
	void computeIMU( void );
	
	int16_t readTempData();
	
	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
	uint8_t readByte(uint8_t address, uint8_t subAddress);
	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
	void readAK8963Bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest);
	void writeAK8963Byte(uint8_t address, uint8_t subAddress, uint8_t data);
	void pirntI2CError();
	
protected:
	//uint8_t _address;
    //TwoWire *_i2c;
    const uint32_t _i2cRate = 400000; // 400 kHz
    size_t _numBytes; // number of bytes received from I2C	
	uint8_t _status;
	uint8_t i2c_err_;

};


#endif
