//----------------------------------------------------------------------------
//    프로그램명 	: MPU6050
//
//    만든이     	: Jae Yong Seo ( sjyong@humanoidsystem.com )
//
//    최종 수정  	: 2019.05.25
//
//    파일명     	: MPU9250.cpp
//----------------------------------------------------------------------------

//#include <Arduino.h>
#include "MPU9250.h"


#define MPU_CS_PIN          BDPIN_SPI_CS_IMU
#define MPU9250_ADDRESS     0x68
#define MPU_CALI_COUNT      512

#define ACC_ORIENTATION(X, Y, Z)  {accADC[PITCH]  =  Y; accADC[ROLL]  =  X; accADC[YAW]  =   Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[PITCH] =  Y; gyroADC[ROLL] =  X; gyroADC[YAW] =   Z;}

#define ENABLE		0x01

/*---------------------------------------------------------------------------
     TITLE   : cMPU9250
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cMPU9250::cMPU9250()
{
	calibratingG = 0;
	calibratingA = 0;
	calibratingM = 0;
	bConnected   = false;
}



/*---------------------------------------------------------------------------
     TITLE   : cMPU9250
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU9250::begin()
{
	uint8_t m_whoami = 0x00;
	uint8_t a_whoami = 0x00;
	uint32_t pre_time;
		
	Wire.begin();
    //_i2c->begin();
	//Serial.println("inner begin: begin");
	
    // setting the I2C clock
	Wire.setClock(_i2cRate);
    //_i2c->setClock(_i2cRate);
	//Serial.println("inner begin: setclock");

	//pinMode( MPU_CS_PIN, OUTPUT );

	//Wire.begin();
	
	//_i2c = &Wire;
	
	//_address = MPU9250_ADDRESS;
	
	//pinMode(38, OUTPUT);	// PD7 pin : CS
	//digitalWrite(38, LOW);
	
	// starting the I2C bus
	//Serial.println("inner begin: start ");
	
	/*MPU_SPI.begin();
	MPU_SPI.setDataMode( SPI_MODE3 );
	MPU_SPI.setBitOrder( MSBFIRST );
	MPU_SPI.setClockDivider( SPI_CLOCK_DIV128 ); // 108Mhz/128 = 0.8MHz
	digitalWrite(MPU_CS_PIN, HIGH);
	delay( 100 );*/
	
	// Display ID of sensor
	Serial.println("MPU9250 9-DOF 16-bit motion sensor 60ug LSB");
	delay(1000);

	m_whoami = isConnectedMPU9250();
	delay(1000);
	
	if (m_whoami)
	//if( readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250) == MPU9250_WHOAMI_DEFAULT_VALUE )
	{
		bConnected = true;
		
		Serial.println(F("MPU9250 is online..."));

		MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
		
		Serial.print(F("x-axis self test: acceleration trim within : ")); 
		Serial.print(SelfTest[0],1); 
		Serial.println(F("% of factory value"));
		Serial.print(F("y-axis self test: acceleration trim within : ")); 
		Serial.print(SelfTest[1],1); 
		Serial.println(F("% of factory value"));
		Serial.print(F("z-axis self test: acceleration trim within : ")); 
		Serial.print(SelfTest[2],1); 
		Serial.println(F("% of factory value"));
		Serial.print(F("x-axis self test: gyration trim within : ")); 
		Serial.print(SelfTest[3],1); 
		Serial.println(F("% of factory value"));
		Serial.print(F("y-axis self test: gyration trim within : ")); 
		Serial.print(SelfTest[4],1); 
		Serial.println(F("% of factory value"));
		Serial.print(F("z-axis self test: gyration trim within : ")); 
		Serial.print(SelfTest[5],1); 
		Serial.println(F("% of factory value"));
	
		calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

		Serial.println(F("MPU9250 bias"));
		Serial.println("\tx\ty\tz");
		Serial.print(String('\t') + String((int)(1000*accelBias[0]))); 
		Serial.print(String('\t') + String((int)(1000*accelBias[1]))); 
		Serial.print(String('\t') + String((int)(1000*accelBias[2]))); 
		Serial.println(F("\tmg"));

		Serial.print(String('\t')) ; 
		Serial.print(gyroBias[0], 1); 
		Serial.print(String('\t')) ; 
		Serial.print(gyroBias[1], 1); 
		Serial.print(String('\t')) ; 
		Serial.print(gyroBias[2], 1); 
		Serial.println(F("\to/s"));
		delay(1000); 
	
		//initMPU9250();
		MPU9250Init();
		
		Serial.println(F("MPU9250 initialized for active data mode....")); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

		a_whoami = isConnectedAK8963();
		
		delay(1000); 
		
		if (a_whoami)
		{
			initAK8963();
		}
		else
		{
			Serial.print("Could not connect to AK8963: 0x");
			Serial.println(a_whoami);
		}
			
		gyro_init();
		acc_init();
		mag_init();

		getGres();
		getAres();
		getMres();

		update_hz = 200;
		update_us = 1000000/update_hz;
		filter.begin(update_hz);

		for (int i=0; i<32; i++)
		{
			update();
		}

		pre_time = millis();
		while(!gyro_cali_get_done())
		{
			update();

			if (millis()-pre_time > 5000)
			{
				break;
			}
		}

		//MPU_SPI.setClockDivider( SPI_CLOCK_DIV8 ); // 13MHz
	}



	return bConnected;
}

/*---------------------------------------------------------------------------
     TITLE   : isConnectedMPU9250
     WORK    :
     ARG     : void
     RET     : bool
---------------------------------------------------------------------------*/
bool cMPU9250::isConnectedMPU9250()
{
	String s;
	byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	s = String("MPU9250: I AM ") + String(c, HEX) + String(". I should be ") + String(0x71, HEX);
	Serial.println(s);
  
	//Serial.print("MPU9250 WHO AM I = ");
	//Serial.println(c, HEX);
	return (c == MPU9250_WHOAMI_DEFAULT_VALUE);
}

/*---------------------------------------------------------------------------
     TITLE   : isConnectedAK8963
     WORK    :
     ARG     : void
     RET     : bool
---------------------------------------------------------------------------*/
bool cMPU9250::isConnectedAK8963()
{
	String s;
	byte c = readByte(AK8963_ADDRESS, MPU9250_AK8963_WHO_AM_I);
	s = String("AK8963: I AM ") + String(c, HEX) + String(". I should be ") + String(0x48, HEX);
	Serial.println(s);
	//Serial.print("AK8963  WHO AM I = ");
	//Serial.println(c, HEX);
	return (c == AK8963_WHOAMI_DEFAULT_VALUE);
}



/*---------------------------------------------------------------------------
     TITLE   : init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::MPU9250Init( void )
{
	uint8_t state;
	uint8_t data;

	Serial.println("Init...");

	//MPU9250 Reset
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00);
	//writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, MPU9250_RESET);
	delay(100);

	//MPU9250 Set Clock Source
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1,  0x01);
	//writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1,  MPU9250_CLOCK_PLLGYROZ);
	delay(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03);   

	//MPU9250 Set Sensors
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_2, MPU9250_XYZ_GYRO & MPU9250_XYZ_ACCEL);
	delay(1);

	//MPU9250 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x04);
	//writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, SMPLRT_DIV);
	delay(1);

	//MPU9250 Set Full Scale Gyro Range
	//Fchoice_b[1:0] = [00] enable DLPF
	//writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, (MPU9250_FSR_2000DPS << 3));
	
	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG);
	//  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
	// writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0] 
	// writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	// writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
	// Clear Fchoice bit[1] and AFS bits[4:3]
	writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, (c & ~(0x02 | 0x18)) | Gscale << 3); // Set full scale range for the gyro
	// writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	
	delay(1);

	//MPU9250 Set Full Scale Accel Range PS:2G
	//writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, (MPU9250_FSR_2G << 3));
	
	// Set accelerometer full-scale range configuration
	c = readByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG);
	//  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
	// writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	// writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, (c & ~0x18) | Ascale << 3); // Set full scale range for the accelerometer 	
	delay(1);

	//MPU9250 Set Accel DLPF
	//data = readByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2);
	//data |= MPU9250_ACCEL_DLPF_41HZ;
	//delay(1);
	//writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, data);
	
	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2);
	// writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	// writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, (c & ~0x0F) | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	
	delay(1);

	//MPU9250 Set Gyro DLPF
	writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, MPU9250_GYRO_DLPF_41HZ);
	delay(1);


	//MPU9250 Set SPI Mode
	/*state = readByte(MPU9250_ADDRESS, MPU9250_USER_CTRL);
	delay(1);
	writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, state | MPU9250_I2C_IF_DIS);
	delay(1);
	state = readByte(MPU9250_ADDRESS, MPU9250_USER_CTRL);
	delay(1);
	writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, state | MPU9250_I2C_MST_EN);
	delay(1);

	
	//
	writeByte(MPU9250_ADDRESS, MPU9250_I2C_MST_CTRL, 0x5D);
	delay(1);
	writeByte(MPU9250_ADDRESS, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
	delay(1);
	writeByte(MPU9250_ADDRESS, MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
	delay(1);
	writeByte(MPU9250_ADDRESS, MPU9250_I2C_SLV0_CTRL, 0x88);
	delay(1);
	//
	
	//
	writeByte(MPU9250_ADDRESS, MPU9250_I2C_SLV4_CTRL, 0x09);
	delay(1);
	//
	writeByte(MPU9250_ADDRESS, MPU9250_I2C_MST_DELAY_CTRL, 0x81);
	delay(100);*/
	
	//MPU9250 Set Interrupt
	writeByte(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG,  0x22);
	//writeByte(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG,  MPU9250_INT_ANYRD_2CLEAR);
	delay(1);
	writeByte(MPU9250_ADDRESS, MPU9250_INT_ENABLE, ENABLE);
	delay(1);
	
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void cMPU9250::MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	uint8_t rawData[6] = {
	0, 0, 0, 0, 0, 0    };
	uint8_t selfTest[6];
	int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
	float factoryTrim[6];
	uint8_t FS = 0;

	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

	for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

		readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
	}

	for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay(25);  // Delay a while to let the device stabilize

	for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

		readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
	}

	for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}   

	// Configure the gyro and accelerometer for normal operation
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00);  
	writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0x00);  
	delay(25);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_GYRO);  // X-axis gyro self-test results
	selfTest[4] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
	selfTest[5] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
		destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
	}

}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void cMPU9250::calibrateMPU9250(float * dest1, float * dest2)
{  
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {
	0, 0, 0    }
	, accel_bias[3] = {
	0, 0, 0    };

	// reset device
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
	// else use the internal oscillator, bits 2:0 = 001
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);  
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_2, 0x00);
	delay(200);                                    

	// Configure device for bias calculation
	writeByte(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);      // Disable FIFO
	writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(MPU9250_ADDRESS, MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x40);   // Enable FIFO  
	writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(MPU9250_ADDRESS, MPU9250_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {
		  0, 0, 0        }
		, gyro_temp[3] = {
		  0, 0, 0        };
		  
		readBytes(MPU9250_ADDRESS, MPU9250_FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];

	}
	
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if(accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers
	writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_L, data[5]);

	// Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {
	0, 0, 0    }; // A place to hold the factory accelerometer trim biases
	
	readBytes(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {
	0, 0, 0    }; // Define array to hold mask bit for each accelerometer bias axis

	for(ii = 0; ii < 3; ii++) {
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	writeByte(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, MPU9250_XA_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, MPU9250_YA_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_L, data[5]);

	// Output scaled accelerometer biases for display in the main program
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


/*---------------------------------------------------------------------------
     TITLE   : initAK8963
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
 void cMPU9250::initAK8963()
 {
	//////////////////////////////////////////////////////////////////////////
	uint8_t response[3] = {0, 0, 0};
	
	//AK8963 Setup
	//reset AK8963
	writeAK8963Byte(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);
	delay(1);

	writeAK8963Byte(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	delay(1);
	writeAK8963Byte(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
	delay(1);

	readAK8963Bytes(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_WIA, 3, response);
	//
	//AK8963 get calibration data
	readAK8963Bytes(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
	
	AK8963_ASA[0] = (float)((response[0] - 128)/256.0f + 1.0f);
	AK8963_ASA[1] = (float)((response[1] - 128)/256.0f + 1.0f);
	AK8963_ASA[2] = (float)((response[2] - 128)/256.0f + 1.0f);
	
	/*AK8963_ASA[0] = (int16_t)(response[0]) + 128;
	AK8963_ASA[1] = (int16_t)(response[1]) + 128;
	AK8963_ASA[2] = (int16_t)(response[2]) + 128;*/
	
	delay(1);

	writeAK8963Byte(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	delay(1);	 

	writeAK8963Byte(MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
	delay(1);
		
 }

/*---------------------------------------------------------------------------
     TITLE   : gyro_init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::gyro_init( void )
{
	uint8_t i;


	for( i=0; i<3; i++ )
	{
		gyroADC[i]  = 0;
		gyroZero[i] = 0;
		gyroRAW[i]  = 0;
	}

	calibratingG = MPU_CALI_COUNT;
}



/*---------------------------------------------------------------------------
     TITLE   : gyro_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::gyro_get_adc( void )
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

	uint8_t rawADC[6];

	if( bConnected == true )
	{
		readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, rawADC );

		x = (((int16_t)rawADC[0]) << 8) | rawADC[1];
		y = (((int16_t)rawADC[2]) << 8) | rawADC[3];
		z = (((int16_t)rawADC[4]) << 8) | rawADC[5];

		gyroRAW[0] = x;
		gyroRAW[1] = y;
		gyroRAW[2] = z;

		GYRO_ORIENTATION( x, y, z );
	}

	gyro_common();
}





/*---------------------------------------------------------------------------
     TITLE   : gyro_cali_start
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::gyro_cali_start()
{
	calibratingG = MPU_CALI_COUNT;
}




/*---------------------------------------------------------------------------
     TITLE   : acc_init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::acc_init( void )
{
	uint8_t i;


	for( i=0; i<3; i++ )
	{
		accADC[i]   = 0;
		accZero[i]  = 0;
		accRAW[i]   = 0;
	}
}




/*---------------------------------------------------------------------------
     TITLE   : acc_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::acc_get_adc( void )
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

	uint8_t rawADC[6];



	if( bConnected == true )
	{
		readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, rawADC );

		x = (((int16_t)rawADC[0]) << 8) | rawADC[1];
		y = (((int16_t)rawADC[2]) << 8) | rawADC[3];
		z = (((int16_t)rawADC[4]) << 8) | rawADC[5];

		accRAW[0] = x;
		accRAW[1] = y;
		accRAW[2] = z;

		ACC_ORIENTATION( x,	y, z );
	}

	acc_common();
}





/*---------------------------------------------------------------------------
     TITLE   : gyro_common
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::gyro_common()
{
	static int16_t previousGyroADC[3] = {0,0,0};
	static int32_t g[3];
	uint8_t axis, tilt=0;

	if (calibratingG>0)
	{
		for (axis = 0; axis < 3; axis++)
		{
			if (calibratingG == MPU_CALI_COUNT)
			{ // Reset g[axis] at start of calibration
				g[axis]=0;
				previousGyroADC[axis] = gyroADC[axis];
			}
			if (calibratingG % 10 == 0)
			{
				//if(abs(gyroADC[axis] - previousGyroADC[axis]) > 8) tilt=1;

				previousGyroADC[axis] = gyroADC[axis];
			}
			g[axis] += gyroADC[axis]; // Sum up 512 readings
			gyroZero[axis]=g[axis]>>9;

			if (calibratingG == 1)
			{
				//SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_ELSE);
			}
		}

		if(tilt)
		{
			calibratingG=1000;
		}
		else
		{
			calibratingG--;
		}
		return;
	}


	for (axis = 0; axis < 3; axis++)
	{
		gyroADC[axis] -= gyroZero[axis];

		//anti gyro glitch, limit the variation between two consecutive readings
		//gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
		previousGyroADC[axis] = gyroADC[axis];
	}
}





/*---------------------------------------------------------------------------
     TITLE   : acc_common
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::acc_common()
{
	static int32_t a[3];

	if (calibratingA>0)
	{
		calibratingA--;
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			if (calibratingA ==(MPU_CALI_COUNT-1)) a[axis]=0;  // Reset a[axis] at start of calibration
			a[axis] += accADC[axis];             // Sum up 512 readings
			accZero[axis] = a[axis]>>9;          // Calculate average, only the last itteration where (calibratingA == 0) is relevant
		}
		if (calibratingA == 0)
		{
			//accZero[YAW] -= ACC_1G;
			accZero[YAW] = 0;
		}
	}

	accADC[ROLL]  -=  accZero[ROLL] ;
	accADC[PITCH] -=  accZero[PITCH];
	accADC[YAW]   -=  accZero[YAW] ;
}





/*---------------------------------------------------------------------------
     TITLE   : acc_init
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::mag_init( void )
{
	uint8_t i;

	for( i=0; i<3; i++ )
	{
		magADC[i]   = 0;
		magZero[i]  = 0;
		magRAW[i]   = 0;
	}
}




/*---------------------------------------------------------------------------
     TITLE   : mag_get_adc
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::mag_get_adc( void )
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

	uint8_t data[8];

	if(readByte(AK8963_ADDRESS, MPU9250_AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
		readBytes(AK8963_ADDRESS, MPU9250_AK8963_XOUT_L, 7, data);  // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = data[6]; // End data read by reading ST2 register
		
		if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			magRAW[0] = ((int16_t)data[1] << 8) | data[0] ;  // Turn the MSB and LSB into a signed 16-bit value
			magRAW[1] = ((int16_t)data[3] << 8) | data[2] ;  // Data stored as little Endian
			magRAW[2] = ((int16_t)data[5] << 8) | data[4] ; 
		}
	}
  
#if 0
	if( bConnected == true )
	{
		readBytes(MPU9250_ADDRESS, MPU9250_EXT_SENS_DATA_00, 8, data);

		if (!(data[0] & MPU9250_AK8963_DATA_READY) || (data[0] & MPU9250_AK8963_DATA_OVERRUN))
		{
			return;
		}
		if (data[7] & MPU9250_AK8963_OVERFLOW)
		{
			return;
		}
		magRAW[0] = (data[2] << 8) | data[1];
		magRAW[1] = (data[4] << 8) | data[3];
		magRAW[2] = (data[6] << 8) | data[5];

		magRAW[0] = ((long)magRAW[0] * AK8963_ASA[0]) >> 8;
		magRAW[1] = ((long)magRAW[1] * AK8963_ASA[1]) >> 8;
		magRAW[2] = ((long)magRAW[2] * AK8963_ASA[2]) >> 8;
	}
#endif

	mag_common();
}


/*---------------------------------------------------------------------------
     TITLE   : readTempData
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
int16_t cMPU9250::readTempData()
{
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, MPU9250_TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
	return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}


/*---------------------------------------------------------------------------
     TITLE   : mag_common
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::mag_common()
{
	magADC[0] = magRAW[0];
	magADC[1] = magRAW[1];
	magADC[2] = magRAW[2];
}



/*---------------------------------------------------------------------------
     TITLE   : acc_cali_start
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::acc_cali_start()
{
	calibratingA = MPU_CALI_COUNT;
}



/*---------------------------------------------------------------------------
     TITLE   : acc_cali_get_done
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU9250::acc_cali_get_done()
{
	if( calibratingA == 0 ) return true;
	else                    return false;
}




/*---------------------------------------------------------------------------
     TITLE   : gyro_cali_get_done
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
bool cMPU9250::gyro_cali_get_done()
{
	if( calibratingG == 0 ) return true;
	else                    return false;
}

void cMPU9250::getMres() {
	switch (Mscale)
	{
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
		case MFS_14BITS:
			mRes = 10.*4219./8190.; // Proper scale to return milliGauss
			break;
			
		case MFS_16BITS:
			mRes = 10.*4219./32760.0; // Proper scale to return milliGauss
			break;
	}
}

void cMPU9250::getGres() {
	switch (Gscale)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case GFS_250DPS:
			gRes = 250.0/32768.0;
			break;
			
		case GFS_500DPS:
			gRes = 500.0/32768.0;
			break;
			
		case GFS_1000DPS:
			gRes = 1000.0/32768.0;
			break;
			
		case GFS_2000DPS:
			gRes = 2000.0/32768.0;
			break;
	}
}

void cMPU9250::getAres() {
	switch (Ascale)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case AFS_2G:
			aRes = 2.0/32768.0;
			break;
			
		case AFS_4G:
			aRes = 4.0/32768.0;
			break;
			
		case AFS_8G:
			aRes = 8.0/32768.0;
			break;
			
		case AFS_16G:
			aRes = 16.0/32768.0;
			break;
	}
}

uint16_t cMPU9250::update( void)
{
	uint16_t ret_time = 0;

	static uint32_t tTime;

	if( (micros()-tTime) >= update_us )
	{
		ret_time = micros()-tTime;
		tTime = micros();

		computeIMU();

	}

	return ret_time;
}

void cMPU9250::computeIMU( void )
{
	static uint32_t prev_process_time = micros();
	static uint32_t cur_process_time = 0;
	static uint32_t process_time = 0;
	uint32_t i;
	static int32_t gyro_ADC[3][FILTER_NUM] = {0,};
	int32_t gyroAdcSum;

	uint32_t axis;

	acc_get_adc();
	gyro_get_adc();
	mag_get_adc();

	for (axis = 0; axis < 3; axis++)
	{
		gyro_ADC[axis][0] = gyroADC[axis];

		gyroAdcSum = 0;
		for (i=0; i<FILTER_NUM; i++)
		{
			gyroAdcSum += gyro_ADC[axis][i];
		}
		gyroADC[axis] = gyroAdcSum/FILTER_NUM;

		for (i=FILTER_NUM-1; i>0; i--)
		{
			gyro_ADC[axis][i] = gyro_ADC[axis][i-1];
		}

		if (abs(gyroADC[axis]) <= 3)
		{
			gyroADC[axis] = 0;
		}
	}


	for( i=0; i<3; i++ )
	{
		//accRaw[i]   = accRAW[i];
		accData[i]  = accADC[i];
		//gyroRaw[i]  = gyroRAW[i];
		gyroData[i] = gyroADC[i];
		//magRaw[i]   = magRAW[i];
		magData[i]  = magADC[i];
	}

	ax = (float)accADC[0]*aRes;
	ay = (float)accADC[1]*aRes;
	az = (float)accADC[2]*aRes;

	gx = (float)gyroADC[0]*gRes;
	gy = (float)gyroADC[1]*gRes;
	gz = (float)gyroADC[2]*gRes;

	mx = (float)magADC[0]*mRes*AK8963_ASA[0];
	my = (float)magADC[1]*mRes*AK8963_ASA[1];
	mz = (float)magADC[2]*mRes*AK8963_ASA[2];


	cur_process_time  = micros();
	process_time      = cur_process_time-prev_process_time;
	prev_process_time = cur_process_time;

	if (calibratingG == 0 && calibratingA == 0)
	{
		filter.invSampleFreq = (float)process_time/1000000.0f;
		//filter.updateIMU(gx, gy, gz, ax, ay, az);
		filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
	}

	rpy[0] = filter.getRoll();
	rpy[1] = filter.getPitch();
	rpy[2] = filter.getYaw()-180.;

	quat[0] = filter.q0;
	quat[1] = filter.q1;
	quat[2] = filter.q2;
	quat[3] = filter.q3;

	angle[0] = (int16_t)(rpy[0] * 10.);
	angle[1] = (int16_t)(rpy[1] * 10.);
	angle[2] = (int16_t)(rpy[1] * 1.);
	
	tempCount = readTempData();  // Read the adc values
    temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
}

/*---------------------------------------------------------------------------
     TITLE   : writeByte
     WORK    :
     ARG     : uint8_t address, uint8_t subAddress, uint8_t data
     RET     : void
---------------------------------------------------------------------------*/
void  cMPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
	
/*	_i2c->beginTransmission(address);  // Initialize the Tx buffer
	_i2c->write(subAddress);           // Put slave register address in Tx buffer
	_i2c->write(data);                 // Put data in Tx buffer
	i2c_err_ = _i2c->endTransmission();           // Send the Tx buffer
	if (i2c_err_) pirntI2CError();*/
	
}


/*---------------------------------------------------------------------------
     TITLE   : readByte
     WORK    :
     ARG     : uint8_t address, uint8_t subAddress
     RET     : uint8_t
---------------------------------------------------------------------------*/
uint8_t cMPU9250::readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data = 0; // `data` will store the register data
	
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(false);        	// Send the Tx buffer, but send a restart to keep connection alive
	//i2c_err_ = _i2c->endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
	//if (i2c_err_) pirntI2CError();
	Wire.requestFrom(address, (uint8_t)1);  // Read one byte from slave register address
	
	data = Wire.read();                      // Fill Rx buffer with result

/*
	_i2c->beginTransmission(address);         // Initialize the Tx buffer
	_i2c->write(subAddress);	                 // Put slave register address in Tx buffer
	_i2c->endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
	//i2c_err_ = _i2c->endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
	//if (i2c_err_) pirntI2CError();
	_i2c->requestFrom(address, (size_t)1);  // Read one byte from slave register address
	if (_i2c->available()) data = _i2c->read();                      // Fill Rx buffer with result
*/
	return data;                             // Return data read from slave register
}


/*---------------------------------------------------------------------------
     TITLE   : readByte
     WORK    :
     ARG     : uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest
     RET     : void
---------------------------------------------------------------------------*/
void cMPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
	Wire.requestFrom(address, count);  // Read bytes from slave register address
	while (Wire.available())
	{
		dest[i++] = Wire.read();
	} // Put read results in the Rx buffer

	
/*	_i2c->beginTransmission(address);   // Initialize the Tx buffer
	_i2c->write(subAddress);            // Put slave register address in Tx buffer
	i2c_err_ = _i2c->endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
	//i2c_err_ = _i2c->endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
	//if (i2c_err_) pirntI2CError();
	uint8_t i = 0;
	_i2c->requestFrom(address, count);  // Read bytes from slave register address
	while (_i2c->available())
	{
		dest[i++] = _i2c->read();
	} // Put read results in the Rx buffer*/
}
	
/*---------------------------------------------------------------------------
     TITLE   : writeAK8963Byte
     WORK    :
     ARG     : uint8_t address, uint8_t subAddress, uint8_t data
     RET     : uint8_t
---------------------------------------------------------------------------*/	
void cMPU9250::writeAK8963Byte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	// set slave 0 to the AK8963 and set for write
	writeByte(address, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR);
	
	// set the register to the desired AK8963 sub address 
	writeByte(address, MPU9250_I2C_SLV0_REG, subAddress);
	
	// store the data for write
	writeByte(address, MPU9250_I2C_SLV0_DO, data);
	
	// enable I2C and send 1 byte
	writeByte(address, MPU9250_I2C_SLV0_CTRL, MPU9250_I2C_SLV0_EN | (uint8_t)1);
}	

/*---------------------------------------------------------------------------
     TITLE   : readAK8963Bytes
     WORK    :
     ARG     : uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest
     RET     : void
---------------------------------------------------------------------------*/	
void cMPU9250::readAK8963Bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest)
{
	// set slave 0 to the AK8963 and set for read
	writeByte(address, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ_FLAG);
	
	// set the register to the desired AK8963 sub address
	writeByte(address, MPU9250_I2C_SLV0_REG, subAddress);
	
	// enable I2C and request the bytes
	writeByte(address, MPU9250_I2C_SLV0_CTRL, MPU9250_I2C_SLV0_EN | count);
	
	delay(1); // takes some time for these registers to fill
	
	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	readBytes(address, MPU9250_EXT_SENS_DATA_00, count, dest); 
}

void cMPU9250::pirntI2CError()
{
	if (i2c_err_ == 7) return; // to avoid stickbreaker-i2c branch's error code
	Serial.print("I2C ERROR CODE : ");
	Serial.println(i2c_err_);
}
