/***************************************************
 ZE07 CO Sensor
  
 ***************************************************
 This example reads the concentration of CO in air by UART mode.
 
 Created 2018-01-03
  
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/
 
 /***********Notice and Trouble shooting***************
 1. This code is tested on Arduino Uno with Arduino IDE 1.0.5 r2.
 2. In order to protect the sensor, do not touch the white sensor film on the sensor module, 
 and high concentration of Hydrogen sulfide, hydrogen, methanol, ethanol should be avoided. 
 3. Please do not use the modules in systems which related to human being’s safety.
 ****************************************************/

#include <ZE07CO_Sensor.h>
#include <SoftwareSerial.h>

#define SensorSerialPin	10	//this pin read the uart signal from the HCHO sensor

HardwareSerial* CO_Serial = &Serial3;
float VO = 0.0;
float ppm = 0.0;
ZE07CO_Sensor CO_Sensor(CO_Serial);

void setup()
{  
    CO_Serial->begin(9600);	//the baudrate of HCHO is 9600
    pinMode(A0,INPUT);
    Serial.begin(9600);
}

void loop()
{
    if(CO_Sensor.available(2000)>0)	
    {
      Serial.print(CO_Sensor.uartReadPPM());
      Serial.println("ppm");
      VO = (float)analogRead(A0)*5/1024.0;
      Serial.println(VO);
      ppm = (VO-0.41)*500/1.6;
      Serial.print(ppm);
      Serial.println("ppm");
    }
}
