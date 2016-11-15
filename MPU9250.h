#ifndef MPU9250_H
#define MPU9250_H
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include "MPU9250RegMap.h"

#define uint8_t unsigned char
#define int8_t char
#define uint16_t unsigned short
#define int16_t short
#define uint32_t unsigned long
#define int32_t long

class MPU9250{

	private:
		// Set initial input parameters
	    enum acceleroSensitivity { AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G };
	    enum gyroSensitivity { GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS };
	    enum magnetoSensitivity { MFS_14BITS = 0, MFS_16BITS };

	    uint8_t acceleroSensitivity, gyroSensitivity, magnetoSensitivity;
		float magnetoResolution, accelroResolution, gyroResolution, tempResolution;
		uint8_t magnetoMode;

		// Variables to store file handlers of open I2C device 
		int fdMPU9250, fdAK8963;


	public:
		MPU9250();

		// These functions will set the resolution per sensor tick variables. These variables will be used to convert the raw sensor value into physical units by incorporating the sensitivity mode of the sensor.
		// Magnetometer resolution. Will set magnetoResolution varaible. (milli Gauss)
		void setMagnetoResolution();
		// Accelerometer resolution. Will set accelroResolution variable. (meter per second squared)
		void setAccelroResolution();
		// Gyroscope resolution. Will set gyroResolution variable. (degrees per second)
		void setGyroResolution();
		// Set Temperature sensor resolution
		void setTempResolution();

		// Get current sensor resolution.
		// Get Magnetometer resolution (milli-Gauss per tick)
		float getMagnetoResolution(){
			return magnetoResolution;
		}
		// Get Accelerometer resolution (meter-per-second-squared per tick)
		float getAccelroResolution(){
			return accelroResolution;
		}
		// Get Gyroscope resolution (Degrees per tick)
		float getGyroResolution(){
			return gyroResolution;
		}
		// Get Temperature Sensor resolution
		float getTempResolution(){
			return tempResolution;
		}

		// Read raw sensor data
		void readMagnetoRawData(int16_t* destination);
		void readAccelroRawData(int16_t* destination);
		void readGyroRawData(int16_t* destination);
		void readTempRawData(int16_t* destination);

		// Initialize and configure sensors
		void initMPU9250();
		void initAK8963(float* destination);

		// IMU Self-testing
		void MPU9250SelfTest(float * destination);

		// IMU Self-Calibration
		void calibrateMPU9250(float * gyroBias, float * accelBias);

		// Register read and write over I2C routines
		char readByte(uint8_t address, uint8_t subAddress);
		void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* destination);
		void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
};

#endif