#include "MPU9250.h"
#include <cmath>

MPU9250::MPU9250()
{
	 fdAK8963 = wiringPiI2CSetup(AK8963_ADDRESS );
	fdMPU9250 = wiringPiI2CSetup(MPU9250_ADDRESS);
	acceleroSensitivity = AFS_2G;
	gyroSensitivity = GFS_250DPS;
	magnetoSensitivity = MFS_16BITS;
	magnetoMode = 0x02;
}

// Based on the current sensitivity configuration of magnetometer, this function will set the resolution per bit of magnetometer.
void MPU9250::setMagnetoResolution()
{
	switch (magnetoSensitivity)
	{
	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          magnetoResolution = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          magnetoResolution = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
	}
}

// Based on the current sensitivity configuration of Accelerometer, this function will set the resolution per bit of Accelerometer.
void MPU9250::setAccelroResolution()
{
	switch (acceleroSensitivity)
	{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
	case AFS_2G:
          accelroResolution = 2.0/32768.0;
          break;
    case AFS_4G:
          accelroResolution = 4.0/32768.0;
          break;
    case AFS_8G:
          accelroResolution = 8.0/32768.0;
          break;
    case AFS_16G:
          accelroResolution = 16.0/32768.0;
          break;
	}
}

// Based on the current sensitivity configuration of Gyroscope, this function will set the resolution per bit of Gyroscope.
void MPU9250::setGyroResolution()
{
	switch(gyroSensitivity)
	{
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GFS_250DPS:
          gyroResolution = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gyroResolution = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gyroResolution = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gyroResolution = 2000.0/32768.0;
          break;
	}
}

// Read three 16-bit registers corressponding to Raw Magnetometer data
void MPU9250::readMagnetoRawData(int16_t* destination)
{
	// x/y/z gyro register data, ST2 register stored here, must read ST2 at end of
	// data acquisition
	uint8_t rawData[7];
	// Wait for magnetometer data ready bit to be set
	if( readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01 )
	{
		// Read the six raw data and ST2 registers sequentially into data array
		readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		// Check if magnetic sensor overflow set, if not then report data
		if( !(c & 0x08) )
		{
			// Turn the MSB and LSB into a signed 16-bit value
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0]; // Data stored as little Endian 
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
    	}
  	}
}

// Read three 16-bit registers corressponding to Raw Accelerometer data
void MPU9250::readAccelroRawData(int16_t* destination)
{
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

// Read three 16-bit registers corressponding to Raw Gyroscope data
void MPU9250::readGyroRawData(int16_t* destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

// Read one 16-bit registers corressponding to Raw Temperature data
void MPU9250::readTempRawData(int16_t* destination)
{
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a 16-bit value
}


// Configure MPU9250 (Accelero+Gyro)
void MPU9250::initMPU9250()
{
	// wake up device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
	delay(50); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  
	// get stable time source
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	
	//Disable Gyro for update rate testing
	writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
	
	// CONFIG = 0x00
	// DLPF_CFG = bits 2:0 = 000; this sets the sample rate at 8 kHz for both
	// Maximum delay is 0.97 ms which is just over a 1030 Hz maximum rate
	
	// CONFIG = 0x07
	// DLPF_CFG = bits 2:0 = 111; this sets the sample rate at 8 kHz for both
	// Maximum delay is 0.17 ms which is just over a 5882 Hz maximum rate

	// CONFIG = 0x03
	// DLPF_CFG = bits 2:0 = 011; this sets the sample rate at 1 kHz for both
	// Maximum delay is 5.9 ms which is just over a 169 Hz maximum rate
	writeByte(MPU9250_ADDRESS, CONFIG, 0x03);
	
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
	

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // Read GYRO_CONFIG register
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | gyroSensitivity << 3); // Set full scale range for the gyro

	// Set accelerometer configuration
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | acceleroSensitivity << 3); // Set full scale range for the accelerometer 

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
	
	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);
	
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

}

// Configure AK8963 (Magnetometer)
void MPU9250::initAK8963(float* destination)
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
	delay(1);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	delay(1);
	readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	destination[0] = (float)(rawData[0] - 128) / 256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] = (float)(rawData[1] - 128) / 256.0f + 1.0f;
	destination[2] = (float)(rawData[2] - 128) / 256.0f + 1.0f;
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
	delay(1);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeByte(AK8963_ADDRESS, AK8963_CNTL, magnetoSensitivity << 4 | magnetoMode); // Set magnetometer data resolution and sample ODR
	delay(1);
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
	float factoryTrim[6];
	uint8_t FS = 0;

	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

	for( int ii = 0; ii < 200; ii++)
	{  
		// get average current values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

		readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
	}

	for (int ii =0; ii < 3; ii++)
	{
		// Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}
  
	// Configure the accelerometer for self-test
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay(25);  // Delay a while to let the device stabilize

	for( int ii = 0; ii < 200; ii++)
	{  
		// get average self-test values of gyro and acclerometer
		readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

		readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
	}

	for (int ii =0; ii < 3; ii++)
	{
		// Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}   

	// Configure the gyro and accelerometer for normal operation
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
	delay(25);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
	selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
	selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++)
	{
		destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
		destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
	}
}



// Read 1 byte register(subAddress) of given device(address)
char MPU9250::readByte(uint8_t address, uint8_t subAddress)
{
	switch(address)
	{
		case MPU9250_ADDRESS:
			return wiringPiI2CReadReg8(fdMPU9250, subAddress);
		case AK8963_ADDRESS:
			return wiringPiI2CReadReg8(fdAK8963, subAddress);
	} 
}

// Read muliple(count) bytes starting from register(subAddress) of given device(address)
void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* destination)
{
	char data;
	for (char i = 0; i < count; i++)
	{
		data = readByte(address, subAddress + i);
		destination[i] = data;
	}
}

// Write 1 byte register(subAddress) of given device(address)
void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	switch(address)
	{
		case MPU9250_ADDRESS:
			wiringPiI2CWriteReg8(fdMPU9250, subAddress, data);
			break;
		case AK8963_ADDRESS:
			wiringPiI2CWriteReg8(fdAK8963, subAddress, data);
			break;

	} 
}


