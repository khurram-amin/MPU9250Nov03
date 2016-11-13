#include "MPU9250.h"

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
	
	
	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	writeByte(MPU9250_ADDRESS, CONFIG, 0x03);
	
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
	

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
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


