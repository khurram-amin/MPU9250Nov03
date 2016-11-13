#include "MPU9250.h"
#include <iostream>
#include <cstdio>

int main()
{
	short* dataT;
	short* dataA = new short[3];
	short* dataG = new short[3];
	short* dataM = new short[3];
	float* dataMB = new float[3];

	MPU9250 mpu9250;
	mpu9250.initMPU9250();
	delay(10);
	mpu9250.initAK8963(dataMB);
	delay(10);
	

	while(1)
	{
		dataT = 0;
		mpu9250.readTempRawData(dataT);

		for (int i=0; i<3; i++){ dataA[i] = 0; }
		mpu9250.readAccelroRawData(dataA);

		for (int i=0; i<3; i++){ dataG[i] = 0; }
		mpu9250.readGyroRawData(dataG);

		for (int i=0; i<3; i++){ dataM[i] = 0; }
		mpu9250.readMagnetoRawData(dataM);


		sprintf("Temp = %d\nAccelX = %d\nAccelY = %d\nAccelZ = %d\nGyroX = %d\nGyroY = %d\nGyroZ = %d\nMagntX = %d\nMagntY = %d\nMagntZ = %d\n\n\n", 1.0f*dataT, 1.0f*dataA[0]*mpu9250.getAccelroResolution, 1.0f*dataA[1]*mpu9250.getAccelroResolution, 1.0f*dataA[2]*mpu9250.getAccelroResolution, 1.0f*dataG[0]*mpu9250.getGyroResolution, 1.0f*dataG[1]*mpu9250.getGyroResolution, 1.0f*dataG[2]*mpu9250.getGyroResolution, 1.0f*dataM[0]*mpu9250.getMagnetoResolution, 1.0f*dataM[1]*mpu9250.getMagnetoResolution, 1.0f*dataM[2]*mpu9250.getMagnetoResolution);

		delay(10);
	}
}