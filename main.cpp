#include "MPU9250.h"
#include <iostream>
#include <cstdio>
#include <ctime>

using namespace std;

int main()
{
	int16_t dataT;
	short* dataA = new short[3];
	short* dataG = new short[3];
	short* dataM = new short[3];
	float* dataMB = new float[3];
	float* selftest_destination = new float[6];
	float* gyroBias = new float[3];
	float* accelBias = new float[3];

	wiringPiSetup () ;
	pinMode (0, INPUT) ;

	MPU9250 mpu9250;

	// mpu9250.calibrateMPU9250(gyroBias, accelBias);
	// cout<< "Self Calibration results " << endl;
	// cout << "AccelX = " << 1.0f*accelBias[0] << endl;
	// cout << "AccelY = " << 1.0f*accelBias[1] << endl;
	// cout << "AccelZ = " << 1.0f*accelBias[2] << endl;
	// cout << "GyroX = " << 1.0f*gyroBias[0] << endl;
	// cout << "GyroY = " << 1.0f*gyroBias[1] << endl;
	// cout << "GyroZ = " << 1.0f*gyroBias[2] << endl;

	// for (int i=0; i<3; i++){ selftest_destination[i] = 0; }
	// mpu9250.MPU9250SelfTest(selftest_destination);
	// cout<< "Self testing results " << endl;
	// cout << "AccelX = " << 1.0f*selftest_destination[0] << endl;
	// cout << "AccelY = " << 1.0f*selftest_destination[1] << endl;
	// cout << "AccelZ = " << 1.0f*selftest_destination[2] << endl;
	// cout << "GyroX = " << 1.0f*selftest_destination[3] << endl;
	// cout << "GyroY = " << 1.0f*selftest_destination[4] << endl;
	// cout << "GyroZ = " << 1.0f*selftest_destination[5] << endl;


	mpu9250.initMPU9250();
	delay(10);
	mpu9250.initAK8963(dataMB);
	delay(10);
	
	time_t before = time(NULL);
	time_t after;
	int counter = 0;
	
	while(1)
	{
		if( digitalRead(0) )
		{
			dataT = 0;
			mpu9250.readTempRawData(&dataT);

			for (int i=0; i<3; i++){ dataA[i] = 0; }
			mpu9250.readAccelroRawData(dataA);

			for (int i=0; i<3; i++){ dataG[i] = 0; }
			mpu9250.readGyroRawData(dataG);

			for (int i=0; i<3; i++){ dataM[i] = 0; }
			mpu9250.readMagnetoRawData(dataM);


			//sprintf("Temp = %d\nAccelX = %d\nAccelY = %d\nAccelZ = %d\nGyroX = %d\nGyroY = %d\nGyroZ = %d\nMagntX = %d\nMagntY = %d\nMagntZ = %d\n\n\n", 1.0f*dataT, 1.0f*dataA[0]*mpu9250.getAccelroResolution, 1.0f*dataA[1]*mpu9250.getAccelroResolution, 1.0f*dataA[2]*mpu9250.getAccelroResolution, 1.0f*dataG[0]*mpu9250.getGyroResolution, 1.0f*dataG[1]*mpu9250.getGyroResolution, 1.0f*dataG[2]*mpu9250.getGyroResolution, 1.0f*dataM[0]*mpu9250.getMagnetoResolution, 1.0f*dataM[1]*mpu9250.getMagnetoResolution, 1.0f*dataM[2]*mpu9250.getMagnetoResolution);
			cout<< "Temp = " << 1.0f*dataT/mpu9250.getTempResolution() + 21 << endl;
			cout<< "AccelX = " << 1.0f*dataA[0]*mpu9250.getAccelroResolution() << endl;
			cout<< "AccelY = " << 1.0f*dataA[1]*mpu9250.getAccelroResolution() << endl;
			cout<< "AccelZ = " << 1.0f*dataA[2]*mpu9250.getAccelroResolution() << endl;
			cout<< "GyroX = " << 1.0f*dataG[0]*mpu9250.getGyroResolution() << endl;
			cout<< "GyroY = " << 1.0f*dataG[1]*mpu9250.getGyroResolution() << endl;
			cout<< "GyroZ = " << 1.0f*dataG[2]*mpu9250.getGyroResolution() << endl;
			cout<< "MagntX = " << 1.0f*dataM[0]*mpu9250.getMagnetoResolution() << endl;
			cout<< "MagntY = " << 1.0f*dataM[1]*mpu9250.getMagnetoResolution() << endl;
			cout<< "MagntZ = " << 1.0f*dataM[2]*mpu9250.getMagnetoResolution() << endl;
			
			mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS);
			after = time(NULL);
			counter++;
			cout << "Current sample rate is " << counter/(after-before) << "Hz" <<endl;
			cout<<endl<<endl<<endl;
		}
		cout<<"here"<<endl;
		delay(1);
	}
	return 1;
}