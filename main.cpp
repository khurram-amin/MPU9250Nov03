#include "MPU9250.h"
#include <iostream>
#include <cstdio>
#include <ctime>
#include <sys/time.h>

#include "quaternionEstimation.h"

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
	float* magBias = new float[3];
	float* magScale = new float[3];
	float* _quat = new float[4];

	wiringPiSetup () ;
	pinMode (0, INPUT) ;

	MPU9250 mpu9250;

	mpu9250.initMPU9250();
	delay(10);
	mpu9250.initAK8963(dataMB);
	delay(10);
	mpu9250.setMagClibration(dataMB);

	//mpu9250.calibrateMPU9250(gyroBias, accelBias);
	//mpu9250.calibrateAK8963Mag(magBias, magScale);
	magBias[0] = 0; magBias[1] = 0; magBias[2] = 0;
	magScale[0] = 1; magScale[1] = 1; magScale[2] = 1;

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
	
	struct timeval before, after, qTime;
	gettimeofday(&before,NULL);
	double beforeDouble, afterDouble, qTimeDouble;
	double counter = 0;
	double freq = 0;

	qEstimator toQuaternion;
	toQuaternion.updateDeltaT((float)0);

	beforeDouble = (double) (before.tv_sec+before.tv_usec/1000000);
	qTimeDouble = (double) (1000000*qTime.tv_sec+qTime.tv_usec);

	while(1)
	{
		if( digitalRead(0) )
		{
			gettimeofday(&after,NULL);
			counter++;

			dataT = 0;
			mpu9250.readTempRawData(&dataT);

			for (int i=0; i<3; i++){ dataA[i] = 0; }
			mpu9250.readAccelroRawData(dataA);

			for (int i=0; i<3; i++){ dataG[i] = 0; }
			mpu9250.readGyroRawData(dataG);

			for (int i=0; i<3; i++){ dataM[i] = 0; }
			mpu9250.readMagnetoRawData(dataM);


			// //sprintf("Temp = %d\nAccelX = %d\nAccelY = %d\nAccelZ = %d\nGyroX = %d\nGyroY = %d\nGyroZ = %d\nMagntX = %d\nMagntY = %d\nMagntZ = %d\n\n\n", 1.0f*dataT, 1.0f*dataA[0]*mpu9250.getAccelroResolution, 1.0f*dataA[1]*mpu9250.getAccelroResolution, 1.0f*dataA[2]*mpu9250.getAccelroResolution, 1.0f*dataG[0]*mpu9250.getGyroResolution, 1.0f*dataG[1]*mpu9250.getGyroResolution, 1.0f*dataG[2]*mpu9250.getGyroResolution, 1.0f*dataM[0]*mpu9250.getMagnetoResolution, 1.0f*dataM[1]*mpu9250.getMagnetoResolution, 1.0f*dataM[2]*mpu9250.getMagnetoResolution);
			// cout<< "Temp = " << 1.0f*dataT/mpu9250.getTempResolution() + 21 << endl;
			// cout<< "AccelX = " << 1.0f*dataA[0]*mpu9250.getAccelroResolution() << endl;
			// cout<< "AccelY = " << 1.0f*dataA[1]*mpu9250.getAccelroResolution() << endl;
			// cout<< "AccelZ = " << 1.0f*dataA[2]*mpu9250.getAccelroResolution() << endl;
			// cout<< "GyroX = " << 1.0f*dataG[0]*mpu9250.getGyroResolution() << endl;
			// cout<< "GyroY = " << 1.0f*dataG[1]*mpu9250.getGyroResolution() << endl;
			// cout<< "GyroZ = " << 1.0f*dataG[2]*mpu9250.getGyroResolution() << endl;
			// cout<< "MagntX = " << (1.0f*dataM[0]*mpu9250.getMagnetoResolution() - 1.0f*magBias[0])/(1.0f*magScale[0]) << endl;
			// cout<< "MagntY = " << (1.0f*dataM[1]*mpu9250.getMagnetoResolution() - 1.0f*magBias[1])/(1.0f*magScale[1])  << endl;
			// cout<< "MagntZ = " << (1.0f*dataM[2]*mpu9250.getMagnetoResolution() - 1.0f*magBias[2])/(1.0f*magScale[2])  << endl;
			


			mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS);
			
			afterDouble = (double) (after.tv_sec+after.tv_usec/1000000);
			freq = (double) ( (double)counter/(double)(afterDouble-beforeDouble) );

			qTimeDouble = (double)(1000000*after.tv_sec+after.tv_usec) - (double)(1000000*qTime.tv_sec+qTime.tv_usec);

			toQuaternion.updateDeltaT((float)(qTimeDouble/1000000));
			toQuaternion.MadgwickUpdate(1.0f*dataA[0]*mpu9250.getAccelroResolution(), 1.0f*dataA[1]*mpu9250.getAccelroResolution(), 1.0f*dataA[2]*mpu9250.getAccelroResolution(), 1.0f*dataG[0]*mpu9250.getGyroResolution(), 1.0f*dataG[1]*mpu9250.getGyroResolution(), 1.0f*dataG[2]*mpu9250.getGyroResolution(), (1.0f*dataM[0]*mpu9250.getMagnetoResolution() - 1.0f*magBias[0])/(1.0f*magScale[0]), (1.0f*dataM[1]*mpu9250.getMagnetoResolution() - 1.0f*magBias[1])/(1.0f*magScale[1]), (1.0f*dataM[2]*mpu9250.getMagnetoResolution() - 1.0f*magBias[2])/(1.0f*magScale[2]));

			for (int i=0; i<4; i++){ _quat[i] = 0; }
			toQuaternion.getQuaternion(_quat);
			cout << "Q1: " << 1.0f*_quat[0] << endl;
			cout << "Q2: " << 1.0f*_quat[1] << endl;
			cout << "Q3: " << 1.0f*_quat[2] << endl;
			cout << "Q4: " << 1.0f*_quat[3] << endl;

			cout << "Current sample rate is " << (double) freq << " Hz" <<endl;
			cout<<endl<<endl<<endl;

			gettimeofday(&qTime,NULL);
		}
		if (afterDouble-beforeDouble > 10)
		{
			gettimeofday(&before,NULL);
			beforeDouble = (double) (before.tv_sec+before.tv_usec/1000000);
			counter = 0;
		}
		//delay(1);
	}
	return 1;
}