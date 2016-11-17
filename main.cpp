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
	double* _degs = new double[3];

	wiringPiSetup () ;
	pinMode (0, INPUT) ;

	MPU9250 mpu9250;

	mpu9250.initMPU9250();
	delay(10);
	mpu9250.initAK8963(dataMB);
	delay(10);
	mpu9250.setMagClibration(dataMB);

	cout << "WHO AM I AK8963 IN MAIN LOOP " << endl;
	mpu9250.whoAmIAK8963();
	mpu9250.calibrateMPU9250(gyroBias, accelBias);
	mpu9250.initAK8963(dataMB);
	//mpu9250.calibrateAK8963Mag(magBias, magScale);
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// uint16_t ii = 0, sample_count = 0;
	// int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};

	// int mag_max[3];
	// mag_max[0] = (int)-32767; mag_max[1] = (int)-32767; mag_max[2] = (int)-32767;
	// int mag_min[3];
	// mag_min[0] = (int)32768; mag_min[1] = (int)32768; mag_min[2] = (int)32768;
	
	// short* mag_temp = new short[3];
	// mag_temp[0] = 1; mag_temp[1] = 1; mag_temp[2] = 1;

	// std::cout << "Mag Calibration: Wave device in a figure eight until done!" << std::endl;
	// delay(2000);

	// sample_count = 500;
	// for(ii = 0; ii < sample_count; ii++)
	// {
	// 	while()
	// 	mpu9250.readMagnetoRawData(mag_temp);  // Read the mag data
	// 	cout << "mag readings in short" << endl;
	// 	cout << 1.0f*mag_temp[0] << endl;
	// 	cout << 1.0f*mag_temp[1] << endl;
	// 	cout << 1.0f*mag_temp[2] << endl;   
	// 	for (int jj = 0; jj < 3; jj++)
	// 	{
	// 		if(mag_temp[jj] > (int)mag_max[jj]) mag_max[jj] = (int)mag_temp[jj];
	// 		if(mag_temp[jj] < (int)mag_min[jj]) mag_min[jj] = (int)mag_temp[jj];
	// 		if(isnan(mag_temp[jj])){
	// 			cout << "NAN" <<endl;
	// 		}
	// 	}
	// }

	// // Get hard iron correction
	// mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	// mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	// mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	// //float * magBias = new float[3];
	// //float * dest2 = new float[3];

	// magBias[0] = (float) mag_bias[0]*mpu9250.magnetoResolution*mpu9250.magCalibration[0];  // save mag biases in G for main program
	// magBias[1] = (float) mag_bias[1]*mpu9250.magnetoResolution*mpu9250.magCalibration[1];   
	// magBias[2] = (float) mag_bias[2]*mpu9250.magnetoResolution*mpu9250.magCalibration[2];  

	// // Get soft iron correction estimate
	// mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	// mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	// mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	// float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	// avg_rad /= 3.0;

	// magScale[0] = avg_rad/((float)mag_scale[0]);
	// magScale[1] = avg_rad/((float)mag_scale[1]);
	// magScale[2] = avg_rad/((float)mag_scale[2]);

	// cout << (int) mag_max[0] << endl;
	// cout << (int) mag_min[0] << endl;

	// if (isnan(mag_bias[0])) { cout << "MAG_BIAS[0] is NAN" << endl; }
	// if (isnan(mag_bias[1])) { cout << "MAG_BIAS[1] is NAN" << endl; }
	// if (isnan(mag_bias[2])) { cout << "MAG_BIAS[2] is NAN" << endl; }
	// if (isnan(mag_scale[0])) { cout << "MAG_SCALE[0] is NAN" << endl; }
	// if (isnan(mag_scale[1])) { cout << "MAG_SCALE[1] is NAN" << endl; }
	// if (isnan(mag_scale[2])) { cout << "MAG_SCALE[2] is NAN" << endl; }

	// std::cout << "Mag Calibration done!" << std::endl;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//magBias[0] = 0; magBias[1] = 0; magBias[2] = 0;
	//magScale[0] = 1; magScale[1] = 1; magScale[2] = 1;
	magBias[0] = -406.09; magBias[1] = 3.46; mag_bias[2] = -121.55;
	magScale[0] = 0.96; magScale[1] = 1.01; magScale[2] = 1.03;

	cout<< "Self Calibration results " << endl;
	cout << "AccelX = " << 1.0f*accelBias[0] << endl;
	cout << "AccelY = " << 1.0f*accelBias[1] << endl;
	cout << "AccelZ = " << 1.0f*accelBias[2] << endl;
	cout << "GyroX = " << 1.0f*gyroBias[0] << endl;
	cout << "GyroY = " << 1.0f*gyroBias[1] << endl;
	cout << "GyroZ = " << 1.0f*gyroBias[2] << endl;
	cout << "MagBiasx = " << 1.0f*magBias[0] << endl;
	cout << "MagBiasY = " << 1.0f*magBias[1] << endl;
	cout << "MagBiasZ = " << 1.0f*magBias[2] << endl;
	cout << "MagScaleX = " << 1.0f*magScale[0] << endl;
	cout << "MagScaleY = " << 1.0f*magScale[1] << endl;
	cout << "MagScaleZ = " << 1.0f*magScale[2] << endl;

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

	gettimeofday(&qTime,NULL);
	qTimeDouble = (double) ((double)1000*qTime.tv_sec+(double)qTime.tv_usec/1000);

	double roll = 0, pitch = 0, yaw = 0;

	float PI = 3.141592653589793;
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

			qTimeDouble = (double)( (double)1000*after.tv_sec+(double)after.tv_usec/1000 ) - (double)( (double)1000*qTime.tv_sec+(double)qTime.tv_usec/1000 );

			for (int i=0; i<4; i++){ _quat[i] = 0; } _quat[0] = 1;
			//toQuaternion.setQuaternion(_quat);

			toQuaternion.updateDeltaT((float)(qTimeDouble/1000));
			toQuaternion.MadgwickUpdate(1.0f*dataA[0]*mpu9250.getAccelroResolution(), 1.0f*dataA[1]*mpu9250.getAccelroResolution(), 1.0f*dataA[2]*mpu9250.getAccelroResolution(), 1.0f*dataG[0]*mpu9250.getGyroResolution()*(PI/180), 1.0f*dataG[1]*mpu9250.getGyroResolution()*(PI/180), 1.0f*dataG[2]*mpu9250.getGyroResolution()*(PI/180), (1.0f*dataM[0]*mpu9250.getMagnetoResolution()*magScale[0] - 1.0f*magBias[0]), (1.0f*dataM[1]*mpu9250.getMagnetoResolution()*magScale[1] - 1.0f*magBias[1]), (1.0f*dataM[2]*mpu9250.getMagnetoResolution()*magScale[2] - 1.0f*magBias[2]) );

			
			toQuaternion.getQuaternion(_quat);
			toQuaternion.toEulerAngle(_degs);
			// cout << "delata " << (float)(qTimeDouble/1000) << endl;
			cout << "Q1: " << 1.0f*_quat[0] << endl;
			cout << "Q2: " << 1.0f*_quat[1] << endl;
			cout << "Q3: " << 1.0f*_quat[2] << endl;
			cout << "Q4: " << 1.0f*_quat[3] << endl;
			roll  = _degs[0];
			pitch = _degs[1];
			yaw   = _degs[2] - 3.36;

			cout << "deltaT: " << (float)qTimeDouble << endl;
			cout << "Roll: " << (double)roll << endl;
			cout << "Pitch: " << (double)pitch << endl;
			cout << "Yaw: " << (double)yaw << endl;

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