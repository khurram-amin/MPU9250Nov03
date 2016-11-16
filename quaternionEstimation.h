#include <cmath>

class qEstimator{
	private:
		float PI;
		float GyroMeasError; // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
		float beta;			 // compute beta
		float GyroMeasDrift; // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
		float zeta;		     // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
		#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
		#define Ki 0.0f

		float deltat;

		float q[4];           // vector to hold quaternion

	public:
		void MadgwickUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
		void updateDeltaT(float _delta){
			deltat = _delta;
		}
};