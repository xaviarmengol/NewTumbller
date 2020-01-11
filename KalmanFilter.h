#ifndef KalmanFilter_h
#define KalmanFilter_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

class KalmanFilter
{
public:
  void Yiorderfilter(float angle_m, float gyro_m,float dt,float K1);
  void Kalman_Filter(double angle_m, double gyro_m,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0);
  void Angle(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1);
  void setGyroOffset(float gxOff, float gyOff, float gzOff);
  float Gyro_x, Gyro_y, Gyro_z;
  float gx, gy, gz; // Raw values with Offset
  float accelz = 0;
  float angle;
  float angle6;
private:
  float angle_err, q_bias;
  float Pdot[4] = { 0, 0, 0, 0};
  float P[2][2] = {{ 1, 0 }, { 0, 1 }};
  float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
  float angle_dot;

  float gxOffset=0.0;
  float gyOffset=0.0;
  float gzOffset=0.0;


   //gx: -400 gy: -56 gz: -218
};

//Band pass chebyshev filter order=1 alpha1=0.057 alpha2=0.061 
class  FilterChBp1
{
	private:
		double v[3];

	public:
		FilterChBp1();
		double step(double x);

};

#endif