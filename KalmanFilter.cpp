#include "KalmanFilter.h"


//No es fa anar TODO:Que es?
void KalmanFilter::Yiorderfilter(float angle_m, float gyro_m,float dt,float K1) {
  angle6 = K1 * angle_m + (1 - K1) * (angle6 + gyro_m * dt);
}

//https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

void KalmanFilter::Kalman_Filter(double angle_m, double gyro_m,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0) {
  angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err;
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias;
}

void KalmanFilter::Angle(int16_t ax, int16_t ay, int16_t az, int16_t gxInp, int16_t gyInp, int16_t gzInp, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1) {

  gx = gxInp - gxOffset;
  gy = gyInp - gyOffset;
  gz = gzInp -  gzOffset;
  
  float Angle = atan2(ay , az) * 57.3;
  Gyro_x = (gx) / 131; // -128.1 originalment? 
  Kalman_Filter(Angle, Gyro_x, dt, Q_angle, Q_gyro, R_angle, C_0);
  Gyro_z = -gz / 131;

  Gyro_y = gy / 131; // Todo mirar si es correcte
}


void KalmanFilter::setGyroOffset(float gxOff, float gyOff, float gzOff){

  gxOffset=gxOff;
  gyOffset=gyOff;
  gzOffset=gzOff;
  
}

//Band pass chebyshev filter order=1 alpha1=0.057 alpha2=0.061 

FilterChBp1::FilterChBp1() {
  v[0] = 0.0;
  v[1] = 0.0;
}

double FilterChBp1::step(double x) {
  v[0] = v[1];
  v[1] = v[2];
  v[2] = (1.244181729691522069e-2 * x)
      + (-0.97511960080873705259 * v[0])
      + (1.84109728886611145882 * v[1]);
  return (v[2] - v[0]);
}


