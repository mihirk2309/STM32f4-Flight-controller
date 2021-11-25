#ifndef _KALMAN_H_
#define _KALMAN_H_

#define COVAR 100
#define NOISE 0.01f

void kalman_init(float angle,float P,float Q,float R);
void kalman_angle(float Expected_angle, float Measured_angle);
float kalman_get_angle(void);

#endif	//_KALMAN_H_
