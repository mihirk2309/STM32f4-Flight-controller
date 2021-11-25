#include "kalman.h"

//Hidden Structure For Kalman
struct{
	float angle;
	float P;
	float Q;
	float R;
	float K;
}kalman;

/*
@Brief: Initializes the kalman parameters
@Param: angle is the initial angle
@Param: P is the predicted error(set initial value)
@Param: Q is the effect of environmental noise in predicted error
@Param: R is the expected error (given in the sensor datasheet)
@Retval: None
*/
void kalman_init(float angle,float P,float Q,float R)
{
	kalman.angle = angle;
	kalman.P = P;
	kalman.Q = Q;
	kalman.R = R;
}

/*
@Brief: This is the predict and update algorithm for kalman computation
@Param: exp_angle is the expected angle
@Param: meas_angle is the gyroscope measured angle
@Param: dt is the time difference between consecutive calls of this function
*/
void kalman_angle(float exp_angle,float meas_angle)
{
	kalman.P = kalman.P + kalman.Q;
	kalman.K = kalman.P /(kalman.P + kalman.R);
	kalman.angle = exp_angle + kalman.K * (meas_angle - exp_angle);
	kalman.P = (1 - kalman.K)* kalman.P;
}

/*
@Brief: Returns the best estimate angle
@Retval: k.angle computed in kalman_angle function
*/
float kalman_get_angle()
{
	if(kalman.angle > 180)
	{
	 kalman.angle-=360;
	}
	else if (kalman.angle < -180)
	{
		kalman.angle+= 360;
	}
	return kalman.angle;
}