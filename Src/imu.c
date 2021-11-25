#include "init.h"
#include "imu.h"
#include "mpu9250.h"
#include "main.h"
#include "kalman.h"
#include "RK4integration.h"

#define SENSI_CONST 65.5f

uint8_t imu_data[10]= {0};
uint8_t imu_cmd[10]= {0};


int32_t sum_X[3]={0},sum_Y[3]={0},sum_Z[3]={0};
float yaw_bias = 0;


void imu_init(void)
{
//	int16_t mag_x[100],mag_y[100],mag_z[100] = {0};
	int16_t ax, ay, az, gx, gy, gz;
	float data =0;
	int i=0;
	mpu9250_initialize();
	for (i=0;i<100;i++)
	{	
			getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

			gx=gx;// /SENSI_CONST;
			gy=gy;// /SENSI_CONST;
			gz=gz;// /SENSI_CONST;
			ax=ax/2048.0f;
			ay=ay/2048.0f;
			az=az/2048.0f;	
		
			sum_X[0] += ax;
			sum_X[1] += gx;
			
		
			sum_Y[0] += ay;
			sum_Y[1] += gy;
			
		
			sum_Z[0] += az;
			sum_Z[1] += gz;
			
	}
	for(i=0;i<3;i++){
		sum_X[i] /= 100;
		sum_Y[i] /= 100;
		
	}
	//MahonyAHRSupdate(gx-sum_X[1],gy-sum_Y[1],gz-sum_Z[1],ax-sum_X[0],ay-sum_Y[0],az-sum_Z[0],mx-sum_X[2],my-sum_Y[2],mz-sum_Z[2],&data);
	//MahonyAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz,&data);
	yaw_bias = data;
	
}

void imu_get_data_roll(float* dt, float* role,int n)
{		
		float exp_angl,meas_angl;
		static float prev_dt = 0,prev_exp_angl=0,prev_dt_angle=0,prev_meas_angl;
	  *dt=(*dt/1000.0f);
	  if(n == 1)
		{
		   exp_angl = RungeKutta4(prev_dt_angle,prev_exp_angl,*dt,*dt - prev_dt,&ExpectRate1);
       //lcd_print(2,1,exp_angl,4);
		   meas_angl = RungeKutta4(prev_dt_angle,prev_exp_angl,*dt,*dt - prev_dt,&GyroRate1);
	     //lcd_print(2,12,meas_angl,4);
			 kalman_angle(exp_angl,meas_angl);
		   prev_exp_angl = exp_angl;
		   prev_dt_angle = *dt;
			 prev_dt = *dt;
	     *role = kalman_get_angle();
		   //lcd_print(1,1,*role,4);
		   //lcd_string(" n is 1");
		}
		else 
		{
			 meas_angl = RungeKutta4(prev_dt_angle,prev_meas_angl,*dt,*dt - prev_dt,&GyroRate1);
			 prev_dt_angle = *dt;
			 prev_meas_angl = meas_angl;
			 prev_dt = *dt;
			 *role = meas_angl;
			 //lcd_print(1,1,*role,4);
		}
	}
void imu_get_data_pitch(float* dt, float* pitch,int n)
{
	  float exp_angl,meas_angl;
		static float prev_dt = 0,prev_exp_angl=0,prev_dt_angle=0,prev_meas_angl;
	  *dt=(*dt/1000.0f);
		if(n == 1)
		{
		   exp_angl = RungeKutta4(prev_dt_angle,prev_exp_angl,*dt,*dt - prev_dt,&ExpectRate2);
      // lcd_print(2,1,exp_angl,4);
		   meas_angl = RungeKutta4(prev_dt_angle,prev_exp_angl,*dt,*dt - prev_dt,&GyroRate2);
	     //lcd_print(2,12,meas_angl,4);
			 kalman_angle(exp_angl,meas_angl);
		   prev_exp_angl = exp_angl;
		   prev_dt_angle = *dt;
			 prev_dt = *dt;
	     *pitch = kalman_get_angle();
		   //lcd_print(1,1,*yaw,4);
		   	//lcd_string(" n is 1");
		}
		else 
		{
			 meas_angl = RungeKutta4(prev_dt_angle,prev_meas_angl,*dt,*dt - prev_dt,&GyroRate2);
			 prev_dt_angle = *dt;
			 prev_meas_angl = meas_angl;
			 prev_dt = *dt;
			 *pitch = meas_angl;
			// lcd_print(1,1,*yaw,4);
		}
	}
void imu_get_data_yaw(float* dt, float* yaw,int n)
{
	 float exp_angl,meas_angl;
		static float prev_dt = 0,prev_exp_angl=0,prev_dt_angle=0,prev_meas_angl;
	  *dt=(*dt/1000.0f);
		if(n == 1)
		{
		   exp_angl = RungeKutta4(prev_dt_angle,prev_exp_angl,*dt,*dt - prev_dt,&ExpectRate3);
      // lcd_print(2,1,exp_angl,4);
		   meas_angl = RungeKutta4(prev_dt_angle,prev_exp_angl,*dt,*dt - prev_dt,&GyroRate3);
	     //lcd_print(2,12,meas_angl,4);
			 kalman_angle(exp_angl,meas_angl);
		   prev_exp_angl = exp_angl;
		   prev_dt_angle = *dt;
			 prev_dt = *dt;
	     *yaw = kalman_get_angle();
		   //lcd_print(1,1,*yaw,4);
		   //	lcd_string(" n is 1");
		}
		else 
		{
			 meas_angl = RungeKutta4(prev_dt_angle,prev_meas_angl,*dt,*dt - prev_dt,&GyroRate3);
			 prev_dt_angle = *dt;
			 prev_meas_angl = meas_angl;
			 prev_dt = *dt;
			 *yaw = meas_angl;
			// lcd_print(1,1,*yaw,4);
		}
	
}

float GyroRate1(float angle,float t){
	int16_t gy = (getRotationY()/SENSI_CONST);
	//lcd_print(1,12,gz,4);
	return gy;
}
float GyroRate2(float angle,float t){
	int16_t gx = (getRotationX()/SENSI_CONST);
	//lcd_print(1,12,gz,4);
	return gx;
}
float GyroRate3(float angle,float t){
	int16_t gz = (getRotationZ()/SENSI_CONST);
	//lcd_print(1,12,gz,4);
	return gz;
}


float ExpectRate1(float angle,float t){
	return WMEGA;
}
float ExpectRate2(float angle,float t){
	return WMEGA;
}
float ExpectRate3(float angle,float t){
	return WMEGA;
}
