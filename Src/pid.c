#include "pid.h"

float pid_p_gain_roll = 0.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.2;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 10;              //Gain setting for the roll D-controller
int pid_max_roll = 350;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch;  //Gain setting for the pitch D-controller.
int pid_max_pitch;       //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 0.2;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 10;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 350;                     //Maximum output of the PID-controller (+/-)

bool auto_level = true;               	  //Auto level on (true) or off (false)

int esc_1, esc_2, esc_3, esc_4;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
bool gyro_angles_set;


void PID_CALCULATE()
{
	
	pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
	pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
	pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
	pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)
	
	//initGyroLCD();
//	GetGyroData();
	
	gyro_roll_input = -getRoll();
  gyro_pitch_input = -getPitch(); 		//pitch; //Made this as zero as pitch drifts.
  gyro_yaw_input = -yaw;
	
	/*
		lcd_print(1,1,gyro_roll_input,4);
		lcd_print(1,6,gyro_pitch_input,4);
		lcd_print(1,11,yaw,4);
	*/
	
	
	
	pid_i_mem_roll = 0;
  pid_last_roll_d_error = 0;
  pid_i_mem_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_mem_yaw = 0;
  pid_last_yaw_d_error = 0;
	
	
	pid_roll_setpoint = 0;
	pid_pitch_setpoint = 0;
	pid_yaw_setpoint = 0;
	
	
	pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
	
	esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
  esc_4 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (rear-right - CW)
  esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
  esc_2 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (front-left - CW)
}








