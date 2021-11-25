#ifndef INIT_H_
#define INIT_H_

#include "main.h"
#include "prep.h"
#include "lcd.h"
#include "delay.h"
#include "motor.h"
#include "imu.h"
#include "mpu9250.h"
#include "RC_Receiver.h"
#include "GyroData.h"
#include "pid.h"
#include "MadgwickAHRS.h"

#define DELAY_MODULE					(&htim7)

#define UART												(&huart2)
#define I2C_MAIN_BUS								(&hi2c1)
#define I2C_EXTRA_BUS								(&hi2c3)

		
#define DELAY_TIM						TIM7


// modules definition

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim5;

extern UART_HandleTypeDef huart2;

void delay_init(TIM_HandleTypeDef *htim);

extern bldc *bldc1;
extern bldc *bldc2;
extern bldc *bldc3;
extern bldc *bldc4;

void bldc_pwm_start(void);
void delay_init(TIM_HandleTypeDef *htim);

#endif /* INIT_H_ */

