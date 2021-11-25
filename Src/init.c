
#include "init.h"

bldc rc_bldc1 = {&(TIM12->CCR1),1000/2,5000/2};	
bldc rc_bldc2 = {&(TIM12->CCR2),1000/2,5000/2};
bldc rc_bldc3 = {&(TIM5->CCR1),1000/2,5000/2};	
bldc rc_bldc4 = {&(TIM5->CCR2),1000/2,5000/2};


bldc *bldc1 = &rc_bldc1;
bldc *bldc2 = &rc_bldc2;
bldc *bldc3 = &rc_bldc3;
bldc *bldc4 = &rc_bldc4;


void bldc_pwm_start(void)
{
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
}


void delay_init(TIM_HandleTypeDef *htim)
{
	HAL_TIM_Base_Start(htim);
	DELAY_TIM->CNT = 0x0000;
}
