#include "init.h"
#include "RC_Receiver.h"

uint32_t Trig_THR1 = 0;
uint32_t Trig_THR2 = 0;
uint8_t Is_First_Captured_T = 0; 
uint32_t throttle;



uint32_t Trig_ELEV1 = 0;
uint32_t Trig_ELEV2 = 0;
uint8_t Is_First_Captured_E = 0;  
uint32_t elevator;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//*********************************************************
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  
	{
		if (Is_First_Captured_T == 0) 
		{
			Trig_THR1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); 
			Is_First_Captured_T = 1;  
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (Is_First_Captured_T == 1)   
		{
			Trig_THR2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  
			__HAL_TIM_SET_COUNTER(htim, 0);  
			if (Trig_THR2 > Trig_THR1)
			{
				throttle = Trig_THR2 - Trig_THR1;
			}
			Is_First_Captured_T = 0; 
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
			
			
		}
	}
	
	
	//*********************************************************
	//Elevator
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  
	{
		if (Is_First_Captured_E == 0) 
		{
			Trig_ELEV1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); 
			Is_First_Captured_E = 1;  
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (Is_First_Captured_E == 1)   
		{
			Trig_ELEV2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  
			__HAL_TIM_SET_COUNTER(htim, 0);  
			if (Trig_ELEV2 > Trig_ELEV1)
			{
				elevator = Trig_ELEV2 - Trig_ELEV1;
			}
			Is_First_Captured_E = 0; 
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
			
		}
	}
	
	
	
	
	
	
	
}





