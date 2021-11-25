#ifndef MOTOR_H_
#define MOTOR_H_

// Includes
#include "stdint.h"
#include "prep.h"


struct bldc_t
{
	__IO uint32_t *tim_ch;
	uint16_t min_pulse;
	uint16_t max_pulse;
	
};

typedef struct bldc_t bldc;
void bldc_set_pulse(bldc *ser,uint16_t on_time);
//void bldc_set_angle(bldc *ser, int angle);


#endif /* MOTOR_H_ */


