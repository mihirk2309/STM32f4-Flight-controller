#include "init.h"
#include "motor.h"

void bldc_set_pulse(bldc *ser,uint16_t on_time)
{
	if(on_time>(ser->max_pulse))
		*(ser->tim_ch) = 2*(ser->max_pulse);
	else if (on_time<(ser->min_pulse))
		*(ser->tim_ch) = 2*(ser->min_pulse);
	else
		*(ser->tim_ch) = 2*on_time;
}

