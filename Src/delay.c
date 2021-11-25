
#include "init.h"

void _delay_us(volatile uint16_t delay_us)
{
	DELAY_TIM->CNT = 0x0000;
	delay_us = delay_us*8;
	while((DELAY_TIM->CNT)<delay_us);
	return;
}

void _delay_ms(volatile uint16_t delay_ms)
{
		HAL_Delay(delay_ms);
}

void _delay_counts(volatile uint16_t delay_counts)
{
	DELAY_TIM->CNT = 0x0000;
	while(DELAY_TIM->CNT < delay_counts);
	return;
}

