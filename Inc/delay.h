#ifndef DELAY_H
#define DELAY_H

#include "prep.h"
#include "stdint.h"
#include "stm32f4xx.h"

void _delay_us(volatile uint16_t);
void _delay_ms(volatile uint16_t);
void _delay_counts(volatile uint16_t);

#endif /* DELAY_H_ */

