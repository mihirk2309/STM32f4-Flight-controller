#ifndef PREP_H_
#define PREP_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "stdbool.h"

#define sbit(sfr,bit)		(sfr |= (1<<bit))
#define cbit(sfr,bit)		(sfr &= ~(1<<bit))
#define check(sfr, bit)		((sfr & ( 1<<bit ) ) ? 1 : 0)
#define COPY(src_sfr,src_bit,dest_sfr,dest_bit) ((check(src_sfr,src_bit)? sbit(dest_sfr,dest_bit) : cbit(dest_sfr,dest_bit)))

#ifdef STM32F446xx
#define setPin(port,pin)	((port->ODR)|= pin)
#define clrPin(port,pin)	((port->ODR)&= ~pin)
#define checkPin(port,pin)	((port->IDR & pin ) ? 1 : 0)	
#define copyReg2Pin(src_sfr,src_bit,dest_port,dest_pin) ((check(src_sfr,src_bit)? setPin(dest_port,dest_pin) : clrPin(dest_port,dest_pin)))
#endif



#define _CONCAT(arg1, arg2) arg1##arg2
#define PORT(reg) _CONCAT(PORT,reg)
#define DDR(reg) _CONCAT(DDR,reg)
#define PIN(reg) _CONCAT(PIN,reg)

#define ODR(REG) _CONCAT(REG,->ODR)
#define IDR(REG) _CONCAT(REG,->IDR)
#define MODER(REG) _CONCAT(REG,->MODER)

#define DEC (10)
#define BIN (2)
#define HEX (16)
#define OCT (8)
#define M_PI 3.1415926535897932384626433832795f

#define upper_byte16(reg16) ((reg16>>8)&0x00ff)
#define lower_byte16(reg16) (reg16&0x00ff)

#define upper_byte32(reg32) ((reg32>>24)&0x00ff)
#define lower_byte32(reg32) (reg32&0x00ff)
#define mid_upper_byte32(reg32) ((reg32>>16)&0x00ff)
#define mid_lower_byte32(reg32) ((reg32>>8)&0x00ff)
#define upper_word32(reg32) ((reg32>>16)&0x0000ffff)
#define lower_word32(reg32) (reg32&0x0000ffff)

#endif /* PREP_H_ */

