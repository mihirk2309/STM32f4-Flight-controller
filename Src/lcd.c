#include "init.h"
#include "lcd.h"
#include "math.h"

/** @defgroup LCD Functions
* @{
*/

#if (LCD_MODE == PORTWISE)

/**
  * @brief  Used to send data latching pulse on EN
  * @param  None
  * @retval None
  */
void pulse(void)
{
	
	setPin(ODR(LCD_PORT),LCD_EN_Pin);				//Set Enable Pin
	_delay_us(5);					//delay
	clrPin(ODR(LCD_PORT),LCD_EN_Pin);				//Clear Enable Pin
	_delay_us(50);
}


/**
  * @brief  Used to write 8 bit data
  * @param  data: 8 bit data to be send
  * @retval None
  */
void lcd_write(uint8_t data)
{
	uint8_t temp;
	temp = data;
	temp = (temp & 0xF0);
	PORT(LCD_PORT) &= 0x0F;
	PORT(LCD_PORT) |= temp;
	setPin(PORT(LCD_PORT),LCD_RS_Pin);
	//clrPin(lcd_port,RW);
	pulse();

	data = data & 0x0F;
	data = data<<4;
	PORT(LCD_PORT) &= 0x0F;
	PORT(LCD_PORT) |= data;
	setPin(PORT(LCD_PORT),LCD_RS_Pin);
	//clrPin(lcd_port,RW);
	pulse();
}

/**
  * @brief  Used to write 8 bit command
  * @param  cmd: 8 bit command to be send
  * @retval None
  */
void lcd_cmd(uint8_t cmd)
{
	uint8_t temp;
	temp = cmd;
	temp = temp & 0xF0;
	PORT(LCD_PORT) &= 0x0F;
	PORT(LCD_PORT) |= temp;
	clrPin(PORT(LCD_PORT),LCD_RS_Pin);
	//clrPin(lcd_port,RW);
	pulse();

	cmd = cmd & 0x0F;
	cmd = cmd<<4;
	PORT(LCD_PORT) &= 0x0F;
	PORT(LCD_PORT) |= cmd;
	clrPin(PORT(LCD_PORT),LCD_RS_Pin);
	//clrPin(lcd_port,RW);
	pulse();

}

#endif

#if (LCD_MODE == BITWISE)
/**
  * @brief  Used to send data latching pulse on EN
  * @param  None
  * @retval None
  */
void pulse(void)
{
	_delay_us(20);
	setPin(LCD_EN_GPIO_Port,LCD_EN_Pin);				
	_delay_us(20);															
	clrPin(LCD_EN_GPIO_Port,LCD_EN_Pin);				
	_delay_us(100);
}


/**
  * @brief  Used to write 8 bit data
  * @param  data: 8 bit data to be send
  * @retval None
  */
void lcd_write(__IO uint8_t data)
{
	//HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
	copyReg2Pin(data,7,LCD_D7_GPIO_Port ,LCD_D7_Pin);
	copyReg2Pin(data,6,LCD_D6_GPIO_Port ,LCD_D6_Pin);
	copyReg2Pin(data,5,LCD_D5_GPIO_Port ,LCD_D5_Pin);
	copyReg2Pin(data,4,LCD_D4_GPIO_Port ,LCD_D4_Pin);
	setPin(LCD_RS_GPIO_Port,LCD_RS_Pin);
	//clrPin(lcd_port,RW);
	pulse();

	copyReg2Pin(data,3,LCD_D7_GPIO_Port,LCD_D7_Pin);
	copyReg2Pin(data,2,LCD_D6_GPIO_Port,LCD_D6_Pin);
	copyReg2Pin(data,1,LCD_D5_GPIO_Port,LCD_D5_Pin);
	copyReg2Pin(data,0,LCD_D4_GPIO_Port,LCD_D4_Pin);
	setPin(LCD_RS_GPIO_Port,LCD_RS_Pin);
	//clrPin(lcd_port,RW);
	pulse();
	//HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
  * @brief  Used to write 8 bit command
  * @param  cmd: 8 bit command to be send
  * @retval None
  */
void lcd_cmd(__IO uint8_t cmd)
{
	//HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
	copyReg2Pin(cmd,7,LCD_D7_GPIO_Port,LCD_D7_Pin);
	copyReg2Pin(cmd,6,LCD_D6_GPIO_Port,LCD_D6_Pin);
	copyReg2Pin(cmd,5,LCD_D5_GPIO_Port,LCD_D5_Pin);
	copyReg2Pin(cmd,4,LCD_D4_GPIO_Port,LCD_D4_Pin);
	clrPin(LCD_RS_GPIO_Port,LCD_RS_Pin);
	//clrPin(lcd_port,RW);
	pulse();

	copyReg2Pin(cmd,3,LCD_D7_GPIO_Port,LCD_D7_Pin);
	copyReg2Pin(cmd,2,LCD_D6_GPIO_Port,LCD_D6_Pin);
	copyReg2Pin(cmd,1,LCD_D5_GPIO_Port,LCD_D5_Pin);
	copyReg2Pin(cmd,0,LCD_D4_GPIO_Port,LCD_D4_Pin);
	clrPin(LCD_RS_GPIO_Port,LCD_RS_Pin);
	//clrPin(lcd_port,RW);
	pulse();
	//HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

#endif

/**
  * @brief  Used to set Cursor at Home Position
  * @param  None
  * @retval None
  */
void lcd_home(void)
{
	lcd_cmd(0x80);
	_delay_ms(3);
}

/**
  * @brief  Used to clear LCD and set cursor at home position
  * @param  None
  * @retval None
  */
void lcd_clear(void)
{
	lcd_cmd(0x01);
	_delay_ms(3);
}

/**
  * @brief  Used to set cursor at LCD. Index from 1
  * @param  row: row of cursor
  * @param column: column of cursor
  * @retval None
  */
void lcd_gotoxy(uint8_t row, uint8_t column)
{
	switch (row) {
	case 1:
		lcd_cmd(0x80 + column - 1); break;
	case 2:
		lcd_cmd(0xc0 + column - 1); break;
	case 3:
		lcd_cmd(0x90 + column - 1); break;
	case 4:
		lcd_cmd(0xd0 + column - 1); break;
	default:
		LCD_HOME;
	}
}

/**
  * @brief  Used to print String on LCD at current cursor position
  * @param  string: String to print
  * @retval None
  */
void lcd_string(const char string[])
{
	while(*(string)!= '\0')
	{
		lcd_write(*(string++));
	}
}


/**
  * @brief  Used to print character on LCD at current cursor position
  * @param  character: ASCII character to print
  * @retval None
  */
void lcd_char(char character)
{
	lcd_write((uint8_t)character);
}

/**
  * @brief  Used to print Signed Decimal Integer on LCD at current cursor position
  * @param  integer: Signed integer to be printed
  * @retval None
  */
void lcd_int(int integer)
{
	char buffer[9];
	sprintf(buffer,"%d",integer);
	lcd_string(buffer);

}

/**
  * @brief  Used to print Unsigned Decimal Integer on LCD at current cursor position
  * @param  integer: unsigned integer to be printed
  * @retval None
  */
void lcd_uint(unsigned int integer)
{
	char buffer[9];
	sprintf(buffer,"%u",integer);
	lcd_string(buffer);

}
/**
  * @brief  Used to print Decimal Long Integer on LCD at current cursor position
  * @param  long_int: signed long integer to be printed
  * @retval None
  */
void lcd_long(long long_int)
{
	char buffer[8];
	sprintf(buffer, "%ld", long_int);
	lcd_string(buffer);
}

/**
  * @brief  Used to print floating point number on LCD at current cursor position with default Precision
  * @param  number: floating point number to be printed
  * @retval None
  */
void lcd_float(float number)
{
	char buffer[8];
	long above_deci = (long) number;
	number = (float) (number - above_deci);
	unsigned long below_deci = (unsigned long) (number*100);
	sprintf(buffer,"%ld.%lu",above_deci,below_deci);
	lcd_string(buffer);
}

/**
  * @brief  Used to print floating point number on LCD at current cursor position with given precision
  * @param  number: floating point number to be printed
  * @param  digits: no. of digits to be printed before decimal point
  * @param  precision: no. of digits after decimal point
  * @retval None
  */
void lcd_float_print(float number, uint8_t digits, uint8_t precision)
{
	char buffer[12],buff_buffer[7];
	long above_deci = (long) number;
	number = (float) (number - above_deci);
	long below_deci = (long) ((number)*(powf(10,precision)));
	sprintf(buff_buffer,"%%0%uld",digits);
	sprintf(buffer,buff_buffer,above_deci);
	lcd_string(buffer);
	lcd_char('.');
	sprintf(buff_buffer,"%%%uld",precision);
	sprintf(buffer,buff_buffer,below_deci);
	lcd_string(buffer);
}


/**
  * @brief  Used to print integer on LCD at given cursor postion
  * @param  row: row of cursor
  * @param  column: column of cursor
  * @param  value: Integer to be printed
  * @param  digits: No. of digits to be displayed
  * @retval None
  */
void lcd_print(uint8_t row, uint8_t column, int value, uint8_t digits)
{
	char buffer[8],buffer_of_buffer[5];

	lcd_gotoxy(row,column);
	sprintf(buffer_of_buffer,"%%0%ud",digits);
	sprintf(buffer,buffer_of_buffer,value);
	lcd_string(buffer);
}

/**
  * @brief  Used to print n bit register on LCD at current cursor position. Max 8 bit for BIN
  * @param  reg: n bit integer (max 32 bit)
  * @param  bits: No. of bits of integer
  * @param  base: Base of system (BIN, DEC, HEX)
  * @retval None
  */
void lcd_reg(uint32_t reg,uint8_t bits, uint8_t base)
{
	char buffer[12],buff_buffer[7];
	switch(base)
	{
		case BIN:
			reg = reg & 0x000000FF;
			sprintf(buffer,BYTETOBINARYPATTERN,BYTETOBINARY(reg));
			break;

		case HEX:
			sprintf(buff_buffer,"%%#%03ux",(bits/4)+2);
			sprintf(buffer,buff_buffer,reg);
			break;

		case DEC:
		default:
			if(bits == 8) bits = 3;
			else if(bits == 16) bits = 5;
			else if(bits == 32) bits = 10;
			else bits = 0;
			sprintf(buff_buffer,"%%%03uu",bits);
			sprintf(buffer,buff_buffer,reg);
			break;
	}
	lcd_string(buffer);
}

#if (LCD_MODE == PORTWISE)
/**
  * @brief  Used internally to Initialise LCD in 4 bit mode (Don't use directly)
  * @param  Nome
  * @retval None
  */
void lcd_set_4bit(void)
{
	_delay_ms(50);

	clrPin(PORT(LCD_PORT),LCD_RS_Pin);				//RS=0 --- Command Input
	//clrPin(lcd_port,RW);										//RW=0 --- Writing to LCD
	PORT(LCD_PORT) = 0x30;										//Sending 3 in the upper nibble
	setPin(PORT(LCD_PORT),LCD_EN_Pin);				//Set Enable Pin
	_delay_us(1);					//delay
	clrPin(PORT(LCD_PORT),LCD_EN_Pin);				//Clear Enable Pin

	_delay_ms(5);

	clrPin(PORT(LCD_PORT),LCD_RS_Pin);				//RS=0 --- Command Input
	//clrPin(lcd_port,RW);				//RW=0 --- Writing to LCD
	PORT(LCD_PORT) = 0x30;				//Sending 3 in the upper nibble
	setPin(PORT(LCD_PORT),LCD_EN_Pin);				//Set Enable Pin
	_delay_us(1);					//delay
	clrPin(PORT(LCD_PORT),LCD_EN_Pin);				//Clear Enable Pin

	_delay_ms(5);

	clrPin(PORT(LCD_PORT),LCD_RS_Pin);				//RS=0 --- Command Input
	//clrPin(lcd_port,RW);				//RW=0 --- Writing to LCD
	PORT(LCD_PORT) = 0x30;				//Sending 3 in the upper nibble
	setPin(PORT(LCD_PORT),LCD_EN_Pin);				//Set Enable Pin
	_delay_us(1);					//delay
	clrPin(PORT(LCD_PORT),LCD_EN_Pin);				//Clear Enable Pin

	_delay_ms(5);

	clrPin(PORT(LCD_PORT),LCD_RS_Pin);				//RS=0 --- Command Input
	//	clrPin(lcd_port,RW);				//RW=0 --- Writing to LCD
	PORT(LCD_PORT) = 0x20;				//Sending 2 in the upper nibble to initialize LCD 4-bit mode
	setPin(PORT(LCD_PORT),LCD_EN_Pin);				//Set Enable Pin
	_delay_us(1);					//delay
	clrPin(PORT(LCD_PORT),LCD_EN_Pin);				//Clear Enable Pin
	_delay_ms(5);
}
#endif

#if (LCD_MODE == BITWISE)
/**
  * @brief  Used internally to Initialise LCD in 4 bit mode (Don't use directly)
  * @param  Nome
  * @retval None
  */
void lcd_set_4bit(void)
{
	__IO uint8_t data;
	_delay_ms(50);

	clrPin(LCD_RS_GPIO_Port,LCD_RS_Pin);				//RS=0 --- Command Input
	data = 0x30;				//Sending 3 in the upper nibble

	copyReg2Pin(data,7,LCD_D7_GPIO_Port,LCD_D7_Pin);
	copyReg2Pin(data,6,LCD_D6_GPIO_Port,LCD_D6_Pin);
	copyReg2Pin(data,5,LCD_D5_GPIO_Port,LCD_D5_Pin);
	copyReg2Pin(data,4,LCD_D4_GPIO_Port,LCD_D4_Pin);

	setPin(LCD_EN_GPIO_Port,LCD_EN_Pin);				//Set Enable Pin
	_delay_us(10);					//delay
	clrPin(LCD_EN_GPIO_Port,LCD_EN_Pin);				//Clear Enable Pin

	_delay_ms(50);

	clrPin(LCD_RS_GPIO_Port,LCD_RS_Pin);				//RS=0 --- Command Input
	data = 0x30;				//Sending 3 in the upper nibble

	copyReg2Pin(data,7,LCD_D7_GPIO_Port,LCD_D7_Pin);
	copyReg2Pin(data,6,LCD_D6_GPIO_Port,LCD_D6_Pin);
	copyReg2Pin(data,5,LCD_D5_GPIO_Port,LCD_D5_Pin);
	copyReg2Pin(data,4,LCD_D4_GPIO_Port,LCD_D4_Pin);

	pulse();

	_delay_ms(50);

	clrPin(LCD_RS_GPIO_Port,LCD_RS_Pin);				//RS=0 --- Command Input
	data = 0x30;				//Sending 3 in the upper nibble

	copyReg2Pin(data,7,LCD_D7_GPIO_Port,LCD_D7_Pin);
	copyReg2Pin(data,6,LCD_D6_GPIO_Port,LCD_D6_Pin);
	copyReg2Pin(data,5,LCD_D5_GPIO_Port,LCD_D5_Pin);
	copyReg2Pin(data,4,LCD_D4_GPIO_Port,LCD_D4_Pin);

	pulse();

	_delay_ms(50);

	clrPin(LCD_RS_GPIO_Port,LCD_RS_Pin);				//RS=0 --- Command Input
	data = 0x20;				//Sending 2 in the upper nibble to initialize LCD 4-bit mode
	
	copyReg2Pin(data,7,LCD_D7_GPIO_Port,LCD_D7_Pin);
	copyReg2Pin(data,6,LCD_D6_GPIO_Port,LCD_D6_Pin);
	copyReg2Pin(data,5,LCD_D5_GPIO_Port,LCD_D5_Pin);
	copyReg2Pin(data,4,LCD_D4_GPIO_Port,LCD_D4_Pin);

	pulse();
	
	_delay_ms(50);
}
#endif

/**
  * @brief  Used to Initialise LCD
  * @param  Nome
  * @retval None
  */
void lcd_init(void)
{
	//lcd port init
	#if (LCD_MODE == PORTWISE)
	setPin(DDR(LCD_PORT),LCD_RS_Pin);
	setPin(DDR(LCD_PORT),LCD_EN_Pin);
	DDR(LCD_PORT) |= 0xF0;
	#endif

	#if (LCD_MODE == BITWISE)
	/*setPin(DDR(LCD_RS_GPIO_Port),LCD_RS_Pin);
	setPin(DDR(LCD_EN_GPIO_Port),LCD_EN_Pin);
	setPin(DDR(LCD_D4_GPIO_Port),LCD_D4_Pin);
	setPin(DDR(LCD_D5_GPIO_Port),LCD_D5_Pin);
	setPin(DDR(LCD_D6_GPIO_Port),LCD_D6_Pin);
	setPin(DDR(LCD_D7_GPIO_Port),LCD_D7_Pin);*/
	#endif

	lcd_set_4bit();

	lcd_cmd(0x28); //4-bit mode and 5x8 dot character font
	_delay_ms(50);
	//lcd_cmd(0x0F); //Turn on LCD and cursor + blinking
	lcd_cmd(0x0E);//Turn on LCD and cursor + no blinking
	_delay_ms(50);
	lcd_cmd(0x01); //Clear LCD display
	_delay_ms(50);
	lcd_cmd(0x06); //Auto increment cursor position
	_delay_ms(50);
	lcd_cmd(0x80); //Set cursor position
	_delay_ms(50);

}
/**
  * @brief  Used to print string in center of LCD at given row
  * @param  row: row to which print
    @param  string: string to print
  * @retval None
  */
void lcd_center(int row, char string[])
{
	int len = strlen(string);
	lcd_gotoxy(row, 7 - len/2);			//for 20x4 lcd
	lcd_string(string);
}

