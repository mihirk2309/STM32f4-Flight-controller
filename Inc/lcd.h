#ifndef LCD_H
#define LCD_H

/* Private define ------------------------------------------------------------*/
#define PORTWISE 0
#define BITWISE 1


/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "string.h"
#include "prep.h"

/************************************************************************/
/*					 USER Defines
   LCD Controlling Mode : Port wise or Bitwise                     */
/************************************************************************/
#define LCD_MODE BITWISE

#if (LCD_MODE == PORTWISE)
#define RS_BIT 8
#define RW_BIT 1
#define EN_BIT 9
#define LCD_PORT D
#endif

#if (LCD_MODE == BITWISE)


#define RS_PORT GPIOC
#define RS_BIT LCD_RS_PIN

#define EN_PORT GPIOC
#define EN_BIT LCD_EN_PIN

#define D4_PORT GPIOC
#define D4_BIT LCD_D4_PIN

#define D5_PORT GPIOD
#define D5_BIT LCD_D5_PIN

#define D6_PORT GPIOB
#define D6_BIT LCD_D6_PIN

#define D7_PORT GPIOB
#define D7_BIT LCD_D7_PIN

#endif

/************************************************************************/

#if((LCD_MODE != PORTWISE) && (LCD_MODE != BITWISE))
	#error "LCD_MODE is incorrect"
#endif

#ifndef LCD_MODE
	#error "LCD_MODE not defined"
#endif


/* Private defines */
#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"
#define BYTETOBINARY(byte)  \
(byte & 0x80 ? 1 : 0), \
(byte & 0x40 ? 1 : 0), \
(byte & 0x20 ? 1 : 0), \
(byte & 0x10 ? 1 : 0), \
(byte & 0x08 ? 1 : 0), \
(byte & 0x04 ? 1 : 0), \
(byte & 0x02 ? 1 : 0), \
(byte & 0x01 ? 1 : 0)


/** @defgroup LCD LCD.C
  * @{
  */

/**
  * @brief  Clears the LCD and set cursor to Home Position
  * @param  None
  * @retval None
  */

#define CLEAR_LCD lcd_clear();

/**
  * @brief  Sets cursor to Home Position
  * @param  None
  * @retval None
  */
#define LCD_HOME lcd_home();


/* User define ------------------------------------------------------------*/

/** @defgroup LCD Operating Mode Definitions
  * @brief    In PORTWISE Mode Px4 to Px7 of Port must be mapped to D4 to D7 of LCD. In BITWISE mode all signals can be mapped to any ports.
  * @{
  */

/* Function Declarations */
void pulse(void);
void lcd_write(uint8_t data);
void lcd_cmd(uint8_t cmd);
void lcd_home(void);
void lcd_clear(void);
void lcd_gotoxy(uint8_t row, uint8_t column);
void lcd_string(const char string[]);
void lcd_char(char character);
void lcd_int(int integer);
void lcd_uint(unsigned int integer);
void lcd_long(long long_int);
void lcd_float(float number);
void lcd_float_print(float number, uint8_t digits, uint8_t precision);
void lcd_print(uint8_t row, uint8_t column, int value, uint8_t digits);
void lcd_reg(uint32_t reg,uint8_t bits, uint8_t base);
void lcd_set_4bit(void);
void lcd_init(void);
void lcd_center(int row, char string[]);

#endif /* LCD_H_ */
