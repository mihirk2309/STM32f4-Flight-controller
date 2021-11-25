#include "GyroData.h"

float roll = 0;
float yaw = 0;
float pitch = 0;
float dt = 0;
float tick;


void initGyroLCD(void)
{
	lcd_clear();
	lcd_gotoxy(1,1);
	lcd_string("R=");
	lcd_gotoxy(1,9);
	lcd_string("Y=");
	lcd_gotoxy(2,1);
	lcd_string("P=");
	
}

void GetGyroData(void)
{
	dt=HAL_GetTick()-tick;
	//imu_get_data_roll(&dt,&roll,0);
	lcd_print(1,3,pitch,4);
	
	dt=HAL_GetTick()-tick;
	//imu_get_data_pitch(&dt,&pitch,0);
	lcd_print(2,3,roll,4);
	
	dt=HAL_GetTick()-tick;
	//imu_get_data_yaw(&dt,&yaw,0);
  lcd_print(1,11,yaw,4);
}
