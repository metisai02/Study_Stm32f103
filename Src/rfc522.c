/*
 * rfc522.c
 *
 *  Created on: Nov 15, 2022
 *      Author: Metisai_02
 */
#include <stdio.h>
#include <string.h>
#include "stm32f103xx.h"
#include <stm32f103xx_i2c_lcd_driver.h>
#include <stm32f103xx_rc522.h>
#include <stm32f103xx_init.h>

#define LCD_ADDR 		0x27
void delay(void)
{
	for(uint32_t i = 0; i < 1000000/2 ;i++ );
}
/*
 * PB6 -> SCL
 * PB7 -> SDA
 */

I2C_Handle_t I2C1Handle;
I2C_LCD_Handle_t LCD;
void I2C1_GPIOInits(void);
void I2C1_Inits(void);


uchar status;
uchar str[MAX_LEN];
char str1[17]={'\0'};
char str2[17]={'\0'};

uchar UID[5];
int main(void)
{

	I2C1_GPIOInits();
	I2C1_Inits();
	I2C_LCD_init(&LCD, &I2C1Handle, LCD_ADDR);

	MFRC522_Init();



	while(1)
	{
		I2C_LCD_display_clear(&LCD);
		I2C_LCD_set_cursor(&LCD, 1, 2);
		I2C_LCD_print_string(&LCD, "Dua the vao !!");
		  for (int i = 0; i < 16; i++) {
			  str[i] = 0;
		  }
		status = MFRC522_Request(PICC_REQIDL, str);
		if (status == MI_OK)
		{
			sprintf(str1,"Card:%x,%x,%x", str[0], str[1], str[2]);
			status = MFRC522_Anticoll(str);
			  if(status == MI_OK)
			  {
				  sprintf(str2,"nga:%x,%x,%x,%x", str[0], str[1], str[2],str[3]);
				  UID[0] = str[0];
				  UID[1] = str[1];
				  UID[2] = str[2];
				  UID[3] = str[3];
				  UID[4] = str[4];
			  }
				I2C_LCD_display_clear(&LCD);
				I2C_LCD_set_cursor(&LCD, 0, 0);
				I2C_LCD_print_string(&LCD, str1);
				I2C_LCD_set_cursor(&LCD, 1, 0);
				I2C_LCD_print_string(&LCD, str2);
				delay();
				I2C_LCD_display_clear(&LCD);
		}
		else
		{

			I2C_LCD_display_clear(&LCD);
//			I2C_LCD_print_string(&LCD, "waiting  your card");
//			I2C_LCD_display_clear(&LCD);
//			I2C_LCD_set_cursor(&LCD, 0, 0);
//			I2C_LCD_print_string(&LCD, str1);
//			I2C_LCD_set_cursor(&LCD, 1, 0);
//			I2C_LCD_print_string(&LCD, str2);
		}
	}

}
void I2C1_GPIOInits(void){
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_ALOUT_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_10MHZ;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);

	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void){
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x0;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SPEED_SM;
	I2C_Init(&I2C1Handle);
}
