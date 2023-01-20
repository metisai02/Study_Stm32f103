/*
 * lcd.c
 *
 *  Created on: Nov 15, 2022
 *      Author: Metisai_02
 */
#include <stdio.h>
#include <string.h>
#include "stm32f103xx.h"
#include <stm32f103xx_i2c_lcd_driver.h>

#define LCD_ADDR 		0x27

/*
 * PB6 -> SCL
 * PB7 -> SDA
 */

I2C_Handle_t I2C1Handle;
I2C_LCD_Handle_t LCD;

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

//void GPIO_ButtonInit(void){
//	GPIO_Handle_t gpioButton;
//	gpioButton.pGPIOx = GPIOA;
//	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
//	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
//	gpioButton.GPIO_PinConfig.GPIO_PinCNF = GPIO_CNF_IN_PUPD;
//	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
//	GPIO_PeriClockControl(GPIOA, ENABLE);
//	GPIO_Init(&gpioButton);
//}

void I2C1_Inits(void){
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x0;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SPEED_SM;
	I2C_Init(&I2C1Handle);
}


int main(void){

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral confiration
	I2C1_Inits();

	//enable the peripheral

	I2C_LCD_init(&LCD, &I2C1Handle, LCD_ADDR);
	I2C_LCD_set_cursor(&LCD, 0, 0);
	I2C_LCD_print_string(&LCD, "trungdaubuoi 2");
	while(1){

	}
}

