/*
 * testKeyPad.c
 *
 *  Created on: Dec 13, 2022
 *      Author: Metisai_02
 */


#include <stdint.h>
#include <string.h>
#include "stm32f103xx.h"
#include "stm32f103xx_i2c_lcd_driver.h"
#include "keypad.h"

void delay(uint32_t time);
void reset(void);

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
	I2C_PeripheralControl(I2C1, ENABLE);

	uint8_t numberOfError = 0;
	char key;
	char password[4] = {'1', '2', '3', '4'};
	char getPassword[4];
	uint8_t count = 0;

	KEYPAD_Inits();
	//I2C_LCD_init(&LCD, &I2C1Handle, LCD_ADDR);
	//reset();
	while(1){
		key = getChar();
		I2C_LCD_print_string(&LCD, &key);
		delay(1000);
	}

	return 0;
}

void reset(void){
	I2C_LCD_set_cursor(&LCD, 0, 0);
	I2C_LCD_print_string(&LCD, "NHAP MAT KHAU 2");
	I2C_LCD_set_cursor(&LCD, 1, 4);
}

void delay(uint32_t time){
	for(uint32_t i = 0 ; i < (time * 1000); i++);
}
