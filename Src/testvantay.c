/*
 * testvantay.c
 *
 *  Created on: Nov 16, 2022
 *      Author: Metisai_02
 */

#include <stdio.h>
#include <string.h>
#include "stm32f103xx.h"
#include <stm32f103xx_i2c_lcd_driver.h>
#include <vantay.h>

#define LCD_ADDR 		0x27


I2C_Handle_t I2C1Handle;
I2C_LCD_Handle_t LCD;

uint8_t IDFromFinger;
uint8_t CurrentNumberFinger;

void I2C1_GPIOInits(void);
void I2C1_Inits(void);

char str1[17]={'\0'};
char str2[17]={'\0'};
USART_Handle_t Usart1;
void USART_GPIOInits(void)
{
	GPIO_Handle_t USARTPins;
	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_ALOUT_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_50MHZ;
	// TX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&USARTPins);

	//  RX
	USARTPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_IN;
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	USARTPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_IN_FLOAT;
	GPIO_Init(&USARTPins);




}
void USART_Inits(void)
{


	Usart1.pUSARTx = USART1;
	Usart1.USARTConfig.Mode = USART_MODE_TXRX;
	Usart1.USARTConfig.WordLength = USART_WORD_LENGTH_8;
	Usart1.USARTConfig.NuOfStopBits = USART_STOPBITS_1;
	Usart1.USARTConfig.Baudrate = USART_STD_BAUD_57600;
	Usart1.USARTConfig.ParityControl = USART_PARITY_DI;

	USART_Init(&Usart1);
}
int main(void)
{
		uint8_t FingerResult;
		uint32_t start1,end1;
uint8_t Temp[1];

			SysTickInit();
			DelayInit();
			USART_GPIOInits();
			USART_Inits();
			Delayms(200);
			USART_PeriControl(USART1, ENABLE);
			I2C1_GPIOInits();
			I2C1_Inits();
			I2C_LCD_init(&LCD, &I2C1Handle, LCD_ADDR);



			I2C_LCD_set_cursor(&LCD, 0, 0);
			I2C_LCD_print_string(&LCD, "Press Finger");

			Delayms(2000);
			CurrentNumberFinger = GetNumberOfFinger(Usart1);
			sprintf(str1,"Card:%d", CurrentNumberFinger);
			Temp[0]=CurrentNumberFinger+48;
			if(CurrentNumberFinger==0xFF) CurrentNumberFinger=0;
			I2C_LCD_display_clear(&LCD);
			I2C_LCD_set_cursor(&LCD, 0, 0);
			I2C_LCD_print_string(&LCD, Temp);

			Delayms(1000);
			start1 = GetTick();
			FingerResult = RegistryNewFinger(Usart1,CurrentNumberFinger+1,LCD);
			end1 = GetTick() - start1;
				if(FingerResult==FP_OK)
				{
					I2C_LCD_display_clear(&LCD);
					I2C_LCD_set_cursor(&LCD, 0, 0);
					I2C_LCD_print_string(&LCD, "successful");
				}
				else
				{
					I2C_LCD_display_clear(&LCD);
					I2C_LCD_set_cursor(&LCD, 0, 0);
					I2C_LCD_print_string(&LCD, "NOPE!");
				}


while(1){
		start1 = GetTick();
		FingerResult=CheckFinger(Usart1,LCD);
		end1 = GetTick() - start1;
		  if(FingerResult==FP_OK)
			{
			  I2C_LCD_set_cursor(&LCD, 0, 0);
			  I2C_LCD_display_clear(&LCD);
			  I2C_LCD_print_string(&LCD, "bonus 1 buscu");
				Temp[0]=IDFromFinger+48;
			I2C_LCD_set_cursor(&LCD, 0, 0);
			I2C_LCD_print_string(&LCD, Temp);
			}
			else
			{
				I2C_LCD_display_clear(&LCD);
				I2C_LCD_set_cursor(&LCD, 0, 0);
				I2C_LCD_print_string(&LCD, "Luon chym cut");
			}
			Delayms(1000);

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
