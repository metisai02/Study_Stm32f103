/*
 * testUsartIT.c
 *
 *  Created on: Dec 13, 2022
 *      Author: Metisai_02
 */


#include <stdio.h>
#include <string.h>
#include "stm32f103xx.h"
#include <stm32f103xx_i2c_lcd_driver.h>
#include <vantay.h>

#define LCD_ADDR 		0x27

		uint8_t data = 0;

I2C_Handle_t I2C1Handle;
I2C_LCD_Handle_t LCD;

uint8_t IDFromFinger;
uint8_t CurrentNumberFinger;

void I2C1_GPIOInits(void);
void I2C1_Inits(void);


USART_Handle_t Usart1;
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
	HAL_UART_Transmit(&Usart1, &data, 1, 1000);
	USART_ReceiveDataIT(&Usart1, &data, 1);
}
void USART_GPIOInits(void)
{
	GPIO_Handle_t USARTPins;
	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_ALOUT_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_50MHZ;
	// TX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_Init(&USARTPins);

	//  RX
	USARTPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_IN;
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	USARTPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_IN_FLOAT;
	GPIO_Init(&USARTPins);

	USART_IRQConfig(IRQ_NO_USART2, ENABLE);




}
void USART_Inits(void)
{


	Usart1.pUSARTx = USART2;
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
			//I2C1_GPIOInits();
			//I2C1_Inits();
			//I2C_LCD_init(&LCD, &I2C1Handle, LCD_ADDR);




			USART_ReceiveDataIT(&Usart1, &data, 1);
while(1){

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
void USART2_IRQHandler(void)
{
	USART_IRQHandling(&Usart1);
}
