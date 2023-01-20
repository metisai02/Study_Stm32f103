/*
 * testUsartFP.c
 *
 *  Created on: Dec 8, 2022
 *      Author: Metisai_02
 */
#include <stdio.h>
#include <string.h>
#include "stm32f103xx.h"
USART_Handle_t Usart1;
void USART_GPIOInits(void)
{
	GPIO_Handle_t USARTPins;
	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_ALOUT_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_50MHZ;
	// cấu hình TX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&USARTPins);

	// cấu hình RX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
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

			DelayInit();
			Delayms(200);
			SysTickInit();
			USART_GPIOInits();
			USART_Inits();
			Delayms(200);
			Usart1.pUSARTx->DR = (uint16_t)0U;
			USART_PeriControl(USART1, ENABLE);


			char data[] = "lam sao tao biet la may dung he lo dcmasdljadlsadkljas\n";
			char data1[] = "hello wworld\n";
			for(uint8_t i=0; i<10; i++)
			{
			HAL_UART_Transmit(&Usart1, (uint8_t*)data, strlen(data), 1000);
			}
			for(uint8_t y=0; y<10; y++)
			{
			HAL_UART_Transmit(&Usart1, (uint8_t*)data1, strlen(data1), 1000);
			}
				while(1){}
}
