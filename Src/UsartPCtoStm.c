/*
 * UsartPCtoStm.c
 *
 *  Created on: Nov 4, 2022
 *      Author: Metisai_02
 */

#include <stm32f103xx.h>
#include<string.h>


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
	Usart1.USARTConfig.Baudrate = USART_STD_BAUD_9600;
	Usart1.USARTConfig.ParityControl = USART_PARITY_DI;

	USART_Init(&Usart1);


}
int main(void)
{
		SysTickInit();
			USART_GPIOInits();
			USART_Inits();
			char user_data[] = "hello\n";

			USART_PeriControl(USART1, ENABLE);

			HAL_UART_Transmit(&Usart1 ,(uint8_t*)user_data, sizeof(user_data),1000);
			while(1){
			}

}
