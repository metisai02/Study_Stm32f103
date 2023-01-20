/*
 * testDelay.c
 *
 *  Created on: Dec 7, 2022
 *      Author: Metisai_02
 */
#include <stm32f103xx.h>
void delay(void)
{
	for(uint32_t i = 0; i < 1000000/2 ;i++ );
}
int main(void)
{
	GPIO_Handle_t GpioLed;
	//cấu hình cho led gpio

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_2MHZ;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GpioLed.GPIO_PinConfig.GPIO_CRF =GPIO_CNF_GEOUT_PP;

	GPIO_Init(&GpioLed);

	DelayInit();


	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_13);
		Delayms(1000);

	}
	return 0;
}
