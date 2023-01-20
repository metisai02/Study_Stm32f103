/*
 * testSystemTick.c
 *
 *  Created on: Dec 7, 2022
 *      Author: Metisai_02
 */

#include <stm32f103xx.h>

	void delay(void)
	{
		for(uint32_t i = 0; i < 100000/2 ;i++ );
	}
	void GpioInits(void)
	{
		GPIO_Handle_t GpioLed;

		GpioLed.pGPIOx = GPIOC;
		GpioLed.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_2MHZ;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
		GpioLed.GPIO_PinConfig.GPIO_CRF =GPIO_CNF_GEOUT_PP;

		GPIO_Init(&GpioLed);

		// cấu hình Interrupt


	}
	int main(void)
	 {
		SysTickInit();


		GpioInits();
		while(1)
		{
			uint32_t start = GetTick();
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_13);
			while((GetTick() - start) < 1000){}
		}
	 }

