/*
 * GpioBotton.c
 *
 *  Created on: Oct 23, 2022
 *      Author: Metisai_02
 */
#include"stm32f103xx.gpio_driver.h"
void delay(void)
{
	for(uint32_t i = 0; i < 5000/2 ;i++ );
}
void GpioInits(void)
{
	GPIO_Handle_t GpioBtn;
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_2MHZ;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GpioLed.GPIO_PinConfig.GPIO_CRF =GPIO_CNF_GEOUT_PP;

	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_IN_PUPD;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	GpioBtn.GPIO_PinConfig.GPIO_PinIRQ_Trigger = GPIO_IRQ_FT;


	GPIO_Init(&GpioBtn);

	// cấu hình Interrupt

	GPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);

}
int main(void)
 {

	GpioInits();
	while(1)
	{

	}
 }
void EXTI0_IRQHandler()
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_0);
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_13);

}
