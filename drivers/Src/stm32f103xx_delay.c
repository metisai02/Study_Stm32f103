/*
 * stm32f103xx_delay.c
 *
 *  Created on: Dec 2, 2022
 *      Author: Metisai_02
 */

#include <stm32f103xx_delay.h>

void DelayInit(void)
{
	uint32_t Clk,temp;

	TIM4_PCLK_EN();
	Clk = RCC_GetPCLK1Value();
	if(((RCC->CFGR >> 10) & 1) == 0 )
	{
		Clk = Clk;
	}
	else {
		Clk = Clk*2;
	}

	temp = Clk/1000; // 1kHZ

	TIM4->PSC = (uint16_t)temp; // once count means 1 ms
	TIM4->ARR = 0xffff; //maximum the value of ARR register
	TIM4->EGR  |= (1 << TIM_EGR_UG);

}

void Delayms(uint32_t ms)
{
	TIM4->CR1 |= (1 << TIM_CR1_CNE);
    uint16_t start;
    uint32_t remaining = ms + 1u; // Add one to guarantee 'at least' us

    start = TIM4->CNT;

    while (remaining > 0u)
    {
        uint16_t delay = (remaining > 0xFFFFu) ? 0xFFFFu : (uint16_t)remaining;
        uint16_t end;

        remaining -= delay;

        do {
            end = TIM4->CNT;
        } while ((end - start) < delay);

        start = end;
    }
    TIM4->CR1 &= ~TIM_CR1_CNE;
}

