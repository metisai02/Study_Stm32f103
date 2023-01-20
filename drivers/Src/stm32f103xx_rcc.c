/*
 * stm32f103xx_rcc.c
 *
 *  Created on: Nov 3, 2022
 *      Author: Metisai_02
 */
#include <stm32f103xx_rcc.h>
uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_Prescaler[4] = {2, 4, 8, 16};
uint8_t APB2_Prescaler[4] = {2, 4, 8, 16};
uint8_t PLLMUL[16] = {2, 3, 4, 5, 6, 7, 8, 9 ,10, 11, 12, 13, 14, 15, 16, 16};

volatile uint32_t	tick;

uint32_t RCC_PillSystem()
{
	uint8_t temp,PillStatus,PLLXTPRE,pllmul;
	uint32_t PillClk,PillSrc;

	PillStatus = ( RCC->CFGR >> 16 ) & 0x01 ;
	if(PillStatus == 0)
	{
		// HSI oscillator clock / 2 selected
		PillSrc = 4000000;
	}
	else
	{
		// HSE oscillator clock selected as PLL input clock
		PLLXTPRE = ( RCC->CFGR >> 17 ) & 0x01 ;
		if(PLLXTPRE == 1)
		{
			//HSE clock divided by 2
			PillSrc = 4000000;
		}
		else
		{
			//HSE clock divided by 2
			PillSrc = 8000000;
		}
	}
	temp = ( RCC->CFGR >> 18 ) & 0xFF;
	pllmul = PLLMUL[temp];

	PillClk = PillSrc * pllmul;

	return PillClk;

}


uint32_t RCC_GetPCLK1Value()
{
	uint32_t pclk1,SystemCLK;
	uint8_t clkStatus,temp,ahb,apb1;

	clkStatus = ( RCC->CFGR >> 2 ) & 0x03 ;


	if(clkStatus == 0)
	{
		//  HSI oscillator used as system clock
		SystemCLK = 8000000;
	}
	else if (clkStatus == 1)
	{
		//HSE oscillator used as system clock
		SystemCLK = 8000000;
	}
	else if (clkStatus == 2)
	{
		RCC_PillSystem();
	}

	temp = (RCC->CFGR >> 4) & 0x0F;
	if(temp < 8)
	{
		ahb = 1;
	}
	else
	{
		ahb = AHB_Prescaler[temp - 8];
	}
	temp = (RCC->CFGR >> 8) & 0x07;
	if(temp < 4)
	{
		apb1 = 1;
	}
	else
	{
		apb1 = APB1_Prescaler[temp - 4];
	}


	pclk1 = (SystemCLK/ahb)/apb1;

	return pclk1;
}
uint32_t RCC_GetPCLK2Value()
{
	{
		uint32_t pclk2,SystemCLK;
		uint8_t clkStatus,temp,ahb,apb2;

		clkStatus = ( RCC->CFGR >> 2 ) & 0x03 ;


		if(clkStatus == 0)
		{
			//  HSI oscillator used as system clock
			SystemCLK = 8000000;
		}
		else if (clkStatus == 1)
		{
			//HSE oscillator used as system clock
			SystemCLK = 8000000;
		}
		else if (clkStatus == 2)
		{
			// PLL used as system clock
			RCC_PillSystem();
		}

		temp = (RCC->CFGR >> 4) & 0x0F;
		if(temp < 8)
		{
			ahb = 1;
		}
		else
		{
			ahb = AHB_Prescaler[temp - 8];
		}
		temp = (RCC->CFGR >> 11) & 0x07;
		if(temp < 4)
		{
			apb2 = 1;
		}
		else
		{
			apb2 = APB1_Prescaler[temp - 4];
		}


		pclk2 = (SystemCLK/ahb)/apb2;

		return pclk2;
	}
}
void SysTickInit(void)
{
	SYSTICK->LOAD = 8U*1000U - 1U;
	SYSTICK->VAL = 0U;
	SYSTICK->CTRL |= (1 << SYSTICK_CSR_TICKINT);
	SYSTICK->CTRL |= (1 << SYSTICK_CSR_CLKSOURCE);
	SYSTICK->CTRL |= (1 << SYSTICK_CSR_ENABLE);
}
uint32_t GetTick(void)
{
	return tick;
}
void SysTick_Handler(void)
{
	tick++;
}

