/*
 * stm32f103xx_rcc.h
 *
 *  Created on: Nov 3, 2022
 *      Author: Metisai_02
 */

#ifndef INC_STM32F103XX_RCC_H_
#define INC_STM32F103XX_RCC_H_

#include <stm32f103xx.h>


//
uint32_t RCC_GetPCLK1Value();
uint32_t RCC_GetPCLK2Value();
void SysTickInit(void);
uint32_t GetTick(void);


#endif /* INC_STM32F103XX_RCC_H_ */
