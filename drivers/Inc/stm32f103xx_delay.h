/*
 * stm32f103xx_delay.h
 *
 *  Created on: Dec 2, 2022
 *      Author: Metisai_02
 */

#ifndef INC_STM32F103XX_DELAY_H_
#define INC_STM32F103XX_DELAY_H_
#include <stm32f103xx.h>

//typedef struct
//{
//	uint32_t *myTicks;
//}DelayHandle_t;

void DelayInit(void);
//void Delayus(uint8_t value,DelayHandle_t *delay1);
void Delayms(uint32_t ms);
#endif /* INC_STM32F103XX_DELAY_H_ */
