/*
 * stm32f103xx_init.h
 *
 *  Created on: Nov 15, 2022
 *      Author: Metisai_02
 */

#ifndef INC_STM32F103XX_INIT_H_
#define INC_STM32F103XX_INIT_H_

#include "stm32f103xx.h"

void SPIx_Init(SPI_RegDef_t *pSPIx, uint16_t SPI_BaudRate);
uint8_t SPI_Transfer(SPI_RegDef_t *pSPIx,uint8_t data);
void USART_Transmit(USART_RegDef_t *pUSARTx,uint8_t data);
uint8_t USART_Reception(USART_RegDef_t *pUSARTx);

#endif /* INC_STM32F103XX_INIT_H_ */
