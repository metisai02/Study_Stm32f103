/*
 * stm32f103xx_crc_driver.h
 *
 *  Created on: Nov 8, 2022
 *      Author: Metisai_02
 */

#ifndef INC_STM32F103XX_CRC_DRIVER_H_
#define INC_STM32F103XX_CRC_DRIVER_H_

#include <stm32f103xx.h>

void CRC_ResetDR(void);
uint32_t CRC_CalcCRC(uint32_t Data);
uint32_t CRC_CalcBlockCRC(uint32_t pBuffer[], uint32_t BufferLength);
uint32_t CRC_GetCRC(void);
void CRC_SetIDRegister(uint8_t IDValue);
uint8_t CRC_GetIDRegister(void);



#endif /* INC_STM32F103XX_CRC_DRIVER_H_ */
