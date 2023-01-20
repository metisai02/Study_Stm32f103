/*
 * vantay.h
 *
 *  Created on: Nov 16, 2022
 *      Author: Metisai_02
 */

#ifndef INC_VANTAY_H_
#define INC_VANTAY_H_

#include "stm32f103xx.h"
#include "stm32f103xx_init.h"
#include "stm32f103xx_i2c_lcd_driver.h"
// cờ báo cho vân tay
 #define FP_OK 0x00
 #define FP_ERROR 0xFE
 #define FP_NOFINGER 0x02
 #define FP_FINGER_NOTMATCH 0x0A
 #define FP_FINGER_NOTMATCH 0x0A
 #define FP_FINGER_NOTFOUND 0x09

 #define MP3COMMANDLEN 24
//đinh nghia khối lệnh cho vân tay

// Thông báo cho vân tay





		uint8_t CheckFPRespsone(USART_Handle_t usart2, uint8_t MaxRead);
		uint8_t GetNumberOfFinger(USART_Handle_t usart2);
		uint8_t RegistryNewFinger(USART_Handle_t usart2, uint16_t LocationID,I2C_LCD_Handle_t LCD);
		uint8_t CheckFinger(USART_Handle_t usart2,I2C_LCD_Handle_t LCD);
		uint8_t deleteallfinger(USART_Handle_t usart2);
		uint8_t Delete_ID(USART_Handle_t usart2, uint16_t ID);


#endif /* INC_VANTAY_H_ */
