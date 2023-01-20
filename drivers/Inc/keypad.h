/*
 * keypad.h
 *
 *  Created on: Dec 11, 2022
 *      Author: Metisai_02
 */

#include "stm32f103xx.h"
#include "stm32f103xx_i2c_lcd_driver.h"

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

#define KEYPAD_PORT	GPIOA

#define	ROW_PIN_1  	GPIO_PIN_0
#define	ROW_PIN_2  	GPIO_PIN_1
#define	ROW_PIN_3 	GPIO_PIN_11
#define	ROW_PIN_4  	GPIO_PIN_12

#define	COL_PIN_1	GPIO_PIN_4
#define	COL_PIN_2 	GPIO_PIN_5
#define	COL_PIN_3 	GPIO_PIN_6
#define	COL_PIN_4  	GPIO_PIN_7

void KEYPAD_Inits(void);
char getChar(void);

#endif /* INC_KEYPAD_H_ */
