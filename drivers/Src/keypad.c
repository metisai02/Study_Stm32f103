/*
 * keypad.c
 *
 *  Created on: Dec 11, 2022
 *      Author: Metisai_02
 */


#include "keypad.h"



static char keyPad[4][4] = {{'1', '2', '3', 'A'},
							{'4', '5', '6', 'B'},
							{'7', '8', '9', 'C'},
							{'*', '0', '#', 'D'}};

uint8_t tableColum[4] = {COL_PIN_1, COL_PIN_2, COL_PIN_3, COL_PIN_4};
uint8_t tableRow[4] = {ROW_PIN_1, ROW_PIN_2, ROW_PIN_3, ROW_PIN_4};

void KEYPAD_Inits(void){
	GPIO_Handle_t RowPins;
	GPIO_Handle_t ColPins;
	RowPins.pGPIOx = KEYPAD_PORT;
	RowPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_IN;
	RowPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_IN_PUPD;
	RowPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;


	RowPins.GPIO_PinConfig.GPIO_PinNumber = ROW_PIN_1;
	GPIO_Init(&RowPins);

	RowPins.GPIO_PinConfig.GPIO_PinNumber = ROW_PIN_2;
	GPIO_Init(&RowPins);

	RowPins.GPIO_PinConfig.GPIO_PinNumber = ROW_PIN_3;
	GPIO_Init(&RowPins);

	RowPins.GPIO_PinConfig.GPIO_PinNumber = ROW_PIN_4;
	GPIO_Init(&RowPins);

	ColPins.pGPIOx = KEYPAD_PORT;
	ColPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_10MHZ;
	ColPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_GEOUT_PP;

	ColPins.GPIO_PinConfig.GPIO_PinNumber = COL_PIN_1;
	GPIO_Init(&ColPins);

	ColPins.GPIO_PinConfig.GPIO_PinNumber = COL_PIN_2;
	GPIO_Init(&ColPins);

	ColPins.GPIO_PinConfig.GPIO_PinNumber = COL_PIN_3;
	GPIO_Init(&ColPins);

	ColPins.GPIO_PinConfig.GPIO_PinNumber = COL_PIN_4;
	GPIO_Init(&ColPins);

	for(uint8_t colum = 0; colum < 4; colum++){
			GPIO_WriteToOutputPin(KEYPAD_PORT, tableColum[colum], GPIO_PIN_SET);
		}

}

char getChar(void){

	for(uint8_t colum = 0; colum < 4; colum++){
		GPIO_WriteToOutputPin(KEYPAD_PORT, tableColum[colum], GPIO_PIN_RESET);
		for(uint8_t row = 0; row < 4; row++){
			if(!GPIO_ReadFromInputPin(KEYPAD_PORT, tableRow[row])){
				Delayms(100);
				while(!GPIO_ReadFromInputPin(KEYPAD_PORT, tableRow[row])){};
					return keyPad[row][colum];
			}
		}
		GPIO_WriteToOutputPin(KEYPAD_PORT, tableColum[colum], GPIO_PIN_SET);
	}
	return 0;

}
