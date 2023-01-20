/*
 * stm32f103xx.gpio_driver.h
 *
 *  Created on: Oct 6, 2022
 *      Author: Metisai_02
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"
#include <stdbool.h>


/*
 *Chỗ này là cấu hình cho GPOI - PIN
  */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t	GPIO_PinModeAndSpeed;
	uint8_t GPIO_PinPuPdControl	;		/* kéo trở lên or kéo trở xuống */
	uint8_t GPIO_CRF;					/* cấu hình type cho output và input */
//	 uint8_t GPIO_PinAltFunMode;			/* Chức năng đặt biệt kiểu SPI or USART*/
	uint8_t GPIO_PinIRQ_Trigger;			/*cấu hình ngắt cạnh xuống hya lên */
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t		*pGPIOx;	/*	cái này giữa những địa chỉ thanh ghi GPIO  */
	GPIO_PinConfig_t	GPIO_PinConfig;/*cái này cài đặt cấu hình cho GPIO_PIN */
}GPIO_Handle_t;

/*
 * GPIO PIN NUMBER
 */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*
 * GPIO pin possible Modes and Speed
 */
#define GPIO_MODE_IN 				0b00

#define GPIO_MODE_OUT_SPEED_10MHZ	0b01
#define GPIO_MODE_OUT_SPEED_2MHZ	0b10
#define GPIO_MODE_OUT_SPEED_50MHZ	0b11


/*
 * CRF_resgister (GPIO pin possible output Type and input type)
 */
#define GPIO_CNF_GEOUT_PP			0b00
#define GPIO_CNF_GEOUT_OD			0b01
#define GPIO_CNF_ALOUT_PP			0b10
#define GPIO_CNF_ALOUT_OD			0b11


#define GPIO_CNF_IN_ANALOG			0b00
#define GPIO_CNF_IN_FLOAT			0b01
#define GPIO_CNF_IN_PUPD			0b10
#define GPIO_CNF_IN_RESERVED 		0b11

#define GPIO_PU						1
#define GPIO_PD						0


/*
 * GPIO PIN IQR
 * */
#define GPIO_IRQ_FT                 0b100 // Interrupt trigger falling edge
#define GPIO_IRQ_RT                 0b010 // Interrupt trigger rising edge
#define GPIO_IRQ_RFT                0b110 // Interrupt trigger falling and rising edge


/*******************************************************************************************************
 * 									APIs supported by this driver
 * 		For more information about the APIs to check the function definitions
*******************************************************************************************************/
/*
 * 	Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,bool EnorDi);


/*
 * 	Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * 	Read and Write data
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * 	IRQ configuration and IRQ Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);





#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
