/*
 * stm32f103xx_init.c
 *
 *  Created on: Nov 15, 2022
 *      Author: Metisai_02
 */


#include <stm32f103xx_init.h>

void SPIx_Init(SPI_RegDef_t *pSPIx, uint16_t SPI_BaudRate)
{
	GPIO_Handle_t SPI_Pins;
	// Enable clock GPIO

	if(pSPIx == SPI1 )
	{

		SPI_Pins.pGPIOx = GPIOB;
		SPI_Pins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_ALOUT_PP;
		SPI_Pins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_50MHZ;
		// cấu hình NSS
		SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
		GPIO_Init(&SPI_Pins);

		// cấu hình SCLK
		SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
		GPIO_Init(&SPI_Pins);

		// cấu hình MOSI
		SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
		GPIO_Init(&SPI_Pins);															// Enable clock SPI1
	}
	else if(pSPIx == SPI2)
	{

		SPI_Pins.pGPIOx = GPIOB;
		SPI_Pins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_ALOUT_PP;
		SPI_Pins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_50MHZ;
		// cấu hình Ncj
		SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
		GPIO_Init(&SPI_Pins);

		// cấu hình miso
//		SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
//		GPIO_Init(&SPI_Pins);

		// cấu hình MOSI
		SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
		GPIO_Init(&SPI_Pins);

		SPI_Pins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_IN;
		SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
		SPI_Pins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_IN_FLOAT;
		GPIO_Init(&SPI_Pins);

														// Enable clock SPI2 or SPI3
	}

	SPI_Handle_t SPI_Inits;

	SPI_Inits.pSPIx = pSPIx;
	SPI_Inits.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI_Inits.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI_Inits.SPIConfig.SPI_SclkSpeed = SPI_BaudRate;
	SPI_Inits.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI_Inits.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI_Inits.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI_Inits.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI_Inits);
	SPI_SSIConfig(SPI2, ENABLE);
}


uint8_t SPI_Transfer(SPI_RegDef_t *pSPIx,uint8_t data)
{
	uint8_t TxBuffer = data;
	uint8_t RxBuffer;

	SPI_SendData(pSPIx, &TxBuffer, 1);

	SPI_ReceiveData(pSPIx, &RxBuffer, 1);
	return RxBuffer;
}
//void USART_Transmit(USART_RegDef_t *pUSARTx,uint8_t data)
//{
//	uint8_t TxBuffer = data;
//
//	USART_SendData(pUSARTx, &TxBuffer, 1);
//
//}
//uint8_t USART_Reception(USART_RegDef_t *pUSARTx)
//{
//	uint8_t RxBuffer;
//
//	USART_ReceiveData(pUSARTx, &RxBuffer, 1);
//	return RxBuffer;
//
//}
