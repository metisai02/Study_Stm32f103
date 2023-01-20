/*
 * SPItoArduino.c
 *
 *  Created on: Oct 23, 2022
 *      Author: Metisai_02
 */

#include"stm32f103xx_spi_driver.h"
#include<string.h>
/*
 * PB12 -> SPI_NSS
 * PB13 -> SPI_SCLK
 * PB14 -> SPI_MISO
 * PB15 -> SPI_MOSI
 */

void delay(void)
{
	for(uint32_t i = 0; i < 100000/2 ;i++ );
}
void SPI_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_ALOUT_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_50MHZ;
	// cấu hình NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);

	// cấu hình SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	// cấu hình MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);


}
void GPIO_BottonInits(void)
{
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_IN_PUPD;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_Init(&GpioBtn);

}
void SPI2_Inits(void)
{
	SPI_Handle_t  SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handle);
}

int main(void)
{
	char user_data[] = "Hello world";
	// init GPIO nút nhấn
//	GPIO_BottonInits();
	// cấu hình GPIO cho SPI
	SPI_GPIOInits();

	// init SPI SMM = 0 ( hardware )
	SPI2_Inits();
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//while( (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)) ){};

			delay();
			SPI_PeriControl(SPI2, ENABLE);
			uint8_t dataLen = strlen(user_data);
			SPI_SendData(SPI2, &dataLen, 1);
			SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));



	}


	return 0;
}
