/*
 * SPItest.c
 *
 *  Created on: Oct 17, 2022
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
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2handle);
}

int main(void)
{
	char user_data[] = "Hello world";
	SPI_GPIOInits();

	SPI2_Inits();

	SPI_SSIConfig(SPI2, ENABLE);
	SPI_PeriControl(SPI2, ENABLE);

	SPI_SEND(SPI2, (uint8_t*)user_data, strlen(user_data));


	return 0;
}
