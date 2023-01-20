/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Oct 16, 2022
 *      Author: Metisai_02
 */

#include <stm32f103xx_spi_driver.h>

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/******************************************************************
 * Tên hàm:	SPI_PeriClockControl
 * Chức năng: Cấu hình xung clock cho SPI
 * Input:*pSPIx-> địa chỉ của SPI , EnorDi-> Enable hay Diable
 * Trả về: Không
 *******************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,bool EnorDi);
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,bool EnorDi)
{
	if (EnorDi) {
	        if (pSPIx == SPI1) SPI1_PCLK_EN();
	        else if (pSPIx == SPI2) SPI2_PCLK_EN();
	        else if (pSPIx == SPI3) SPI3_PCLK_EN();
	    } else
	    {
	    	if (pSPIx == SPI1) SPI1_PCLK_DI();
			else if (pSPIx == SPI2) SPI2_PCLK_DI();
			else if (pSPIx == SPI3) SPI3_PCLK_DI();
	    }
}
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	// Bật sung clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

// đầu tiên cấu hình cho thanh ghi SPI_CR1

	uint32_t tempreg = 0;
	// bổ xung ssi
	tempreg |= (1 << SPI_CR1_SSI);
	//1. cấu hình mode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
	//2. cấu hình DFF
	tempreg |=	(pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);
	//3. cấu hình CPHA
	tempreg	|=	(pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);
	//4. cấu hình CPOL
	tempreg	|=	(pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);
	//5. cấu hình SPEED
	tempreg |= 	(pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);
	//6. cấu hình NSS
	if(pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_EN)
	{
		tempreg	|=	(pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);
	}else
	{
		tempreg	&=	~(pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);
		SPI_SSOEConfig(SPI2, ENABLE);
	}


	//7. cấu hình BUS
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg	&= ~(1 << SPI_CR1_RXONLY);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg	|= (1 << SPI_CR1_RXONLY);
	}

	// nhét hết vào thanh ghi CR1
	pSPIHandle->pSPIx->CR1	|= tempreg;

	//pSPIHandle->pSPIx->CRCPR = pSPIHandle->SPIConfig.SPI_CRCPolynomial;
}
void SPI_DeInit(SPI_RegDef_t *pSPIx);
/******************************************************************
 * Tên hàm:	SPI_GetFlagStatus
 * Chức năng: kiểm tra cờ trong thanh ghi SR
 * Input:	*pSPIx-> địa chỉ của SPI loại (SPI1,SPI2,SPI3)
 * 			FlagName ->	tên của cờ
 * Trả về: 	FLAG_SET => cờ lên 1
 * 			FLAG_RESET => cờ xuống 0
 *******************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName )
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/******************************************************************
 * Tên hàm:	SPI_PeriControl
 * Chức năng: Mở cho SPI bắt đầu chạy (cấu hình chân SPE )
 * Input:	*pSPIx-> địa chỉ của SPI loại (SPI1,SPI2,SPI3)
 * 			EnorDi -> enable hay disable
 * Trả về: Không
 *******************************************************************/
void SPI_PeriControl(SPI_RegDef_t *pSPIx,bool EnorDi)
{
	if(EnorDi == 1)
	{
		pSPIx->CR1 |= (1<< SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}
/******************************************************************
 * Tên hàm:	SPI_SSOEConfig
 * Chức năng: cấu hình SSI ( khi dùng NSS software)
 * Input:	*pSPIx-> địa chỉ của SPI loại (SPI1,SPI2,SPI3)
 * 			EnorDi -> enable hay disable
 * Trả về: Không
 *******************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,bool EnorDi)
{
	if(EnorDi == 1)
	{
		pSPIx->CR1 |= (1<< SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}
/******************************************************************
 * Tên hàm:	SPI_SSOEConfig
 * Chức năng: cấu hình SSOE ( khi dùng NSS hardware)
 * Input:	*pSPIx-> địa chỉ của SPI loại (SPI1,SPI2,SPI3)
 * 			EnorDi -> enable hay disable
 * Trả về: Không
 *******************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,bool EnorDi)
{
	if(EnorDi == 1)
	{
		pSPIx->CR2 |= (1<< SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
	}
}
/******************************************************************
 * Tên hàm:	SPI_SendData
 * Chức năng: Gửi data
 * Input:	*pSPIx-> địa chỉ của SPI loại (SPI1,SPI2,SPI3)
 * 			pTxBuffer -> địa chỉ của Txbuffer
 * 			len -> độ dài của dữ liệu
 * Trả về: Không
 *******************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx ,uint8_t *pTxBuffer,uint32_t len)
{
	SPI_PeriControl(SPI2, ENABLE);
	while(len > 0)
	{
		// 1. chờ cờ TXE set (chờ cho thanh txbuffer trống)
		while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET);
		//2. kiểm tra DFF

		if(((pSPIx->CR1 >> SPI_CR1_DFF) & 1) == SPI_DFF_16BIT)
		{
			// 16 bit
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			(uint16_t*) pTxBuffer++;
			len--;

		}else
		{
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
		}

		 len--;
	}
	while( SPI_GetFlagStatus(SPI2, SPI_FLAG_BUSY) );
	//SPI_PeriControl(SPI2, DISABLE);
}
/******************************************************************
 * Tên hàm:	SPI_ReceiveData
 * Chức năng: Gửi  data
 * Input:	*pSPIx-> địa chỉ của SPI loại (SPI1,SPI2,SPI3)
 * 			pRxBuffer ->địa chỉ của RX buffer
 * 			len -> độ dài của dữ liệu
 * Trả về: Không
 *******************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx ,uint8_t *pRxBuffer,uint32_t len)
{
	while(len > 0)
	{
		// 1. chờ cờ RXXE set ( chờ cho thanh rxbuffer đầy)
		while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == FLAG_RESET);
		//2. kiểm tra DFF

		if(((pSPIx->CR1 >> SPI_CR1_DFF) & 1) == SPI_DFF_16BIT)
		{
			// 16 bit
			*((uint16_t *) pRxBuffer) = pSPIx->DR ;
			len--;
			(uint16_t *) pRxBuffer++;

		}else
		{
			 *pRxBuffer = pSPIx->DR ;
			 pRxBuffer++;
		}

		 len--;
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle ,uint8_t *pTxBuffer,uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_STATE_BUSY_IN_TX)
	{
		//1. Lưu địa chỉ Tx Bufer và chiều dài vào biến toàn cục
			pSPIHandle->pTxBuffer = pTxBuffer;
			pSPIHandle->TxLen = len;

			//2. Đánh dấu là SPI đang bận trong quá tình truyền
			pSPIHandle->TxState = SPI_STATE_BUSY_IN_TX;
			//3. Set bit TXEIE để nhận Interrupt bất cứ khi nào nào cờ TXE set
			pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
			//4. dữ
	}
	return state;

}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle ,uint8_t *pRxBuffer,uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_STATE_BUSY_IN_RX)
	{
		//1. Lưu địa chỉ Tx Bufer và chiều dài vào biến toàn cục
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = len;

			//2. Đánh dấu là SPI đang bận trong quá tình truyền
			pSPIHandle->RxState = SPI_STATE_BUSY_IN_RX;
			//3. Set bit RXNEIE để nhận Interrupt bất cứ khi nào nào cờ TXE set
			pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	}
	return state;
}

void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    uint8_t regx = IRQNumber / 32;
    if (EnorDi == ENABLE)
        NVIC_ISER->REG_NU[regx] |= (1 << (IRQNumber % 32));
    else
        NVIC_ICER->REG_NUM[regx] |= (1 << (IRQNumber % 32));
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//kiểm tra TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<< SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// kiểm tra RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<< SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// kiểm tra Over flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1<< SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(((pSPIHandle->pSPIx->CR1 >> SPI_CR1_DFF) & 1) == SPI_DFF_16BIT)
	{
		// 16 bit
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer++);
		pSPIHandle->TxLen--;

	}else pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer++);

	pSPIHandle->TxLen--;

	if(!pSPIHandle ->TxLen)
	{
		//Txlen là 0 , thì sẽ đóng giao tiếp và thông báo TX đã xong

		//
		pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_STATE_READY;
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(((pSPIHandle->pSPIx->CR1 >> SPI_CR1_DFF) & 1) == SPI_DFF_16BIT)
			{
				// 16 bit
				*((uint16_t *)pSPIHandle-> pRxBuffer) = pSPIHandle->pSPIx->DR ;
				pSPIHandle->RxLen--;
				(uint16_t *) pSPIHandle->pRxBuffer++;

			}else
			{
				 *pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR ;
				 pSPIHandle->pRxBuffer++;
			}

			pSPIHandle->RxLen--;

			if(!pSPIHandle ->RxLen)
			{
				//Rxlen là 0 , thì sẽ đóng giao tiếp và thông báo RX đã xong

				//
				pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
				pSPIHandle->pRxBuffer = NULL;
				pSPIHandle->RxLen = 0;
				pSPIHandle->RxState = SPI_STATE_READY;
				SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
			}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

}
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

}
