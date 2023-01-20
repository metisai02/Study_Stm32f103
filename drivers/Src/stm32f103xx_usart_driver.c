/*
 * stm32f103xx_usart_driver.c
 *
 *  Created on: Nov 1, 2022
 *      Author: Metisai_02
 */

#include <stm32f103xx_usart_driver.h>

/******************************************************************
 * Tên hàm:	USART_PeriClockControl
 * Chức năng: Cấu hình xung clock cho USART
 * Input:*pUSARTx-> địa chỉ của SPI , EnorDi-> Enable hay Diable
 * Trả về: Không
 *******************************************************************/
static HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(USART_Handle_t *huart, uint32_t Flag, FlagStatus Status, uint32_t Tickstart, uint32_t Timeout);
void USART_PeriClockControl(USART_RegDef_t *pUSART,bool EnorDi)
{
	if (EnorDi) {
	        if (pUSART == USART1) USART1_PCLK_EN();
	        else if (pUSART == USART2) USART2_PCLK_EN();
	        else if (pUSART == USART3) USART3_PCLK_EN();
	    } else {
	    	if (pUSART == USART1) USART1_PCLK_DI();
			else if (pUSART == USART2) USART2_PCLK_DI();
			else if (pUSART == USART3) USART3_PCLK_DI();
	    }
}

void USART_BaudRateConfigure(USART_RegDef_t *pUSARTx , uint32_t BaudRate)
{
	uint32_t PCLKx;
	uint32_t usartdiv;

	uint32_t Mantissa_part;
	uint32_t Fraction_part;

	uint32_t tempreg = 0;

	if(pUSARTx == USART1 )
	{
		//APB2
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	usartdiv = (PCLKx * 100) / (16 * BaudRate);

	Mantissa_part = usartdiv/100;
	tempreg |= 	(Mantissa_part << 4);

	Fraction_part = ((usartdiv - Mantissa_part * 100)*16 +50) /100;

	tempreg |= Fraction_part;

	pUSARTx->BRR = tempreg;
}
/*
 *
 */
void USART_PeriControl(USART_RegDef_t *pUSARTx,bool EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
}

/*
 * Init and De-init
 */

uint8_t USART_Init(USART_Handle_t *pUSARTHandle)
{
	if(pUSARTHandle == NULL)
	{
		return HAL_ERROR;
	}
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);
	uint32_t tempregcr1 = 0;
	uint32_t tempregcr2 = 0;
//	uint32_t tempregcr3 = 0

	//1. cấu hình MODE
		if(pUSARTHandle->USARTConfig.Mode == USART_MODE_ONLY_TX)
		{
			tempregcr1 |= (1 << USART_CR1_TE);
		}else if(pUSARTHandle->USARTConfig.Mode == USART_MODE_ONLY_TX)
		{
			tempregcr1 |= (1 << USART_CR1_RE);
		}else if (pUSARTHandle->USARTConfig.Mode == USART_MODE_TXRX)
		{
			tempregcr1 |= (1 << USART_CR1_TE);
			tempregcr1 |= (1 << USART_CR1_RE);
		}

	//2. cấu hình độ dài bit 8 or 9 (9 bit mà không phải parity thì là truyền 1 byte and 1 bit )
			tempregcr1 |= (pUSARTHandle->USARTConfig.WordLength << USART_CR1_M);
	//3. cấu hình bit parity ( chẵn or lẻ )
			if(pUSARTHandle->USARTConfig.ParityControl == USART_PARITY_EVEN)
			{
				tempregcr1 |= (1 << USART_CR1_PCE);
				tempregcr1 &= ~(1 <<USART_CR1_PS);
			}else if (pUSARTHandle->USARTConfig.ParityControl == USART_PARITY_ODD)
			{
				tempregcr1 |= (1 << USART_CR1_PCE);
				tempregcr1 |= (1 <<USART_CR1_PS);
			}else if (pUSARTHandle->USARTConfig.ParityControl == USART_PARITY_DI)
			{
				tempregcr1 &= ~(1 << USART_CR1_PCE);
			}

	//4. cấu hình STOP bits
		tempregcr2 |= (pUSARTHandle->USARTConfig.NuOfStopBits << USART_CR2_STOP);




	//5. cấu hình tốc độ Baud
		USART_BaudRateConfigure(pUSARTHandle->pUSARTx, pUSARTHandle->USARTConfig.Baudrate);
	//6. cấu hình USART hardware flow control

		pUSARTHandle->pUSARTx->CR1 = tempregcr1;
		pUSARTHandle->pUSARTx->CR2 = tempregcr2;
//		pUSARTHandle->pUSARTx->CR3 = tempregcr3
//		USART_PeriControl(pUSARTHandle->pUSARTx, ENABLE);
		pUSARTHandle->gState = USART_STATE_READY;
		pUSARTHandle->RxState = USART_STATE_READY;

		return HAL_OK;
}
void USART_DeInit(USART_RegDef_t *pUSARTx);
/******************************************************************
 * Tên hàm:	USART_GetFlagStatus
 * Chức năng: kiểm tra cờ trong thanh ghi SR
 * Input:	*pUSARTx-> địa chỉ của USART loại (USART1,USART2,USART3)
 * 			FlagName ->	tên của cờ
 * Trả về: 	FLAG_SET => cờ lên 1
 * 			FLAG_RESET => cờ xuống 0
 *******************************************************************/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if(pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}
/******************************************************************
 * Tên hàm:	USART_ClearFlagStatus
 * Chức năng: XÓA cờ trong thanh ghi SR
 * Input:	*pUSARTx-> địa chỉ của USART loại (USART1,USART2,USART3)
 * 			FlagName ->	tên của cờ
 * Trả về: 	không
 *******************************************************************/
void USART_ClearFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	pUSARTx->SR &= ~FlagName ;
}
/******************************************************************
 * Tên hàm:	USART_SendData
 * Chức năng: Gửi data
 * Input:	*pUSARTHandle -> Truyền vào APIs và pUSART
 * 			pTxBuffer -> địa chỉ của Txbuffer
 * 			len -> độ dài của dữ liệu
 * Trả về: Không
 *******************************************************************/

uint8_t USART_SendData(USART_Handle_t *pUSARTHandle ,uint8_t *pTxBuffer,uint32_t len)
{
	if(pUSARTHandle->gState == USART_STATE_READY)
	{
		if((pTxBuffer == NULL)||len == 0u)
		{
			return HAL_ERROR;
		}
		USART_PeriControl(pUSARTHandle->pUSARTx, ENABLE);
		pUSARTHandle->gState = USART_STATE_BUSY_IN_TX;
		for(uint8_t i = 0; i < len; i++)
		{
			// 1. chờ cờ TXE set ( chờ cho thanh txbuffer trống )
			while (USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE) == FLAG_RESET);
			//
			if( pUSARTHandle->USARTConfig.WordLength == USART_WORD_LENGTH_9)
			{
				// 9 bit thì mình ép kiểu nó về 16bit rồi and với 0x1FF để sài 9bit
				pUSARTHandle->pUSARTx->DR = ((*(uint16_t*)pTxBuffer) & (uint16_t)0x1FF);

				if(pUSARTHandle->USARTConfig.ParityControl == USART_PARITY_DI)
				{
					pTxBuffer++;
					pTxBuffer++;
				}else
				{
					pTxBuffer++;
				}
			}else
			{
				pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
				pTxBuffer++;
			}
		}
		// chờ cờ completely transmit  bật
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
		pUSARTHandle->gState = USART_STATE_READY;
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}
/******************************************************************
 * Tên hàm:	USART_ReceiveData
 * Chức năng: Nhận data
 * Input:	*pUSARTHandle -> Truyền vào APIs và pUSART
 * 			pRxBuffer -> địa chỉ của Rxbuffer
 * 			len -> độ dài của dữ liệu
 * Trả về: Không
 *******************************************************************/
uint8_t USART_ReceiveData(USART_Handle_t *pUSARTHandle ,uint8_t *pRxBuffer,uint32_t len)
{
	if(pUSARTHandle->RxState == USART_STATE_READY)
	{
		if((pRxBuffer == NULL) || len == 0u)
		{
			return HAL_ERROR;
		}
		USART_PeriControl(pUSARTHandle->pUSARTx, ENABLE);
		pUSARTHandle->RxState =USART_STATE_BUSY_IN_RX;
		for( uint8_t i = 0; i < len; i++)
		{
			while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));
			if(pUSARTHandle->USARTConfig.WordLength == USART_WORD_LENGTH_9)
			{

				if(pUSARTHandle->USARTConfig.ParityControl == USART_PARITY_DI)
				{
					*((uint16_t*)pRxBuffer) = pUSARTHandle->pUSARTx->DR & (uint16_t)0x1FF;
					pRxBuffer++;
					pRxBuffer++;
				}else
				{
					*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					pRxBuffer++;
				}
			}else
			{
				if(pUSARTHandle->USARTConfig.ParityControl == USART_PARITY_DI)
				{
					// truyền với 8 bit Data
					*pRxBuffer = (uint8_t)pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF;
				}else
				{
					// truyền với 7bit data và 1 bit partity
					*pRxBuffer = (uint8_t)pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F;
				}
				pRxBuffer++;
			}
			pUSARTHandle->RxState = USART_STATE_READY;
			return HAL_OK;
		}
	}
	return HAL_BUSY;


}
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle ,uint8_t *pTxBuffer,uint32_t len)
{
	uint8_t state = pUSARTHandle->gState;

	if(state != USART_STATE_BUSY_IN_TX)
	{
		//1. Lưu địa chỉ Tx Bufer và chiều dài vào biến toàn cục
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxLen = len;

			//2. Đánh dấu là SPI đang bận trong quá tình truyền
		pUSARTHandle->gState = USART_STATE_BUSY_IN_TX;
			//3. Set bit TXEIE để nhận Interrupt bất cứ khi nào nào cờ TXE set
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

	}
	return state;
}
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle ,uint8_t *pRxBuffer,uint32_t len)
{
	uint8_t state = pUSARTHandle->RxState;
	USART_PeriControl(pUSARTHandle->pUSARTx, ENABLE);
	if(state != USART_STATE_BUSY_IN_RX)
	{
		//1. Lưu địa chỉ Tx Bufer và chiều dài vào biến toàn cục
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxLen = len;

			//2. Đánh dấu là SPI đang bận trong quá tình truyền
		pUSARTHandle->RxState = USART_STATE_BUSY_IN_RX;
			//3. Set bit TXEIE để nhận Interrupt bất cứ khi nào nào cờ TXE set
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	}
	return state;
}
void USART_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    uint8_t regx = IRQNumber / 32;
    if (EnorDi == ENABLE)
        NVIC_ISER->REG_NU[regx] |= (1 << (IRQNumber % 32));
    else
        NVIC_ICER->REG_NUM[regx] |= (1 << (IRQNumber % 32));
}
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2;

	uint16_t *pdata;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->gState == USART_STATE_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit

				//Reset the application state
				pUSARTHandle->gState = USART_STATE_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->gState == USART_STATE_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USARTConfig.WordLength == USART_WORD_LENGTH_9)
				{
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USARTConfig.ParityControl == USART_PARITY_DI)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		if(pUSARTHandle->RxState == USART_STATE_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USARTConfig.WordLength == USART_WORD_LENGTH_9)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USARTConfig.ParityControl == USART_PARITY_DI)
					{
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen-=1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USARTConfig.ParityControl == USART_PARITY_DI)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					 pUSARTHandle->RxLen-=1;
				}


			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxState = USART_STATE_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
//	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);
//
//	//Implement the code to check the state of CTSE bit in CR1
//	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);
//
//	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
//	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);
//
//
//	if(temp1  && temp2 )
//	{
//		//Implement the code to clear the CTS flag in SR
//		pUSARTHandle->pUSARTx->SR &=  ~( 1 << USART_SR_CTS);
//
//		//this interrupt is because of cts
//		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
//	}

/*************************Check for IDLE detection flag ********************************************/
//
//	//Implement the code to check the status of IDLE flag bit in the SR
//	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);
//
//	//Implement the code to check the state of IDLEIE bit in CR1
//	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);
//
//
//	if(temp1 && temp2)
//	{
//		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
//		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);
//
//		//this interrupt is because of idle
//		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
//	}
//
///*************************Check for Overrun detection flag ********************************************/
//
//	//Implement the code to check the status of ORE flag  in the SR
//	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;
//
//	//Implement the code to check the status of RXNEIE  bit in the CR1
//	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;
//
//
//	if(temp1  && temp2 )
//	{
//		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
//
//		//this interrupt is because of Overrun error
//		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
//	}
//
//
//
///*************************Check for Error Flag ********************************************/
//
////Noise Flag, Overrun error and Framing Error in multibuffer communication
////We dont discuss multibuffer communication in this course. please refer to the RM
////The blow code will get executed in only if multibuffer mode is used.
//
//	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;
//
//	if(temp2 )
//	{
//		temp1 = pUSARTHandle->pUSARTx->SR;
//		if(temp1 & ( 1 << USART_SR_FE))
//		{
//			/*
//				This bit is set by hardware when a de-synchronization, excessive noise or a break character
//				is detected. It is cleared by a software sequence (an read to the USART_SR register
//				followed by a read to the USART_DR register).
//			*/
//			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
//		}
//
//		if(temp1 & ( 1 << USART_SR_NE) )
//		{
//			/*
//				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
//				software sequence (an read to the USART_SR register followed by a read to the
//				USART_DR register).
//			*/
//			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
//		}
//
//		if(temp1 & ( 1 << USART_SR_ORE) )
//		{
//			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
//		}
//	}


}
__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{

}
HAL_StatusTypeDef HAL_UART_Transmit(USART_Handle_t *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint8_t  *pdata8bits;
  uint16_t *pdata16bits;
  uint32_t tickstart = 0U;

  /* Check that a Tx process is not already ongoing */
  if (huart->gState == USART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    /* Process Locked */
  //  __HAL_LOCK(huart);

    huart->ErrorCode = USART_STATE_ERROR;
    huart->gState = USART_STATE_BUSY_IN_TX;

    /* Init tickstart for timeout managment */
    tickstart = GetTick();

    huart->TxXferSize = Size;
    huart->TxXferCount = Size;

    /* In case of 9bits/No Parity transfer, pData needs to be handled as a uint16_t pointer */
    if ((huart->USARTConfig.WordLength == USART_WORD_LENGTH_9) && (huart->USARTConfig.ParityControl == USART_PARITY_DI))
    {
      pdata8bits  = NULL;
      pdata16bits = (uint16_t *) pData;
    }
    else
    {
      pdata8bits  = pData;
      pdata16bits = NULL;
    }

    /* Process Unlocked */
  //  __HAL_UNLOCK(huart);

    while (huart->TxXferCount > 0U)
    {
      if (UART_WaitOnFlagUntilTimeout(huart, USART_FLAG_TXE, RESET, tickstart, Timeout) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }
      if (pdata8bits == NULL)
      {
        huart->pUSARTx->DR = (uint16_t)(*pdata16bits & 0x01FFU);
        pdata16bits++;
      }
      else
      {
        huart->pUSARTx->DR = (uint8_t)(*pdata8bits & 0xFFU);
        pdata8bits++;
      }
      huart->TxXferCount--;
    }

    if (UART_WaitOnFlagUntilTimeout(huart, USART_FLAG_TC, RESET, tickstart, Timeout) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

    /* At end of Tx process, restore huart->gState to Ready */
    huart->gState = USART_STATE_READY;

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
static HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(USART_Handle_t *huart, uint32_t Flag, FlagStatus Status, uint32_t Tickstart, uint32_t Timeout)
{
  /* Wait until flag is set */
  while (USART_GetFlagStatus(huart->pUSARTx, Flag) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((GetTick() - Tickstart) > Timeout))
      {
        /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
     //   CLEAR_BIT(huart->pUSARTx->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_TXEIE));
      //  CLEAR_BIT(huart->pUSARTx->CR3, USART_CR3_EIE);

        huart->gState  = USART_STATE_READY;
        huart->RxState = USART_STATE_READY;

        /* Process Unlocked */
       // __HAL_UNLOCK(huart);

        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive(USART_Handle_t *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint8_t  *pdata8bits;
  uint16_t *pdata16bits;
  uint32_t tickstart = 0U;

  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == USART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      return  HAL_ERROR;
    }

    /* Process Locked */
//    __HAL_LOCK(huart);

    huart->ErrorCode = USART_STATE_ERROR;
    huart->RxState = USART_STATE_BUSY_IN_RX;
 //   huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

    /* Init tickstart for timeout management */
    tickstart = GetTick();

    huart->RxXferSize = Size;
    huart->RxXferCount = Size;

    /* In case of 9bits/No Parity transfer, pRxData needs to be handled as a uint16_t pointer */
    if ((huart->USARTConfig.WordLength == USART_WORD_LENGTH_9) && (huart->USARTConfig.ParityControl == USART_PARITY_DI))
    {
      pdata8bits  = NULL;
      pdata16bits = (uint16_t *) pData;
    }
    else
    {
      pdata8bits  = pData;
      pdata16bits = NULL;
    }

    /* Process Unlocked */
 //   __HAL_UNLOCK(huart);

    /* Check the remain data to be received */
    while (huart->RxXferCount > 0U)
    {
      if (UART_WaitOnFlagUntilTimeout(huart, USART_FLAG_RXNE, RESET, tickstart, Timeout) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }
      if (pdata8bits == NULL)
      {
        *pdata16bits = (uint16_t)(huart->pUSARTx->DR & 0x01FF);
        pdata16bits++;
      }
      else
      {
        if ((huart->USARTConfig.WordLength == USART_WORD_LENGTH_9) || ((huart->USARTConfig.WordLength == USART_WORD_LENGTH_8) && (huart->USARTConfig.ParityControl == USART_PARITY_DI)))
        {
          *pdata8bits = (uint8_t)(huart->pUSARTx->DR & (uint8_t)0x00FF);
        }
        else
        {
          *pdata8bits = (uint8_t)(huart->pUSARTx->DR & (uint8_t)0x007F);
        }
        pdata8bits++;
      }
      huart->RxXferCount--;
    }

    /* At end of Rx process, restore huart->RxState to Ready */
    huart->RxState = USART_STATE_READY;

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}






