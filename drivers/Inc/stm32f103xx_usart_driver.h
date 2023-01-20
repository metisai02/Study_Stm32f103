/*
 * stm32f103xx_usart_driver.h
 *
 *  Created on: Nov 1, 2022
 *      Author: Metisai_02
 */

#ifndef INC_STM32F103XX_USART_DRIVER_H_
#define INC_STM32F103XX_USART_DRIVER_H_


#include <stm32f103xx.h>
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */
#define HAL_MAX_DELAY      0xFFFFFFFFU
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define __HAL_UART_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->pUSARTx->SR & (__FLAG__)) == (__FLAG__))
//#define USART_STATE_READY				0
//#define USART_STATE_BUSY_IN_RX			1
//#define USART_STATE_BUSY_IN_TX			2


#define USART_EVENT_TX_CMPLT			1
#define USART_EVENT_RX_CMPLT			2
//#define USART_EVENT_OVR_ORE			3
typedef enum
{
  HRESET = 0,
  HSET = !HRESET
} FlagStatus, ITStatus;
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;
typedef enum
{
  USART_STATE_RESET             		= 0x00U,    /*!< Peripheral is not yet Initialized
                                                   Value is allowed for gState and RxState */
  USART_STATE_READY		             = 0x20U,    /*!< Peripheral Initialized and ready for use
                                                   Value is allowed for gState and RxState */
  USART_STATE_BUSY              		= 0x24U,    /*!< an internal process is ongoing
                                                   Value is allowed for gState only */
  USART_STATE_BUSY_IN_TX           = 0x21U,    /*!< Data Transmission process is ongoing
                                                   Value is allowed for gState only */
  USART_STATE_BUSY_IN_RX           = 0x22U,    /*!< Data Reception process is ongoing
                                                   Value is allowed for RxState only */
  USART_STATE_BUSY_TX_RX        		= 0x23U,    /*!< Data Transmission and Reception process is ongoing
                                                   Not to be used for neither gState nor RxState.
                                                   Value is result of combination (Or) between gState and RxState values */
  USART_STATE_TIMEOUT          		 = 0xA0U,    /*!< Timeout state
                                                   Value is allowed for gState only */
  USART_STATE_ERROR             = 0xE0U     /*!< Error
                                                   Value is allowed for gState only */
} UART_StateTypeDef_t;
typedef struct
{
	uint8_t	 Mode;
	uint32_t Baudrate;
	uint8_t	 NuOfStopBits;
	uint8_t	 WordLength;
	uint8_t	 ParityControl;
//	uint8_t	HWFlowControl;

}USART_Config_t;

typedef struct
{
	USART_RegDef_t 				*pUSARTx;
	USART_Config_t				USARTConfig;
	uint8_t						*pTxBuffer;
	uint8_t						*pRxBuffer;
	uint8_t						TxLen;
	uint8_t						RxLen;
	UART_StateTypeDef_t			gState;
	UART_StateTypeDef_t			RxState;
	//hal
	__IO uint32_t                 ErrorCode;
	uint16_t                      RxXferSize;
	uint8_t                       *pRxBuffPtr;
	__IO uint16_t                 TxXferCount;
	__IO uint16_t                 RxXferCount;
	uint16_t                      TxXferSize;
	uint8_t                       *pTxBuffPtr;


}USART_Handle_t;

/*
 * USART mode
 */
#define USART_MODE_ONLY_TX 	0
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_TXRX  	2
/*
 * USART PARITY -
 */

#define USART_PARITY_EVEN	0
#define USART_PARITY_ODD	1
#define USART_PARITY_DI		2
/*
 * USART WORD LENGTH  ( DATA BITS)
 */

#define USART_WORD_LENGTH_8		0
#define USART_WORD_LENGTH_9		1
/*
 * USART STOP BITS
 */

#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 * USART CPHA CLOCK
 */
#define USART_CPHA_LOW	0
#define USART_CPHA_HIGH	1
/*
 * USART CPOL CLOCK
 */
#define USART_CPOL_LOW	0
#define USART_CPOL_HIGH	1

/*
 *
 */
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
// HAL


void USART_PeriClockControl(USART_RegDef_t *pUSARTx,bool EnorDi);

/*
 * Init and De-init
 */
uint8_t USART_Init(USART_Handle_t *pUSARThandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);
/*
 * Data Send and receive
 */

uint8_t USART_SendData(USART_Handle_t *pUSARTHandle ,uint8_t *pTxBuffer,uint32_t len);
uint8_t USART_ReceiveData(USART_Handle_t *pUSARTHandle ,uint8_t *pRxBuffer,uint32_t len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle ,uint8_t *pTxBuffer,uint32_t len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle ,uint8_t *pRxBuffer,uint32_t len);
/*
 * IQR configure and ISR Handle
 */
void USART_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);


uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
/*
 *
 */

void USART_PeriControl(USART_RegDef_t *pUSARTx,bool EnorDi);
__attribute__((weak))void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t ApEv);
//hal
HAL_StatusTypeDef HAL_UART_Transmit(USART_Handle_t *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(USART_Handle_t *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
#endif /* INC_STM32F103XX_USART_DRIVER_H_ */
