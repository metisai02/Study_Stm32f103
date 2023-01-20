/*
 * testUsartFP.c
 *
 *  Created on: Dec 8, 2022
 *      Author: Metisai_02
 */
#include <stdio.h>
#include <string.h>
#include "stm32f103xx.h"
 #define FP_OK 0x00
 #define FP_ERROR 0xFE
 #define FP_NOFINGER 0x02
 #define FP_FINGER_NOTMATCH 0x0A
 #define FP_FINGER_NOTMATCH 0x0A
 #define FP_FINGER_NOTFOUND 0x09

 #define MP3COMMANDLEN 24
    uint8_t FPHeader[6]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF};
	 uint8_t FPGetImage[6]={0x01,0x00,0x03,0x01,0x00,0x05};
	 uint8_t FPCreateCharFile1[7]={0x01,0x00,0x04,0x02,0x01,0x00,0x08};
	 uint8_t FPCreateCharFile2[7]={0x01,0x00,0x04,0x02,0x02,0x00,0x09};
	 uint8_t FPCreateTemplate[6]={0x01,0x00,0x03,0x05,0x00,0x09};
	 uint8_t FPDeleteAllFinger[6]={0x01,0x00,0x03,0x0D,0x00,0x11};
	 uint8_t FPSearchFinger[11]={0x01,0x00,0x08,0x04,0x01,0x00,0x00,0x00,0x40,0x00,0x4E};
	 uint8_t FPGetNumberOfFinger[6]={0x01,0x00,0x03,0x1D,0x00,0x21};
	 // MP3 Instruction

	 char* RegistryFingger = "Registry New Finger   \r\n";
	 char* PutFinger =       "Please put Finger In  \r\n";
	 char* RemoveFinger =    "Please Remove Fingger  \r\n";
	 char* ReputFinger =     "Put Finger agian      \r\n";
	 char* FingerNotMatch =  "Finger not match      \r\n";
	 char* FingerError =     "Finger Error          \r\n";
	 char* CheckingFinger =  "Checking Finger       \r\n";
	 char* FingerNotFound =  "Finger Not Found      \r\n";
	 char* FingerFound =     "Finger Found  !!!!    \r\n";
	 char* RegistryDone =    "Registry Finger Done  \r\n";
   uint8_t IDFromFinger;


	 uint8_t CurrentNumberFinger;
USART_Handle_t Usart1;
void USART_GPIOInits(void)
{
	GPIO_Handle_t USARTPins;
	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_ALOUT_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_50MHZ;
	// TX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&USARTPins);

	//  RX
	USARTPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_IN;
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	USARTPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_IN_FLOAT;
	GPIO_Init(&USARTPins);




}
void USART_Inits(void)
{


	Usart1.pUSARTx = USART1;
	Usart1.USARTConfig.Mode = USART_MODE_TXRX;
	Usart1.USARTConfig.WordLength = USART_WORD_LENGTH_8;
	Usart1.USARTConfig.NuOfStopBits = USART_STOPBITS_1;
	Usart1.USARTConfig.Baudrate = USART_STD_BAUD_57600;
	Usart1.USARTConfig.ParityControl = USART_PARITY_DI;

	USART_Init(&Usart1);
}

void SendFPHeader()
{
	HAL_UART_Transmit(&Usart1,FPHeader,6,1000);
}

void SendFPGetImage()
{
	 HAL_UART_Transmit(&Usart1,FPGetImage,6,1000);
}

void SendFPCreateCharFile1()
{
	HAL_UART_Transmit(&Usart1,FPCreateCharFile1,7,1000);
}

void SendFPCreateCharFile2()
{
	HAL_UART_Transmit(&Usart1,FPCreateCharFile2,7,1000);
}

void SendFPCreateTemplate()
{
	HAL_UART_Transmit(&Usart1,FPCreateTemplate,6,1000);
}

void SendFPDeleteAllFinger()
{
	HAL_UART_Transmit(&Usart1,FPDeleteAllFinger,6,1000);
}

void SendFPDSearchFinger()
{
	HAL_UART_Transmit(&Usart1,FPSearchFinger,11,1000);
}

void SendFGetNumberOfFinger()
{
	HAL_UART_Transmit(&Usart1,FPGetNumberOfFinger,6,1000);
}

void SendStoreFinger(uint16_t IDStore)
{
	uint16_t Sum=0;
	uint8_t DataSend[9]={0};

	DataSend[0]=0x01;
	Sum=Sum+DataSend[0];
	DataSend[1]=0x00;
	Sum=Sum+DataSend[1];
	DataSend[2]=0x06;
	Sum=Sum+DataSend[2];
	DataSend[3]=0x06;
	Sum=Sum+DataSend[3];
	DataSend[4]=0x01;
	Sum=Sum+DataSend[4];
	DataSend[5]= (uint8_t)(IDStore>> 8);
	Sum=Sum+DataSend[5];
	DataSend[6]=(uint8_t) (IDStore&0xFF);
	Sum=Sum+DataSend[6];
	DataSend[7]=(uint8_t)(Sum>> 8);
	DataSend[8]=(uint8_t)(Sum&0xFF);
	HAL_UART_Transmit(&Usart1,DataSend,9,1000);
}

void SendDeleteFinger(uint16_t IDDelete)
{
	uint16_t Sum=0;
	uint8_t DataSend[10]={0};

	DataSend[0]=0x01;
	Sum=Sum+DataSend[0];
	DataSend[1]=0x00;
	Sum=Sum+DataSend[1];
	DataSend[2]=0x07;
	Sum=Sum+DataSend[2];
	DataSend[3]=0x0C;
	Sum=Sum+DataSend[3];
	DataSend[4]=(uint8_t)(IDDelete>> 8);
	Sum=Sum+DataSend[4];
	DataSend[5]= (uint8_t) (IDDelete&0xFF);
	Sum=Sum+DataSend[5];
	DataSend[6]=0x00;
	Sum=Sum+DataSend[6];
	DataSend[7]=0x001;
	Sum=Sum+DataSend[7];
  DataSend[8]=(uint8_t)(Sum>> 8);
	DataSend[9]=(uint8_t)(Sum&0xFF);
	HAL_UART_Transmit(&Usart1,DataSend,10,1000);
}




uint8_t CheckFPRespsone(uint8_t MaxRead)
{
	uint8_t ByteCount=0;
	uint8_t FPRXData[20]={0xFF};
	uint8_t UARTData[1]={0};
	uint32_t TimeOut = GetTick();
	uint8_t Result;
	IDFromFinger=0xFF;
	while((GetTick() - TimeOut < 1000) && ByteCount<MaxRead) // time out is 1000 ms
	{
	    if(HAL_UART_Receive(&Usart1, (uint8_t *)UARTData, 1, 1000) == HAL_OK)
			{
				FPRXData[ByteCount] = UARTData[0];
				ByteCount++;
			}
	}

	if(ByteCount==0)
	{
		//FPRXData[0]=0xEE;
		//FPRXData[1]=0xEE;
		//HAL_UART_Transmit(&Usart2,FPRXData,2,1000);
		Result=FP_ERROR;
		return Result;
	}
	else if(ByteCount<MaxRead)
	{
		Result=FP_ERROR;
		return Result;
	}
	  else // vail data return
	{

		 Result=FPRXData[9];
		 IDFromFinger=FPRXData[11];
	   //HAL_UART_Transmit(&Usart2,FPRXData,MaxRead,1000);
		 return Result;

	}
}

uint8_t GetNumberOfFinger()
{
	uint8_t Result;
	SendFPHeader();
	SendFGetNumberOfFinger();
	Result=CheckFPRespsone(14);
	if(Result!=FP_OK) return 0xFF;

	return IDFromFinger;
}

uint8_t RegistryNewFinger(uint16_t LocationID)
{

	uint8_t Result=FP_NOFINGER;
	uint32_t TimeOut = GetTick();
//	HAL_UART_Transmit(&Usart2,(uint8_t *) RegistryFingger,MP3COMMANDLEN,1000);
//	HAL_UART_Transmit(&Usart2,(uint8_t *) PutFinger,MP3COMMANDLEN,1000);
	while(Result==FP_NOFINGER&&(GetTick() - TimeOut < 5000)) // time out is 5000 ms
	{

		SendFPHeader();
		SendFPGetImage();
		Result=CheckFPRespsone(12);
	}
	if(Result!=FP_OK) return FP_ERROR;
	// continue if detect finger;
	SendFPHeader();
	SendFPCreateCharFile1();
	Result=CheckFPRespsone(12);
	if(Result!=FP_OK) return FP_ERROR;

	// second get image
//	HAL_UART_Transmit(&Usart2,(uint8_t *) RemoveFinger,MP3COMMANDLEN,1000);
	Delayms(2000);
	Result=FP_NOFINGER;
	TimeOut = GetTick();
//	HAL_UART_Transmit(&Usart2,(uint8_t *) ReputFinger,MP3COMMANDLEN,1000);

	while(Result==FP_NOFINGER&&(GetTick() - TimeOut < 5000)) // time out is 5000 ms
	{

		SendFPHeader();
		SendFPGetImage();
		Result=CheckFPRespsone(12);
	}
	if(Result!=FP_OK) return FP_ERROR;

	// continue if detect finger;
	SendFPHeader();
	SendFPCreateCharFile2();
	Result=CheckFPRespsone(12);
	if(Result!=FP_OK) return FP_ERROR;

	// Compare finger, create template
	SendFPHeader();
	SendFPCreateTemplate();
	Result=CheckFPRespsone(12);
	if(Result==FP_FINGER_NOTMATCH)
	{

		return FP_FINGER_NOTMATCH;
	}
	else if(Result!=FP_OK) return FP_ERROR;

	// save finger
	SendFPHeader();
	SendStoreFinger(LocationID);
	Result=CheckFPRespsone(12);
	if(Result!=FP_OK) return FP_ERROR;
	else
	{
		return FP_OK;
	}

}

uint8_t CheckFinger()
{
	uint8_t Result=FP_NOFINGER;
	uint32_t TimeOut = GetTick();
//	HAL_UART_Transmit(&Usart2,(uint8_t *) CheckingFinger,MP3COMMANDLEN,1000);

	while(Result==FP_NOFINGER&&(GetTick() - TimeOut < 5000)) // time out is 5000 ms
	{

		SendFPHeader();
		SendFPGetImage();
		Result=CheckFPRespsone(12);
	}
	if(Result!=FP_OK) return FP_ERROR;
	// continue if detect finger;
	SendFPHeader();
	SendFPCreateCharFile1();
	Result=CheckFPRespsone(12);
	if(Result!=FP_OK) return FP_ERROR;

	// Search Fingger
	SendFPHeader();
	SendFPDSearchFinger();
	Result=CheckFPRespsone(16);
	return Result;


}
int main(void)
{
	   uint8_t FingerResult;
	   uint8_t Temp[1];
		DelayInit();
		Delayms(200);
		SysTickInit();
		USART_GPIOInits();
		USART_Inits();
		Delayms(200);
		Usart1.pUSARTx->DR = (uint16_t)0U;
		USART_PeriControl(USART1, ENABLE);




			Delayms(2000);
			CurrentNumberFinger=GetNumberOfFinger();
			Temp[0]=CurrentNumberFinger+48;
			if(CurrentNumberFinger==0xFF) CurrentNumberFinger=0;
			//HAL_UART_Transmit(&huart2,Temp,1,1000);


			Delayms(1000);
		  FingerResult=RegistryNewFinger(CurrentNumberFinger+1);
			if(FingerResult==FP_OK)
			{
				Delayms(20);
			}
			else if(FingerResult==FP_FINGER_NOTMATCH)
			{
				Delayms(30);
			}
			else
			{
				Delayms(40);
			}


}
