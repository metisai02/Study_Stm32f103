/*
 * vantay.c
 *
 *  Created on: Nov 16, 2022
 *      Author: Metisai_02
 */



#include "vantay.h"
#include "stdio.h"
	uint8_t FPHeader[6]={0xEF,0x01,0xFF,0xFF,0xFF,0xFF};
	uint8_t FPGetImage[6]={0x01,0x00,0x03,0x01,0x00,0x05};
	uint8_t FPCreateCharFile1[7]={0x01,0x00,0x04,0x02,0x01,0x00,0x08};
	uint8_t FPCreateCharFile2[7]={0x01,0x00,0x04,0x02,0x02,0x00,0x09};
	uint8_t FPCreateTemplate[6]={0x01,0x00,0x03,0x05,0x00,0x09};
	uint8_t FPDeleteAllFinger[6]={0x01,0x00,0x03,0x0D,0x00,0x11};
	uint8_t FPSearchFinger[11]={0x01,0x00,0x08,0x04,0x01,0x00,0x00,0x00,0x40,0x00,0x4E};
	uint8_t FPGetNumberOfFinger[6]={0x01,0x00,0x03,0x1D,0x00,0x21};
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
	 char* deletealldone =   "delete all done \r\n";
		uint8_t IDFromFinger;
		uint8_t CurrentNumberFinger;
void SendFPHeader(USART_Handle_t Usart1)
{
	HAL_UART_Transmit(&Usart1, FPHeader, 6,1000);
}

void SendFPGetImage(USART_Handle_t Usart1)
{
	 HAL_UART_Transmit(&Usart1,FPGetImage,6,1000);
}

void SendFPCreateCharFile1(USART_Handle_t Usart1)
{
	HAL_UART_Transmit(&Usart1,FPCreateCharFile1,7,1000);
}

void SendFPCreateCharFile2(USART_Handle_t Usart1)
{
	HAL_UART_Transmit(&Usart1,FPCreateCharFile2,7,1000);
}

void SendFPCreateTemplate(USART_Handle_t Usart1)
{
	HAL_UART_Transmit(&Usart1,FPCreateTemplate,6,1000);
}

void SendFPDeleteAllFinger(USART_Handle_t Usart1)
{
	HAL_UART_Transmit(&Usart1,FPDeleteAllFinger,6,1000);
}

void SendFPDSearchFinger(USART_Handle_t Usart1)
{
	HAL_UART_Transmit(&Usart1,FPSearchFinger,11,1000);
}

void SendFGetNumberOfFinger(USART_Handle_t Usart1)
{
	HAL_UART_Transmit(&Usart1,FPGetNumberOfFinger,6,1000);
}
void SendStoreFinger(USART_Handle_t Usart1, uint16_t IDStore)
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
void SendDeleteFinger(USART_Handle_t Usart1 , uint16_t IDDelete)
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
uint8_t CheckFPRespsone(USART_Handle_t Usart1, uint8_t MaxRead)
{
	uint8_t ByteCount=0;
	uint8_t FPRXData[20]={0xFE};
	uint8_t UARTData[1]={0};

	uint32_t TimeOut = GetTick();

	uint8_t Result;
	IDFromFinger=0xFF;

	while((GetTick() - TimeOut < 1000) && ByteCount<MaxRead) // time out is 1000 ms
	{
	    	if(HAL_UART_Receive(&Usart1, (uint8_t *)UARTData, 1,1000) == HAL_OK)
			{
				FPRXData[ByteCount] = UARTData[0];
				ByteCount++;
			}

	}


	if(ByteCount==0)
	{
		Result=FP_ERROR;
		return Result;
	}
	else if(ByteCount<MaxRead)
	{
		Result=FP_ERROR;
		return Result;
	}
	  else // vailue data return
	{

		 Result=FPRXData[9];
		 IDFromFinger=FPRXData[11];
		 return Result;

	}
}
uint8_t GetNumberOfFinger(USART_Handle_t Usart1)
{
	uint8_t Result;
	SendFPHeader(Usart1);
	SendFGetNumberOfFinger(Usart1);
	Result=CheckFPRespsone(Usart1,14);
	if(Result!=FP_OK) return 0xFF;

	return IDFromFinger;
}
uint8_t RegistryNewFinger(USART_Handle_t Usart1, uint16_t LocationID,I2C_LCD_Handle_t LCD)
{

	uint8_t Result=FP_NOFINGER;
	uint32_t TimeOut = GetTick();

	while(Result==FP_NOFINGER&&(GetTick() - TimeOut < 3000)) // time out is 5000 ms
	{
		SendFPHeader(Usart1);
		SendFPGetImage(Usart1);
		Result=CheckFPRespsone(Usart1,12);
	}
	if(Result == FP_NOFINGER) return FP_NOFINGER;
	if (Result!=FP_OK) return FP_ERROR;
	// continue if detect finger;
	SendFPHeader(Usart1);
	SendFPCreateCharFile1(Usart1);
	Result=CheckFPRespsone(Usart1,12);
	if(Result!=FP_OK) return FP_ERROR;

//		Delayms(2000);
	Result=FP_NOFINGER;
	TimeOut = GetTick();

	I2C_LCD_display_clear(&LCD);
	I2C_LCD_print_string(&LCD, ReputFinger);

	while(Result==FP_NOFINGER&&(GetTick() - TimeOut < 3000))
	{
		SendFPHeader(Usart1);
		SendFPGetImage(Usart1);
		Result=CheckFPRespsone(Usart1,12);
	}
	if(Result == FP_NOFINGER) return FP_NOFINGER;
	if (Result!=FP_OK) return FP_ERROR;

	// continue if detect finger;
	SendFPHeader(Usart1);
	SendFPCreateCharFile2(Usart1);
	Result=CheckFPRespsone(Usart1,12);
	if(Result!=FP_OK) return FP_ERROR;

	// Compare finger, create template
	SendFPHeader(Usart1);
	SendFPCreateTemplate(Usart1);
	Result=CheckFPRespsone(Usart1,12);
	if(Result==FP_FINGER_NOTMATCH)
	{

		return FP_FINGER_NOTMATCH;
	}
	else if(Result!=FP_OK) return FP_ERROR;

	// save finger
	SendFPHeader(Usart1);
	SendStoreFinger(Usart1,LocationID);
	Result=CheckFPRespsone(Usart1,12);
	if(Result!=FP_OK) return FP_ERROR;
	else
	{
		return FP_OK;
	}

}
uint8_t CheckFinger(USART_Handle_t Usart1,I2C_LCD_Handle_t LCD)
{
	uint8_t Result=FP_NOFINGER;
	uint32_t TimeOut = GetTick();

	while(Result==FP_NOFINGER&&(GetTick() - TimeOut < 2000))
	{
		SendFPHeader(Usart1);
		SendFPGetImage(Usart1);
		Result=CheckFPRespsone(Usart1,12);
	}
	if(Result == FP_NOFINGER) return FP_NOFINGER;
	if (Result!=FP_OK) return FP_ERROR;
	// continue if detect finger;
	SendFPHeader(Usart1);
	SendFPCreateCharFile1(Usart1);
	Result=CheckFPRespsone(Usart1,12);
	if(Result!=FP_OK) return FP_ERROR;

	// Search Fingger
	SendFPHeader(Usart1);
	SendFPDSearchFinger(Usart1);
	Result=CheckFPRespsone(Usart1,16);
	return Result;


}


uint8_t deleteallfinger(USART_Handle_t Usart1){
	uint8_t Result;
	SendFPHeader(Usart1);
	SendFPDeleteAllFinger(Usart1);
	Result=CheckFPRespsone(Usart1,12);
	if(Result!=FP_OK) return FP_ERROR ;
	else return Result;


}

uint8_t Delete_ID(USART_Handle_t Usart1, uint16_t ID)
{
	uint8_t Result;
	SendFPHeader(Usart1);
	SendDeleteFinger(Usart1,ID);
	Result=CheckFPRespsone(Usart1,16);
	return Result;
}
