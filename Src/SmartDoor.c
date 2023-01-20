/*
 * testSmartDoor.c
 *
 *  Created on: Dec 11, 2022
 *      Author: Metisai_02
 */

/*
 * testvantay.c
 *
 *  Created on: Nov 16, 2022
 *      Author: Metisai_02
 */

#include <stdio.h>
#include <string.h>
#include "stm32f103xx.h"
#include <stm32f103xx_i2c_lcd_driver.h>
#include <vantay.h>
#include <keypad.h>
#include <stm32f103xx_rc522.h>
#include <stm32f103xx_init.h>

#define LCD_ADDR 		0x27

uint8_t data_winform = 0;
uint32_t start1,end1,ticks;
uint8_t register_FingerResult = FP_NOFINGER;

uint8_t ack_fp[] = "N";
uint8_t ack_rfid[] = "M";
uint8_t ack_pass[] = "H";

char password[4] = {'1', '2', '3', '4'};
char getPassword[4];
char getPassword_temp1[4];
char getPassword_temp2[4];

uchar UID_1[5],UID_2[5],UID_3[5];
uchar str[16];
uchar status;
char fp_str[17]={'\0'};
char str2[17]={'\0'};

char str_it[17] = {'\0'};



I2C_Handle_t I2C1Handle;
I2C_LCD_Handle_t LCD;
USART_Handle_t Usart1;
USART_Handle_t Usart2;

uint8_t IDFromFinger;
uint8_t CurrentNumberFinger;

void I2C1_GPIOInits(void);
void I2C1_Inits(void);
void USART_Inits(void);
void USART_GPIOInits(void);

int main(void)
{
	uint8_t FingerResult;
	// Calculate the time of function
	uint8_t Temp[1];
	uchar rc_status1,rc_status2 = MI_ERR;
	uint8_t count = 0;
	uint8_t numberOfError = 0;
	char key;

	SysTickInit();
	DelayInit();
	//
	USART_GPIOInits();
	USART_Inits();
	KEYPAD_Inits();
	//
	I2C1_GPIOInits();
	I2C1_Inits();
	I2C_LCD_init(&LCD, &I2C1Handle, LCD_ADDR);
	//

	//
	MFRC522_Init();
	// Receive data from Winform
	USART_ReceiveDataIT(&Usart2, &data_winform, 1);

	I2C_LCD_display_clear(&LCD);
	I2C_LCD_print_string(&LCD, "Choose Ur Option !!");
	I2C_LCD_set_cursor(&LCD, 1, 0);
	I2C_LCD_print_string(&LCD, "A: FP");
	I2C_LCD_set_cursor(&LCD, 1, 8);
	I2C_LCD_print_string(&LCD, "B: RFID");

	while(1)
	{
			key = getChar();

			if(key != '*' && key != '#' && key != 'D' && key != 0 && key != 0x01 && key !='A' && key != 'B')
			{
						I2C_LCD_display_clear(&LCD);
						I2C_LCD_print_string(&LCD, "Press Ur Pass!");
						getPassword[count] = key;
						Delayms(100);
						for(uint8_t i = 0 ; i <= count; i++)
						{
							I2C_LCD_set_cursor(&LCD, 1, i);
							I2C_LCD_print_char(&LCD, '*');
						}
						count++;
						//cusor++;
						if(count == 4)
						{
							if(getPassword[0] == password[0] && getPassword[1] == password[1] &&
								getPassword[2] == password[2] && getPassword[3] == password[3])
							{
								I2C_LCD_display_clear(&LCD);
								I2C_LCD_set_cursor(&LCD, 0, 4);
								I2C_LCD_print_string(&LCD, "WELCOME!!");
								I2C_LCD_set_cursor(&LCD, 1, 4);
								I2C_LCD_print_string(&LCD, "WELCOME!!");
								Delayms(2000);
								count = 0;
							}
							else
							{
								I2C_LCD_display_clear(&LCD);
								I2C_LCD_set_cursor(&LCD, 0, 1);
								I2C_LCD_print_string(&LCD, "Wrong Password");
								I2C_LCD_set_cursor(&LCD, 1, 1);
								I2C_LCD_print_string(&LCD, "Wrong Password");
								Delayms(1000);
								I2C_LCD_display_clear(&LCD);
								count = 0;
							}
							I2C_LCD_display_clear(&LCD);
							I2C_LCD_print_string(&LCD, "Choose Ur Option !!");
							I2C_LCD_set_cursor(&LCD, 1, 0);
							I2C_LCD_print_string(&LCD, "A: FP");
							I2C_LCD_set_cursor(&LCD, 1, 8);
							I2C_LCD_print_string(&LCD, "B: RFID");
						}
			}
			else if (key == 'A')
			{
				// check the fingerprint
				I2C_LCD_display_clear(&LCD);
				I2C_LCD_set_cursor(&LCD, 0, 1);
				I2C_LCD_print_string(&LCD, "Checking Finger");
				FingerResult = CheckFinger(Usart1, LCD);
				while(FingerResult == FP_NOFINGER)
				{
					FingerResult = CheckFinger(Usart1, LCD);

				}
				if(FingerResult == FP_OK)
				{
					I2C_LCD_display_clear(&LCD);
					I2C_LCD_set_cursor(&LCD, 0, 4);
					I2C_LCD_print_string(&LCD, "WELCOME!!");
					I2C_LCD_set_cursor(&LCD, 1, 4);
					I2C_LCD_print_string(&LCD, "WELCOME!!");
					Delayms(2000);
				}
				else
				{
					I2C_LCD_display_clear(&LCD);
					I2C_LCD_set_cursor(&LCD, 0, 1);
					I2C_LCD_print_string(&LCD, "Finger Error!!");
					I2C_LCD_set_cursor(&LCD, 1, 1);
					I2C_LCD_print_string(&LCD, "Finger Error!!");
					Delayms(2000);

				}
				I2C_LCD_display_clear(&LCD);
				I2C_LCD_print_string(&LCD, "Choose Ur Option !!");
				I2C_LCD_set_cursor(&LCD, 1, 0);
				I2C_LCD_print_string(&LCD, "A: FP");
				I2C_LCD_set_cursor(&LCD, 1, 8);
				I2C_LCD_print_string(&LCD, "B: RFID");
			}
			else if( key == 'B')
			{
				// check RFID
				I2C_LCD_display_clear(&LCD);
				I2C_LCD_set_cursor(&LCD, 0, 2);
				I2C_LCD_print_string(&LCD, "Put Your Card");
				status = MFRC522_Request(PICC_REQIDL, str);
				while(status != MI_OK)
				{
					status = MFRC522_Request(PICC_REQIDL, str);
				}

					status = MFRC522_Anticoll(str);
					if(status == MI_OK)
					{
						if((UID_1[0] == str[0]&&UID_1[1] == str[1]&&UID_1[2] == str[2]&&UID_1[3] == str[3]&&UID_1[4] == str[4])||
								(UID_2[0] == str[0]&&UID_2[1] == str[1]&&UID_2[2] == str[2]&&UID_2[3] == str[3]&&UID_2[4] == str[4])||
								(UID_3[0] == str[0]&&UID_3[1] == str[1]&&UID_3[2] == str[2]&&UID_3[3] == str[3]&&UID_3[4] == str[4]))
						{
							I2C_LCD_display_clear(&LCD);
							I2C_LCD_set_cursor(&LCD, 0, 4);
							I2C_LCD_print_string(&LCD, "WELCOME!!");
							I2C_LCD_set_cursor(&LCD, 1, 4);
							I2C_LCD_print_string(&LCD, "WELCOME!!");
							Delayms(2000);
						}
						else
						{
							I2C_LCD_display_clear(&LCD);
							I2C_LCD_set_cursor(&LCD, 0, 3);
							I2C_LCD_print_string(&LCD, "RFID Error!!");
							I2C_LCD_set_cursor(&LCD, 1, 3);
							I2C_LCD_print_string(&LCD, "RFID Error!!");
							Delayms(1000);
						}

					}
					else
					{
						I2C_LCD_display_clear(&LCD);
						I2C_LCD_set_cursor(&LCD, 0, 3);
						I2C_LCD_print_string(&LCD, "RFID Error!!");
						I2C_LCD_set_cursor(&LCD, 1, 3);
						I2C_LCD_print_string(&LCD, "RFID Error!!");
						Delayms(1000);
					}
				I2C_LCD_display_clear(&LCD);
				I2C_LCD_print_string(&LCD, "Choose Ur Option !!");
				I2C_LCD_set_cursor(&LCD, 1, 0);
				I2C_LCD_print_string(&LCD, "A: FP");
				I2C_LCD_set_cursor(&LCD, 1, 8);
				I2C_LCD_print_string(&LCD, "B: RFID");

			}

	}

}



void I2C1_GPIOInits(void){
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_ALOUT_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_10MHZ;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);

	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void){
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x0;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SPEED_SM;
	I2C_Init(&I2C1Handle);
}
void USART_GPIOInits(void)
{
	GPIO_Handle_t USARTPins;
	USARTPins.pGPIOx = GPIOA;
	USARTPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_ALOUT_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_OUT_SPEED_50MHZ;
	// TX
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&USARTPins);
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_Init(&USARTPins);

	//  RX
	USARTPins.GPIO_PinConfig.GPIO_PinModeAndSpeed = GPIO_MODE_IN;
	USARTPins.GPIO_PinConfig.GPIO_CRF = GPIO_CNF_IN_FLOAT;


	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	GPIO_Init(&USARTPins);
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&USARTPins);

	USART_IRQConfig(IRQ_NO_USART2, ENABLE);

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

	Usart2.pUSARTx = USART2;
	Usart2.USARTConfig.Mode = USART_MODE_TXRX;
	Usart2.USARTConfig.WordLength = USART_WORD_LENGTH_8;
	Usart2.USARTConfig.NuOfStopBits = USART_STOPBITS_1;
	Usart2.USARTConfig.Baudrate = USART_STD_BAUD_57600;
	Usart2.USARTConfig.ParityControl = USART_PARITY_DI;

	USART_Init(&Usart2);

	USART_IRQConfig(IRQ_NO_USART2, ENABLE);

	Delayms(200);
	USART_PeriControl(USART1, ENABLE);
	USART_PeriControl(USART2, ENABLE);
}
void USART2_IRQHandler(void)
{
	USART_IRQHandling(&Usart2);
}
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t ApEv)
{
	if(data_winform == 'A')
	{
		//FINGERPRINT
		I2C_LCD_display_clear(&LCD);
		I2C_LCD_print_string(&LCD, "Register Finger");
		CurrentNumberFinger = GetNumberOfFinger(Usart1);
		if(CurrentNumberFinger > 255)
		{
			CurrentNumberFinger = 1;
		}
		I2C_LCD_set_cursor(&LCD, 1, 0);
		sprintf(fp_str,"Number ID: %d", CurrentNumberFinger);
		I2C_LCD_print_string(&LCD, fp_str);
		register_FingerResult =  RegistryNewFinger(Usart1, CurrentNumberFinger + 1, LCD);

		if(register_FingerResult == FP_NOFINGER)
		{
			I2C_LCD_display_clear(&LCD);
			I2C_LCD_print_string(&LCD, "Finger Not Found");
			Delayms(500);
		}
		else if (register_FingerResult == FP_OK)
		{
			I2C_LCD_display_clear(&LCD);
			I2C_LCD_print_string(&LCD, "Registry Finger");
			I2C_LCD_set_cursor(&LCD, 1, 5);
			I2C_LCD_print_string(&LCD, "DONE :>");
			Delayms(1000);
		}
		else
		{
			I2C_LCD_display_clear(&LCD);
			I2C_LCD_set_cursor(&LCD, 0, 1);
			I2C_LCD_print_string(&LCD, "Finger Error!!");
			I2C_LCD_set_cursor(&LCD, 1, 1);
			I2C_LCD_print_string(&LCD, "Finger Error!!");
			Delayms(500);
		}
		I2C_LCD_display_clear(&LCD);
		I2C_LCD_print_string(&LCD, "Choose Ur Option !!");
		I2C_LCD_set_cursor(&LCD, 1, 0);
		I2C_LCD_print_string(&LCD, "A: FP");
		I2C_LCD_set_cursor(&LCD, 1, 8);
		I2C_LCD_print_string(&LCD, "B: RFID");
		HAL_UART_Transmit(&Usart2, ack_fp, 1, 1000);

	}
	else if (data_winform == 'B')
	{
		uint8_t r_counter = 0;
		char rc_key = 0;
		//RFID
		I2C_LCD_display_clear(&LCD);
		I2C_LCD_set_cursor(&LCD, 0, 4);
		I2C_LCD_print_string(&LCD, "ADD RFID");
		Delayms(700);
		I2C_LCD_display_clear(&LCD);
		I2C_LCD_print_string(&LCD, "Choose The Card");
		I2C_LCD_set_cursor(&LCD, 1, 0);
		I2C_LCD_print_string(&LCD, "To Replace:1,2,3");
		do
		{
			while((rc_key = getChar()) == 0 );
			if(rc_key == '1' ||rc_key == '2' ||rc_key == '3')
			{
				r_counter++;
			}

		}while(r_counter < 1);
		r_counter = 0;
		sprintf(str_it,"Your number: %x",rc_key);
		I2C_LCD_display_clear(&LCD);
		I2C_LCD_print_string(&LCD, str_it);
		Delayms(1000);

		I2C_LCD_display_clear(&LCD);
		I2C_LCD_set_cursor(&LCD, 0, 2);
		I2C_LCD_print_string(&LCD, "Put Ur Card!!");
		I2C_LCD_set_cursor(&LCD, 1, 2);
		I2C_LCD_print_string(&LCD, "Put Ur Card!!");
		while((status = MFRC522_Request(PICC_REQIDL, str)) != MI_OK){}
			status = MFRC522_Anticoll(str);
			  if(status == MI_OK)
			  {
				  sprintf(str2,"UID_%x: %x,%x,%x,%x", rc_key,str[0], str[1], str[2],str[3]);
				  if(rc_key == '1')
				  {
					  UID_1[0] = str[0];
					  UID_1[1] = str[1];
					  UID_1[2] = str[2];
					  UID_1[3] = str[3];
					  UID_1[4] = str[4];
				  }
				  else if(rc_key == '2')
				  {
					  UID_2[0] = str[0];
					  UID_2[1] = str[1];
					  UID_2[2] = str[2];
					  UID_2[3] = str[3];
					  UID_2[4] = str[4];
				  }
				  else
				  {
					  UID_3[0] = str[0];
					  UID_3[1] = str[1];
					  UID_3[2] = str[2];
					  UID_3[3] = str[3];
					  UID_3[4] = str[4];
				  }
				  I2C_LCD_display_clear(&LCD);
				  I2C_LCD_set_cursor(&LCD, 1, 0);
				  I2C_LCD_print_string(&LCD, str2);
				  Delayms(2000);
			  }
			  else
			  {
				  I2C_LCD_display_clear(&LCD);
				I2C_LCD_set_cursor(&LCD, 0, 3);
				I2C_LCD_print_string(&LCD, "RFID Error!!");
				I2C_LCD_set_cursor(&LCD, 1, 3);
				I2C_LCD_print_string(&LCD, "RFID Error!!");
				Delayms(1000);
			  }
				I2C_LCD_display_clear(&LCD);
				I2C_LCD_print_string(&LCD, "Choose Ur Option !!");
				I2C_LCD_set_cursor(&LCD, 1, 0);
				I2C_LCD_print_string(&LCD, "A: FP");
				I2C_LCD_set_cursor(&LCD, 1, 8);
				I2C_LCD_print_string(&LCD, "B: RFID");
				HAL_UART_Transmit(&Usart2, ack_rfid, 1, 1000);

	}
	else if(data_winform == 'C')
	{
		char r_key;
		uint8_t r_counter = 0;

		//PASSWORD
		I2C_LCD_display_clear(&LCD);
		I2C_LCD_print_string(&LCD, "Change Password");
		Delayms(500);
		I2C_LCD_display_clear(&LCD);
		I2C_LCD_set_cursor(&LCD, 0, 4);
		I2C_LCD_print_string(&LCD, "PassWord");

		do
		{
			while((r_key = getChar()) == 0 );
			if(r_key != '*' && r_key != '#' && r_key != 'D' && r_key != 0 && r_key != 0x01 && r_key !='A' && r_key != 'B')
			{
				getPassword_temp1[r_counter] = r_key;
				Delayms(100);
				for(uint8_t i = 0 ; i <= r_counter; i++)
				{
				I2C_LCD_set_cursor(&LCD, 1, i);
				I2C_LCD_print_char(&LCD, '*');
				}
				r_counter++;
			}

		}while(r_counter < 4);

		r_counter = 0;
		I2C_LCD_display_clear(&LCD);
		I2C_LCD_print_string(&LCD, "Confirm Password:");
		do
		{
			while((r_key = getChar()) == 0 );
			if(r_key != '*' && r_key != '#' && r_key != 'D' && r_key != 0 && r_key != 0x01 && r_key !='A' && r_key != 'B')
			{
				getPassword_temp2[r_counter] = r_key;
				Delayms(100);
				for(uint8_t i = 0 ; i <= r_counter; i++)
				{
					I2C_LCD_set_cursor(&LCD, 1, i);
					I2C_LCD_print_char(&LCD, '*');
				}
				r_counter++;
			}
			if(r_counter == 4)
			{
				if(getPassword_temp1[0] == getPassword_temp2[0] && getPassword_temp1[1] == getPassword_temp2[1] &&
					getPassword_temp1[2] == getPassword_temp2[2] && getPassword_temp1[3] == getPassword_temp2[3])
				{
					password[0] = getPassword_temp1[0];
					password[1] = getPassword_temp1[1];
					password[2] = getPassword_temp1[2];
					password[3] = getPassword_temp1[3];
					I2C_LCD_display_clear(&LCD);
					I2C_LCD_print_string(&LCD, "Registry Pass");
					I2C_LCD_set_cursor(&LCD, 1, 5);
					I2C_LCD_print_string(&LCD, "DONE :>");
					Delayms(2000);
				}
				else
				{
					I2C_LCD_display_clear(&LCD);
					I2C_LCD_set_cursor(&LCD, 0, 4);
					I2C_LCD_print_string(&LCD, " Both Are");
					I2C_LCD_set_cursor(&LCD, 1, 4);
					I2C_LCD_print_string(&LCD, "Different");
					Delayms(2000);

				}
			}

		}while(r_counter < 4);
		I2C_LCD_display_clear(&LCD);
		I2C_LCD_print_string(&LCD, "Choose Ur Option !!");
		I2C_LCD_set_cursor(&LCD, 1, 0);
		I2C_LCD_print_string(&LCD, "A: FP");
		I2C_LCD_set_cursor(&LCD, 1, 8);
		I2C_LCD_print_string(&LCD, "B: RFID");
		HAL_UART_Transmit(&Usart2, ack_pass, 1, 1000);

	}
	else if(data_winform == 'P')
	{
		I2C_LCD_display_clear(&LCD);
		I2C_LCD_set_cursor(&LCD, 0, 4);
		I2C_LCD_print_string(&LCD,"Clear ALL!!!");
		I2C_LCD_set_cursor(&LCD, 1, 4);
		I2C_LCD_print_string(&LCD,"Clear ALL!!!");
		Delayms(1000);
		if(deleteallfinger(Usart1) == FP_ERROR)
		{
			I2C_LCD_display_clear(&LCD);
			I2C_LCD_set_cursor(&LCD, 0, 1);
			I2C_LCD_print_string(&LCD,"Could Not Clear");
			I2C_LCD_set_cursor(&LCD, 1, 1);
			I2C_LCD_print_string(&LCD,"Could Not Clear :>>");
		}
		{
			I2C_LCD_display_clear(&LCD);
			I2C_LCD_set_cursor(&LCD, 0, 6);
			I2C_LCD_print_string(&LCD,"DONE");
			I2C_LCD_set_cursor(&LCD, 1, 6);
			I2C_LCD_print_string(&LCD,"DONE :>>");
			Delayms(1000);
		}
		I2C_LCD_display_clear(&LCD);
		I2C_LCD_print_string(&LCD, "Choose Ur Option !!");
		I2C_LCD_set_cursor(&LCD, 1, 0);
		I2C_LCD_print_string(&LCD, "A: FP");
		I2C_LCD_set_cursor(&LCD, 1, 8);
		I2C_LCD_print_string(&LCD, "B: RFID");

	}
	else if(data_winform == 'R')
	{
		uint8_t r_counter = 0;
				char rc_key = 0;
				//RFID
				I2C_LCD_display_clear(&LCD);
				I2C_LCD_print_string(&LCD, "Choose The Card");
				I2C_LCD_set_cursor(&LCD, 1, 0);
				I2C_LCD_print_string(&LCD, "To Clear: 1,2,3");
				do
				{
					while((rc_key = getChar()) == 0 );
					if(rc_key == '1' ||rc_key == '2' ||rc_key == '3')
					{
						r_counter++;
					}

				}while(r_counter < 1);
				r_counter = 0;
				sprintf(str_it,"Your number: %x",rc_key);
				I2C_LCD_display_clear(&LCD);
				I2C_LCD_print_string(&LCD, str_it);
				Delayms(1000);
						  if(rc_key == '1')
						  {
							  UID_1[0] = 0;
							  UID_1[1] = 0;
							  UID_1[2] = 0;
							  UID_1[3] = 0;
							  UID_1[4] = 0;
						  }
						  else if(rc_key == '2')
						  {
							  UID_2[0] = 0;
							  UID_2[1] = 0;
							  UID_2[2] = 0;
							  UID_2[3] = 0;
							  UID_2[4] = 0;
						  }
						  else
						  {
							  UID_3[0] = 0;
							  UID_3[1] = 0;
							  UID_3[2] = 0;
							  UID_3[3] = 0;
							  UID_3[4] = 0;
						  }
							I2C_LCD_display_clear(&LCD);
							I2C_LCD_set_cursor(&LCD, 0, 6);
							I2C_LCD_print_string(&LCD,"DONE");
							I2C_LCD_set_cursor(&LCD, 1, 6);
							I2C_LCD_print_string(&LCD,"DONE :>>");
							Delayms(1000);
							I2C_LCD_display_clear(&LCD);
							I2C_LCD_print_string(&LCD, "Choose Ur Option !!");
							I2C_LCD_set_cursor(&LCD, 1, 0);
							I2C_LCD_print_string(&LCD, "A: FP");
							I2C_LCD_set_cursor(&LCD, 1, 8);
							I2C_LCD_print_string(&LCD, "B: RFID");
	}
	USART_ReceiveDataIT(&Usart2, &data_winform, 1);
}

