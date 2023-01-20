/*
 * stm32f103xx.gpio_driver.c
 *
 *  Created on: Oct 6, 2022
 *      Author: Metisai_02
 */
#include "stm32f103xx.gpio_driver.h"

/******************************************************************
 * Tên hàm:	GPIO_PeriClockControl
 * Chức năng: Cấu hình xung clock cho PORT
 * Input:	*pGPIOx-> địa chỉ của PORT
 * 			EnorDi-> Enable hay Diable
 * Trả về: Không
 *******************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, bool EnOrDi) {
    if (EnOrDi) {
        if (pGPIOx == GPIOA) GPIOA_PCLK_EN();
        else if (pGPIOx == GPIOB) GPIOB_PCLK_EN();
        else if (pGPIOx == GPIOC) GPIOC_PCLK_EN();
        else if (pGPIOx == GPIOD) GPIOD_PCLK_EN();
        else if (pGPIOx == GPIOE) GPIOE_PCLK_EN();
    } else {
        if (pGPIOx == GPIOA) GPIOA_PCLK_DI();
        else if (pGPIOx == GPIOB) GPIOB_PCLK_DI();
        else if (pGPIOx == GPIOC) GPIOC_PCLK_DI();
        else if (pGPIOx == GPIOD) GPIOD_PCLK_DI();
        else if (pGPIOx == GPIOE) GPIOE_PCLK_DI();
    }
}

/*
 * 	Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// bật xung clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

		bool crLorH = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber >= GPIO_PIN_8;
		uint8_t crShifVal = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
	//1. Configure the mode of GPIO Pin
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinModeAndSpeed <= 0b11)
		{
			pGPIOHandle->pGPIOx->CR[crLorH]	&=	~((0b11)<<(4*crShifVal));
			pGPIOHandle->pGPIOx->CR[crLorH]	|= pGPIOHandle->GPIO_PinConfig.GPIO_PinModeAndSpeed<<(4*crShifVal); /*SET: Tại ở giữa MODE là có CNF nên*/
		}
	//2. Configure the CRF

	    if (((pGPIOHandle->GPIO_PinConfig.GPIO_PinModeAndSpeed) == GPIO_MODE_IN) &&
	        // if the cnf is reserved, configure the pin as Floating Input(reset state)
	        (pGPIOHandle->GPIO_PinConfig.GPIO_CRF == GPIO_CNF_IN_RESERVED))
	        pGPIOHandle->GPIO_PinConfig.GPIO_CRF = GPIO_CNF_IN_FLOAT;
	    pGPIOHandle->pGPIOx->CR[crLorH] &= ~(0b11 << ((4 * crShifVal) + 2)); // clear
	    pGPIOHandle->pGPIOx->CR[crLorH] |= pGPIOHandle->GPIO_PinConfig.GPIO_CRF << ((4 * crShifVal) + 2);


	//3. Configure the Push up or push down settings

	    if ((pGPIOHandle->GPIO_PinConfig.GPIO_PinModeAndSpeed) == GPIO_MODE_IN) {
	        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl == GPIO_PU)
	            pGPIOHandle->pGPIOx->ODR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	        else pGPIOHandle->pGPIOx->ODR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	    }
	//4. Configure the IRQ
		if((pGPIOHandle->GPIO_PinConfig.GPIO_PinModeAndSpeed) == GPIO_MODE_IN && pGPIOHandle->GPIO_PinConfig.GPIO_PinIRQ_Trigger == GPIO_IRQ_RT)
		{
			/*cấu hình cạnh lên cho ngắt*/
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/*clear*/
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			AFIO_PCLK_EN();

			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

			uint8_t PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); // cái lìn này để xác định port

			AFIO->EXTICR[temp1] |=	(PortCode << (temp2 * 4));

			/*SET thanh ghi EXTI MASK*/

			EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if((pGPIOHandle->GPIO_PinConfig.GPIO_PinModeAndSpeed) == GPIO_MODE_IN && pGPIOHandle->GPIO_PinConfig.GPIO_PinIRQ_Trigger == GPIO_IRQ_FT)

		{
			/*cấu hình cạnh xuống cho ngắt*/
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/*clear*/
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			AFIO_PCLK_EN();

			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

			uint8_t PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); // cái lìn này để xác định port

			AFIO->EXTICR[temp1] |=	(PortCode << (temp2 * 4));

			/*SET thanh ghi EXTI MASK*/

			EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if((pGPIOHandle->GPIO_PinConfig.GPIO_PinModeAndSpeed) == GPIO_MODE_IN && pGPIOHandle->GPIO_PinConfig.GPIO_PinIRQ_Trigger == GPIO_IRQ_RFT)
		{
			/*cấu hình cạnh lên cho ngắt*/
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			AFIO_PCLK_EN();

			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

			uint8_t PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); // cái lìn này để xác định port

			AFIO->EXTICR[temp1] |=	(PortCode << (temp2 * 4));

			/*SET thanh ghi EXTI MASK*/

			EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		// bật clock cho AFIO
//		AFIO_PCLK_EN();

//		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
//		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

//		uint8_t PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); // cái lìn này để xác định port

//		AFIO->AFIO_EXTICR[temp1] |=	(PortCode << (temp2 * 4));

//		/*SET thanh ghi EXTI MASK*/

//		EXTI->EXTI_IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);



	//5. Configue the Alt functionality
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * 	Read and Write data
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    return (pGPIOx->IDR >> PinNumber) & 1;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
    return (uint16_t) (pGPIOx->IDR);
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
    if (Value == GPIO_PIN_SET) {
        pGPIOx->ODR |= (1 << PinNumber);
    } else {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value) {
    pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * 	IRQ configuration and IRQ Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    uint8_t regx = IRQNumber / 32;
    if (EnorDi == ENABLE)
        NVIC_ISER->REG_NU[regx] |= (1 << (IRQNumber % 32));
    else
        NVIC_ICER->REG_NUM[regx] |= (1 << (IRQNumber % 32));
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
    // clear the EXTI PR register corresponding to the pin
    if (EXTI->PR & (1 << PinNumber)) EXTI->PR |= (1 << PinNumber); // clear
}








