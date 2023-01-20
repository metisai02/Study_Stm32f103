/*
 * stm32f103_i2c_driver.h
 *
 *  Created on: Nov 14, 2022
 *      Author: Metisai_02
 */

#ifndef INC_STM32F103_I2C_DRIVER_H_
#define INC_STM32F103_I2C_DRIVER_H_

#include <stm32f103xx.h>


/*
 * This is a configuration structure for a I2Cx peripheral
 */

typedef struct{
	uint32_t I2C_SCLSpeed;				/** possible values from @I2C_SCLSpeed */
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;			/** possible values from @I2C_ACKControl */
	uint8_t  I2C_FMDutyCycle;			/** possible values from @I2C_FMDutyCycle */
}I2C_Config_t;


/*
 * This is a Handle structure for a I2Cx peripheral
 */
typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;


/*
 * APIs suppoerted by this driver
 * For more information about the APIs check the function definitiond
 */

/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Acknowledgement control
 */
void I2C_AckControl (I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Other Peripheral Control  APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t Flagname);

/*
 * Init and De-Init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint8_t len, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t len, uint8_t SlaveAddr);

/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriConfig(uint8_t IRQNumber, uint32_t IRQPriority);



/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t pI2CHandle, uint8_t AppEv);

/*
 * @I2C_SCLSpeed
 */
#define I2C_SPEED_SM			100000
#define I2C_SPEED_FM			400000
#define I2C_SPEED_FM_2K			200000

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE			(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE			(1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB				(1 << I2C_SR1_SB)
#define I2C_FLAG_OVR			(1 << I2C_SR1_OVR)
#define I2C_FLAG_AF				(1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO			(1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR			(1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF			(1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10			(1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF			(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR			(1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT		(1 << I2C_SR1_TIMEOUT)

#endif /* INC_STM32F103_I2C_DRIVER_H_ */
