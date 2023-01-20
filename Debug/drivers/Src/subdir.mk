################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/keypad.c \
../drivers/Src/stm32f103_i2c_driver.c \
../drivers/Src/stm32f103xx.gpio_driver.c \
../drivers/Src/stm32f103xx_HAL_usart.c \
../drivers/Src/stm32f103xx_crc_driver.c \
../drivers/Src/stm32f103xx_delay.c \
../drivers/Src/stm32f103xx_i2c_lcd_driver.c \
../drivers/Src/stm32f103xx_init.c \
../drivers/Src/stm32f103xx_rc522.c \
../drivers/Src/stm32f103xx_rcc.c \
../drivers/Src/stm32f103xx_spi_driver.c \
../drivers/Src/stm32f103xx_usart_driver.c \
../drivers/Src/vantay.c 

OBJS += \
./drivers/Src/keypad.o \
./drivers/Src/stm32f103_i2c_driver.o \
./drivers/Src/stm32f103xx.gpio_driver.o \
./drivers/Src/stm32f103xx_HAL_usart.o \
./drivers/Src/stm32f103xx_crc_driver.o \
./drivers/Src/stm32f103xx_delay.o \
./drivers/Src/stm32f103xx_i2c_lcd_driver.o \
./drivers/Src/stm32f103xx_init.o \
./drivers/Src/stm32f103xx_rc522.o \
./drivers/Src/stm32f103xx_rcc.o \
./drivers/Src/stm32f103xx_spi_driver.o \
./drivers/Src/stm32f103xx_usart_driver.o \
./drivers/Src/vantay.o 

C_DEPS += \
./drivers/Src/keypad.d \
./drivers/Src/stm32f103_i2c_driver.d \
./drivers/Src/stm32f103xx.gpio_driver.d \
./drivers/Src/stm32f103xx_HAL_usart.d \
./drivers/Src/stm32f103xx_crc_driver.d \
./drivers/Src/stm32f103xx_delay.d \
./drivers/Src/stm32f103xx_i2c_lcd_driver.d \
./drivers/Src/stm32f103xx_init.d \
./drivers/Src/stm32f103xx_rc522.d \
./drivers/Src/stm32f103xx_rcc.d \
./drivers/Src/stm32f103xx_spi_driver.d \
./drivers/Src/stm32f103xx_usart_driver.d \
./drivers/Src/vantay.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/keypad.o: ../drivers/Src/keypad.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/keypad.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f103_i2c_driver.o: ../drivers/Src/stm32f103_i2c_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103_i2c_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f103xx.gpio_driver.o: ../drivers/Src/stm32f103xx.gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103xx.gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f103xx_HAL_usart.o: ../drivers/Src/stm32f103xx_HAL_usart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103xx_HAL_usart.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f103xx_crc_driver.o: ../drivers/Src/stm32f103xx_crc_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103xx_crc_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f103xx_delay.o: ../drivers/Src/stm32f103xx_delay.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103xx_delay.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f103xx_i2c_lcd_driver.o: ../drivers/Src/stm32f103xx_i2c_lcd_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103xx_i2c_lcd_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f103xx_init.o: ../drivers/Src/stm32f103xx_init.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103xx_init.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f103xx_rc522.o: ../drivers/Src/stm32f103xx_rc522.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103xx_rc522.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f103xx_rcc.o: ../drivers/Src/stm32f103xx_rcc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103xx_rcc.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f103xx_spi_driver.o: ../drivers/Src/stm32f103xx_spi_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103xx_spi_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/stm32f103xx_usart_driver.o: ../drivers/Src/stm32f103xx_usart_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103xx_usart_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
drivers/Src/vantay.o: ../drivers/Src/vantay.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I"C:/Users/Metisai_02/STM32CubeIDE/workspace_1.0.2/stm32f103xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/vantay.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

