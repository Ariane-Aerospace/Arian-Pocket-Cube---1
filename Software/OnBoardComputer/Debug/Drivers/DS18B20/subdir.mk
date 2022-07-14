################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/DS18B20/DS18B20.c \
../Drivers/DS18B20/DS18B20_port.c 

OBJS += \
./Drivers/DS18B20/DS18B20.o \
./Drivers/DS18B20/DS18B20_port.o 

C_DEPS += \
./Drivers/DS18B20/DS18B20.d \
./Drivers/DS18B20/DS18B20_port.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/DS18B20/%.o Drivers/DS18B20/%.su: ../Drivers/DS18B20/%.c Drivers/DS18B20/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"C:/Users/ForseFire/Desktop/OnBoardComputer/Drivers" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-DS18B20

clean-Drivers-2f-DS18B20:
	-$(RM) ./Drivers/DS18B20/DS18B20.d ./Drivers/DS18B20/DS18B20.o ./Drivers/DS18B20/DS18B20.su ./Drivers/DS18B20/DS18B20_port.d ./Drivers/DS18B20/DS18B20_port.o ./Drivers/DS18B20/DS18B20_port.su

.PHONY: clean-Drivers-2f-DS18B20

