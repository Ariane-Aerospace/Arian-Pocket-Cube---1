################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MS5607/MS5607.c \
../Drivers/MS5607/MS5607_port.c 

OBJS += \
./Drivers/MS5607/MS5607.o \
./Drivers/MS5607/MS5607_port.o 

C_DEPS += \
./Drivers/MS5607/MS5607.d \
./Drivers/MS5607/MS5607_port.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MS5607/%.o Drivers/MS5607/%.su: ../Drivers/MS5607/%.c Drivers/MS5607/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"C:/Users/ForseFire/Desktop/OnBoardComputer/Drivers" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MS5607

clean-Drivers-2f-MS5607:
	-$(RM) ./Drivers/MS5607/MS5607.d ./Drivers/MS5607/MS5607.o ./Drivers/MS5607/MS5607.su ./Drivers/MS5607/MS5607_port.d ./Drivers/MS5607/MS5607_port.o ./Drivers/MS5607/MS5607_port.su

.PHONY: clean-Drivers-2f-MS5607

