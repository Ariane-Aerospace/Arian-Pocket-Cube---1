################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/QMC5883L/QMC5883L.c \
../Drivers/QMC5883L/QMC5883L_port.c 

OBJS += \
./Drivers/QMC5883L/QMC5883L.o \
./Drivers/QMC5883L/QMC5883L_port.o 

C_DEPS += \
./Drivers/QMC5883L/QMC5883L.d \
./Drivers/QMC5883L/QMC5883L_port.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/QMC5883L/%.o Drivers/QMC5883L/%.su: ../Drivers/QMC5883L/%.c Drivers/QMC5883L/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"C:/Users/ForseFire/Desktop/OnBoardComputer/Drivers" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-QMC5883L

clean-Drivers-2f-QMC5883L:
	-$(RM) ./Drivers/QMC5883L/QMC5883L.d ./Drivers/QMC5883L/QMC5883L.o ./Drivers/QMC5883L/QMC5883L.su ./Drivers/QMC5883L/QMC5883L_port.d ./Drivers/QMC5883L/QMC5883L_port.o ./Drivers/QMC5883L/QMC5883L_port.su

.PHONY: clean-Drivers-2f-QMC5883L

