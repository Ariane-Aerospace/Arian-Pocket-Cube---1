################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../UserLibs/LSM6DS3/LSM6DS3.c \
../UserLibs/LSM6DS3/LSM6DS3_port.c 

OBJS += \
./UserLibs/LSM6DS3/LSM6DS3.o \
./UserLibs/LSM6DS3/LSM6DS3_port.o 

C_DEPS += \
./UserLibs/LSM6DS3/LSM6DS3.d \
./UserLibs/LSM6DS3/LSM6DS3_port.d 


# Each subdirectory must supply rules for building sources it contributes
UserLibs/LSM6DS3/%.o UserLibs/LSM6DS3/%.su: ../UserLibs/LSM6DS3/%.c UserLibs/LSM6DS3/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"C:/Users/Sergey/Desktop/Ariane/Arian-Pocket-Cube---1/Software/OBC/Drivers" -I"C:/Users/Sergey/Desktop/Ariane/Arian-Pocket-Cube---1/Software/OBC/UserLibs" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-UserLibs-2f-LSM6DS3

clean-UserLibs-2f-LSM6DS3:
	-$(RM) ./UserLibs/LSM6DS3/LSM6DS3.d ./UserLibs/LSM6DS3/LSM6DS3.o ./UserLibs/LSM6DS3/LSM6DS3.su ./UserLibs/LSM6DS3/LSM6DS3_port.d ./UserLibs/LSM6DS3/LSM6DS3_port.o ./UserLibs/LSM6DS3/LSM6DS3_port.su

.PHONY: clean-UserLibs-2f-LSM6DS3

