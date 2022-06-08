################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../UserLibs/DS18B20/DS18B20.c \
../UserLibs/DS18B20/DS18B20_port.c 

OBJS += \
./UserLibs/DS18B20/DS18B20.o \
./UserLibs/DS18B20/DS18B20_port.o 

C_DEPS += \
./UserLibs/DS18B20/DS18B20.d \
./UserLibs/DS18B20/DS18B20_port.d 


# Each subdirectory must supply rules for building sources it contributes
UserLibs/DS18B20/%.o UserLibs/DS18B20/%.su: ../UserLibs/DS18B20/%.c UserLibs/DS18B20/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"C:/Users/Sergey/Desktop/Ariane/Arian-Pocket-Cube---1/Software/OBC/Drivers" -I"C:/Users/Sergey/Desktop/Ariane/Arian-Pocket-Cube---1/Software/OBC/UserLibs" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-UserLibs-2f-DS18B20

clean-UserLibs-2f-DS18B20:
	-$(RM) ./UserLibs/DS18B20/DS18B20.d ./UserLibs/DS18B20/DS18B20.o ./UserLibs/DS18B20/DS18B20.su ./UserLibs/DS18B20/DS18B20_port.d ./UserLibs/DS18B20/DS18B20_port.o ./UserLibs/DS18B20/DS18B20_port.su

.PHONY: clean-UserLibs-2f-DS18B20

