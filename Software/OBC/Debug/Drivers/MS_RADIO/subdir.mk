################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MS_RADIO/MSR.c \
../Drivers/MS_RADIO/MSR_Port.c 

OBJS += \
./Drivers/MS_RADIO/MSR.o \
./Drivers/MS_RADIO/MSR_Port.o 

C_DEPS += \
./Drivers/MS_RADIO/MSR.d \
./Drivers/MS_RADIO/MSR_Port.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MS_RADIO/%.o Drivers/MS_RADIO/%.su: ../Drivers/MS_RADIO/%.c Drivers/MS_RADIO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"C:/Users/Sergey/Desktop/Ariane/Arian-Pocket-Cube---1/Software/OBC/Drivers" -I"C:/Users/Sergey/Desktop/Ariane/Arian-Pocket-Cube---1/Software/OBC/UserLibs" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MS_RADIO

clean-Drivers-2f-MS_RADIO:
	-$(RM) ./Drivers/MS_RADIO/MSR.d ./Drivers/MS_RADIO/MSR.o ./Drivers/MS_RADIO/MSR.su ./Drivers/MS_RADIO/MSR_Port.d ./Drivers/MS_RADIO/MSR_Port.o ./Drivers/MS_RADIO/MSR_Port.su

.PHONY: clean-Drivers-2f-MS_RADIO

