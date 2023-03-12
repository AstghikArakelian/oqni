################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Buffer/Src/Buffer.c 

OBJS += \
./Buffer/Src/Buffer.o 

C_DEPS += \
./Buffer/Src/Buffer.d 


# Each subdirectory must supply rules for building sources it contributes
Buffer/Src/%.o Buffer/Src/%.su: ../Buffer/Src/%.c Buffer/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -DHAL_STM32F103 -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../ob1203_lib/Inc -I../Buffer/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Buffer-2f-Src

clean-Buffer-2f-Src:
	-$(RM) ./Buffer/Src/Buffer.d ./Buffer/Src/Buffer.o ./Buffer/Src/Buffer.su

.PHONY: clean-Buffer-2f-Src

