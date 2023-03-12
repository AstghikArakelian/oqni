################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../afe_lib/Src/f103.c \
../afe_lib/Src/heartrate_3.c \
../afe_lib/Src/heartrate_3_hal.c \
../afe_lib/Src/heartrate_3_hw.c 

OBJS += \
./afe_lib/Src/f103.o \
./afe_lib/Src/heartrate_3.o \
./afe_lib/Src/heartrate_3_hal.o \
./afe_lib/Src/heartrate_3_hw.o 

C_DEPS += \
./afe_lib/Src/f103.d \
./afe_lib/Src/heartrate_3.d \
./afe_lib/Src/heartrate_3_hal.d \
./afe_lib/Src/heartrate_3_hw.d 


# Each subdirectory must supply rules for building sources it contributes
afe_lib/Src/%.o afe_lib/Src/%.su: ../afe_lib/Src/%.c afe_lib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -DHAL_STM32F103 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/../afe_lib/Inc -I../Buffer/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-afe_lib-2f-Src

clean-afe_lib-2f-Src:
	-$(RM) ./afe_lib/Src/f103.d ./afe_lib/Src/f103.o ./afe_lib/Src/f103.su ./afe_lib/Src/heartrate_3.d ./afe_lib/Src/heartrate_3.o ./afe_lib/Src/heartrate_3.su ./afe_lib/Src/heartrate_3_hal.d ./afe_lib/Src/heartrate_3_hal.o ./afe_lib/Src/heartrate_3_hal.su ./afe_lib/Src/heartrate_3_hw.d ./afe_lib/Src/heartrate_3_hw.o ./afe_lib/Src/heartrate_3_hw.su

.PHONY: clean-afe_lib-2f-Src

