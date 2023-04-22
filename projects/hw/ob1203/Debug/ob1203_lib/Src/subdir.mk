################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ob1203_lib/Src/BMX055.c \
../ob1203_lib/Src/f103.c \
../ob1203_lib/Src/heartrate11.c 

OBJS += \
./ob1203_lib/Src/BMX055.o \
./ob1203_lib/Src/f103.o \
./ob1203_lib/Src/heartrate11.o 

C_DEPS += \
./ob1203_lib/Src/BMX055.d \
./ob1203_lib/Src/f103.d \
./ob1203_lib/Src/heartrate11.d 


# Each subdirectory must supply rules for building sources it contributes
ob1203_lib/Src/%.o ob1203_lib/Src/%.su: ../ob1203_lib/Src/%.c ob1203_lib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -DHAL_STM32F103 -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../ob1203_lib/Inc -I../Buffer/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ob1203_lib-2f-Src

clean-ob1203_lib-2f-Src:
	-$(RM) ./ob1203_lib/Src/BMX055.d ./ob1203_lib/Src/BMX055.o ./ob1203_lib/Src/BMX055.su ./ob1203_lib/Src/f103.d ./ob1203_lib/Src/f103.o ./ob1203_lib/Src/f103.su ./ob1203_lib/Src/heartrate11.d ./ob1203_lib/Src/heartrate11.o ./ob1203_lib/Src/heartrate11.su

.PHONY: clean-ob1203_lib-2f-Src

