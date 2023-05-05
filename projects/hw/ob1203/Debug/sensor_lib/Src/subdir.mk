################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../sensor_lib/Src/bmx055.c \
../sensor_lib/Src/f103.c \
../sensor_lib/Src/heartrate11.c 

OBJS += \
./sensor_lib/Src/bmx055.o \
./sensor_lib/Src/f103.o \
./sensor_lib/Src/heartrate11.o 

C_DEPS += \
./sensor_lib/Src/bmx055.d \
./sensor_lib/Src/f103.d \
./sensor_lib/Src/heartrate11.d 


# Each subdirectory must supply rules for building sources it contributes
sensor_lib/Src/%.o sensor_lib/Src/%.su sensor_lib/Src/%.cyclo: ../sensor_lib/Src/%.c sensor_lib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -DHAL_STM32F103 -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Buffer/Inc -I../sensor_lib/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-sensor_lib-2f-Src

clean-sensor_lib-2f-Src:
	-$(RM) ./sensor_lib/Src/bmx055.cyclo ./sensor_lib/Src/bmx055.d ./sensor_lib/Src/bmx055.o ./sensor_lib/Src/bmx055.su ./sensor_lib/Src/f103.cyclo ./sensor_lib/Src/f103.d ./sensor_lib/Src/f103.o ./sensor_lib/Src/f103.su ./sensor_lib/Src/heartrate11.cyclo ./sensor_lib/Src/heartrate11.d ./sensor_lib/Src/heartrate11.o ./sensor_lib/Src/heartrate11.su

.PHONY: clean-sensor_lib-2f-Src

