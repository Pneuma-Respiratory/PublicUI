################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/fingerprint.c \
../Src/freertos.c \
../Src/i2c.c \
../Src/led_thread.c \
../Src/libbarley.c \
../Src/main.c \
../Src/platform_api.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_hal_timebase_tim.c \
../Src/stm32l4xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32l4xx.c \
../Src/tim.c \
../Src/usart.c 

OBJS += \
./Src/adc.o \
./Src/fingerprint.o \
./Src/freertos.o \
./Src/i2c.o \
./Src/led_thread.o \
./Src/libbarley.o \
./Src/main.o \
./Src/platform_api.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_hal_timebase_tim.o \
./Src/stm32l4xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32l4xx.o \
./Src/tim.o \
./Src/usart.o 

C_DEPS += \
./Src/adc.d \
./Src/fingerprint.d \
./Src/freertos.d \
./Src/i2c.d \
./Src/led_thread.d \
./Src/libbarley.d \
./Src/main.d \
./Src/platform_api.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_hal_timebase_tim.d \
./Src/stm32l4xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32l4xx.d \
./Src/tim.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L452xx -DDUAL_TEMPLATE -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Inc -I"C:/Users/rapps/git/BlueSkyGUI/workspace_STM/BlueSky4.1_LinearTech_ST/libs/inc" -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/adc.d ./Src/adc.o ./Src/adc.su ./Src/fingerprint.d ./Src/fingerprint.o ./Src/fingerprint.su ./Src/freertos.d ./Src/freertos.o ./Src/freertos.su ./Src/i2c.d ./Src/i2c.o ./Src/i2c.su ./Src/led_thread.d ./Src/led_thread.o ./Src/led_thread.su ./Src/libbarley.d ./Src/libbarley.o ./Src/libbarley.su ./Src/main.d ./Src/main.o ./Src/main.su ./Src/platform_api.d ./Src/platform_api.o ./Src/platform_api.su ./Src/stm32l4xx_hal_msp.d ./Src/stm32l4xx_hal_msp.o ./Src/stm32l4xx_hal_msp.su ./Src/stm32l4xx_hal_timebase_tim.d ./Src/stm32l4xx_hal_timebase_tim.o ./Src/stm32l4xx_hal_timebase_tim.su ./Src/stm32l4xx_it.d ./Src/stm32l4xx_it.o ./Src/stm32l4xx_it.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32l4xx.d ./Src/system_stm32l4xx.o ./Src/system_stm32l4xx.su ./Src/tim.d ./Src/tim.o ./Src/tim.su ./Src/usart.d ./Src/usart.o ./Src/usart.su

.PHONY: clean-Src

