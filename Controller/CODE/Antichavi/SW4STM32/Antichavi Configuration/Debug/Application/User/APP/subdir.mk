################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/APP/app_main.c \
../Application/User/APP/ihm.c \
../Application/User/APP/threshold.c 

OBJS += \
./Application/User/APP/app_main.o \
./Application/User/APP/ihm.o \
./Application/User/APP/threshold.o 

C_DEPS += \
./Application/User/APP/app_main.d \
./Application/User/APP/ihm.d \
./Application/User/APP/threshold.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/APP/%.o: ../Application/User/APP/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c11 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F415xx -I../../../Inc -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/devrobot/Antichavi/SW4STM32/Antichavi Configuration/Application/User/HAL" -I"C:/devrobot/Antichavi/SW4STM32/Antichavi Configuration/Application/User/APP" -I"C:/devrobot/Antichavi/SW4STM32/Antichavi Configuration/Application/User/IMU" -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


