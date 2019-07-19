################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/HAL/app_fw.c \
../Application/User/HAL/battery.c \
../Application/User/HAL/delay.c \
../Application/User/HAL/imu.c \
../Application/User/HAL/itoa.c \
../Application/User/HAL/led.c \
../Application/User/HAL/serial.c 

OBJS += \
./Application/User/HAL/app_fw.o \
./Application/User/HAL/battery.o \
./Application/User/HAL/delay.o \
./Application/User/HAL/imu.o \
./Application/User/HAL/itoa.o \
./Application/User/HAL/led.o \
./Application/User/HAL/serial.o 

C_DEPS += \
./Application/User/HAL/app_fw.d \
./Application/User/HAL/battery.d \
./Application/User/HAL/delay.d \
./Application/User/HAL/imu.d \
./Application/User/HAL/itoa.d \
./Application/User/HAL/led.d \
./Application/User/HAL/serial.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/HAL/%.o: ../Application/User/HAL/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c11 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F415xx -I../../../Inc -I../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../../Drivers/CMSIS/Include -I../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I"C:/devrobot/Antichavi/SW4STM32/Antichavi Configuration/Application/User/HAL" -I"C:/devrobot/Antichavi/SW4STM32/Antichavi Configuration/Application/User/APP" -I"C:/devrobot/Antichavi/SW4STM32/Antichavi Configuration/Application/User/IMU" -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


