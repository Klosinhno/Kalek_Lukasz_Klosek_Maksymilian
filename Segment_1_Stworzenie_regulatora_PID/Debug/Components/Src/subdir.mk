################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Components/Src/bh1750.c \
../Components/Src/bh1750_config.c \
../Components/Src/led_pwm.c \
../Components/Src/led_pwm_config.c \
../Components/Src/pid_controller.c \
../Components/Src/pid_controller_config.c 

OBJS += \
./Components/Src/bh1750.o \
./Components/Src/bh1750_config.o \
./Components/Src/led_pwm.o \
./Components/Src/led_pwm_config.o \
./Components/Src/pid_controller.o \
./Components/Src/pid_controller_config.o 

C_DEPS += \
./Components/Src/bh1750.d \
./Components/Src/bh1750_config.d \
./Components/Src/led_pwm.d \
./Components/Src/led_pwm_config.d \
./Components/Src/pid_controller.d \
./Components/Src/pid_controller_config.d 


# Each subdirectory must supply rules for building sources it contributes
Components/Src/%.o Components/Src/%.su Components/Src/%.cyclo: ../Components/Src/%.c Components/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/kalek/OneDrive/Pulpit/L05_Example/Components/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Components-2f-Src

clean-Components-2f-Src:
	-$(RM) ./Components/Src/bh1750.cyclo ./Components/Src/bh1750.d ./Components/Src/bh1750.o ./Components/Src/bh1750.su ./Components/Src/bh1750_config.cyclo ./Components/Src/bh1750_config.d ./Components/Src/bh1750_config.o ./Components/Src/bh1750_config.su ./Components/Src/led_pwm.cyclo ./Components/Src/led_pwm.d ./Components/Src/led_pwm.o ./Components/Src/led_pwm.su ./Components/Src/led_pwm_config.cyclo ./Components/Src/led_pwm_config.d ./Components/Src/led_pwm_config.o ./Components/Src/led_pwm_config.su ./Components/Src/pid_controller.cyclo ./Components/Src/pid_controller.d ./Components/Src/pid_controller.o ./Components/Src/pid_controller.su ./Components/Src/pid_controller_config.cyclo ./Components/Src/pid_controller_config.d ./Components/Src/pid_controller_config.o ./Components/Src/pid_controller_config.su

.PHONY: clean-Components-2f-Src

