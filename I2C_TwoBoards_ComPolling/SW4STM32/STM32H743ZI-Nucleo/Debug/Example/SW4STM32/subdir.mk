################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/HarryFu/STM32_Platform/STM32CubeIDE_workspace/I2C_TwoBoards_ComPolling/SW4STM32/startup_stm32h743xx.s 

OBJS += \
./Example/SW4STM32/startup_stm32h743xx.o 

S_DEPS += \
./Example/SW4STM32/startup_stm32h743xx.d 


# Each subdirectory must supply rules for building sources it contributes
Example/SW4STM32/startup_stm32h743xx.o: C:/HarryFu/STM32_Platform/STM32CubeIDE_workspace/I2C_TwoBoards_ComPolling/SW4STM32/startup_stm32h743xx.s Example/SW4STM32/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Example/SW4STM32/startup_stm32h743xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

