################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Application/User/Startup/startup_stm32h723zgtx.s 

OBJS += \
./Application/User/Startup/startup_stm32h723zgtx.o 

S_DEPS += \
./Application/User/Startup/startup_stm32h723zgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/Startup/%.o: ../Application/User/Startup/%.s Application/User/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -I"C:/Users/abess/OneDrive/Documents/Software/AD9910_Driver/AD9910_Driver/STM32CubeIDE/Application/User/Middlewares/AD9910" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Application-2f-User-2f-Startup

clean-Application-2f-User-2f-Startup:
	-$(RM) ./Application/User/Startup/startup_stm32h723zgtx.d ./Application/User/Startup/startup_stm32h723zgtx.o

.PHONY: clean-Application-2f-User-2f-Startup

