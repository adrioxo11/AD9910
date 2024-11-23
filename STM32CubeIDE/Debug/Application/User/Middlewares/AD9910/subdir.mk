################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/Middlewares/AD9910/ad9910.c 

OBJS += \
./Application/User/Middlewares/AD9910/ad9910.o 

C_DEPS += \
./Application/User/Middlewares/AD9910/ad9910.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/Middlewares/AD9910/%.o Application/User/Middlewares/AD9910/%.su Application/User/Middlewares/AD9910/%.cyclo: ../Application/User/Middlewares/AD9910/%.c Application/User/Middlewares/AD9910/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx -c -I"C:/Users/abess/OneDrive/Documents/Software/AD9910_Driver/AD9910_Driver/STM32CubeIDE/Application/User/Middlewares/AD9910" -I../../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User-2f-Middlewares-2f-AD9910

clean-Application-2f-User-2f-Middlewares-2f-AD9910:
	-$(RM) ./Application/User/Middlewares/AD9910/ad9910.cyclo ./Application/User/Middlewares/AD9910/ad9910.d ./Application/User/Middlewares/AD9910/ad9910.o ./Application/User/Middlewares/AD9910/ad9910.su

.PHONY: clean-Application-2f-User-2f-Middlewares-2f-AD9910

