################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Drivers/Barometer.c \
../Core/Src/Drivers/Buzzer.c \
../Core/Src/Drivers/SDCard.c 

OBJS += \
./Core/Src/Drivers/Barometer.o \
./Core/Src/Drivers/Buzzer.o \
./Core/Src/Drivers/SDCard.o 

C_DEPS += \
./Core/Src/Drivers/Barometer.d \
./Core/Src/Drivers/Buzzer.d \
./Core/Src/Drivers/SDCard.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Drivers/%.o Core/Src/Drivers/%.su Core/Src/Drivers/%.cyclo: ../Core/Src/Drivers/%.c Core/Src/Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Core/bmp280 -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"/Users/gabrielmoblicci/STM32CubeIDE/workspace_1.17.0/Projeto_CubeSat/FATFS/Target" -I../FATFS/Target -I../Core/Inc/Drivers -I../Core/Inc/Service -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Drivers

clean-Core-2f-Src-2f-Drivers:
	-$(RM) ./Core/Src/Drivers/Barometer.cyclo ./Core/Src/Drivers/Barometer.d ./Core/Src/Drivers/Barometer.o ./Core/Src/Drivers/Barometer.su ./Core/Src/Drivers/Buzzer.cyclo ./Core/Src/Drivers/Buzzer.d ./Core/Src/Drivers/Buzzer.o ./Core/Src/Drivers/Buzzer.su ./Core/Src/Drivers/SDCard.cyclo ./Core/Src/Drivers/SDCard.d ./Core/Src/Drivers/SDCard.o ./Core/Src/Drivers/SDCard.su

.PHONY: clean-Core-2f-Src-2f-Drivers

