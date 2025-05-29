################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Service/LoggerService.c 

OBJS += \
./Core/Src/Service/LoggerService.o 

C_DEPS += \
./Core/Src/Service/LoggerService.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Service/%.o Core/Src/Service/%.su Core/Src/Service/%.cyclo: ../Core/Src/Service/%.c Core/Src/Service/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Core/bmp280 -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"/Users/gabrielmoblicci/STM32CubeIDE/workspace_1.17.0/Projeto_CubeSat/FATFS/Target" -I../FATFS/Target -I../Core/Inc/Drivers -I../Core/Inc/Service -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Service

clean-Core-2f-Src-2f-Service:
	-$(RM) ./Core/Src/Service/LoggerService.cyclo ./Core/Src/Service/LoggerService.d ./Core/Src/Service/LoggerService.o ./Core/Src/Service/LoggerService.su

.PHONY: clean-Core-2f-Src-2f-Service

