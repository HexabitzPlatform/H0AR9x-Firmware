################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/MemMang/heap_4.c 

OBJS += \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/MemMang/heap_4.o 

C_DEPS += \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/MemMang/heap_4.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/MemMang/heap_4.o: D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/MemMang/heap_4.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../H0AR9 -I../../BOS -I../../User -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/MemMang/heap_4.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"

