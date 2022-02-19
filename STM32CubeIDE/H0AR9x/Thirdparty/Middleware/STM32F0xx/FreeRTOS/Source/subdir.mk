################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/FreeRTOS_CLI.c \
D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/croutine.c \
D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/event_groups.c \
D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/list.c \
D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/queue.c \
D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/tasks.c \
D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/timers.c 

OBJS += \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/FreeRTOS_CLI.o \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/croutine.o \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/event_groups.o \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/list.o \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/queue.o \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/tasks.o \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/timers.o 

C_DEPS += \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/FreeRTOS_CLI.d \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/croutine.d \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/event_groups.d \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/list.d \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/queue.d \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/tasks.d \
./Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/FreeRTOS_CLI.o: D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/FreeRTOS_CLI.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../H0AR9 -I../../BOS -I../../User -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/FreeRTOS_CLI.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/croutine.o: D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/croutine.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../H0AR9 -I../../BOS -I../../User -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/croutine.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/event_groups.o: D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/event_groups.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../H0AR9 -I../../BOS -I../../User -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/event_groups.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/list.o: D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/list.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../H0AR9 -I../../BOS -I../../User -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/list.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/queue.o: D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/queue.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../H0AR9 -I../../BOS -I../../User -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/queue.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/tasks.o: D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/tasks.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../H0AR9 -I../../BOS -I../../User -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/tasks.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/timers.o: D:/Hexabitz\ release/H0AR9x/Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../H0AR9 -I../../BOS -I../../User -I../../Thirdparty/CMSIS/STM32F0xx/Include -I../../Thirdparty/CMSIS/STM32F0xx/Device/ST/STM32F0xx/Include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/include -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/CMSIS_RTOS -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"Thirdparty/Middleware/STM32F0xx/FreeRTOS/Source/timers.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"

