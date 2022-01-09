################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
D:/Hexabitz\ release/H0AR9x/H0AR9/startup_stm32f091xc.s 

C_SRCS += \
D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9.c \
D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_dma.c \
D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_gpio.c \
D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_i2c.c \
D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_it.c \
D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_rtc.c \
D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_timers.c \
D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_uart.c 

OBJS += \
./H0AR9/H0AR9.o \
./H0AR9/H0AR9_dma.o \
./H0AR9/H0AR9_gpio.o \
./H0AR9/H0AR9_i2c.o \
./H0AR9/H0AR9_it.o \
./H0AR9/H0AR9_rtc.o \
./H0AR9/H0AR9_timers.o \
./H0AR9/H0AR9_uart.o \
./H0AR9/startup_stm32f091xc.o 

S_DEPS += \
./H0AR9/startup_stm32f091xc.d 

C_DEPS += \
./H0AR9/H0AR9.d \
./H0AR9/H0AR9_dma.d \
./H0AR9/H0AR9_gpio.d \
./H0AR9/H0AR9_i2c.d \
./H0AR9/H0AR9_it.d \
./H0AR9/H0AR9_rtc.d \
./H0AR9/H0AR9_timers.d \
./H0AR9/H0AR9_uart.d 


# Each subdirectory must supply rules for building sources it contributes
H0AR9/H0AR9.o: D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0AR9 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0AR9/H0AR9.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0AR9/H0AR9_dma.o: D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_dma.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0AR9 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0AR9/H0AR9_dma.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0AR9/H0AR9_gpio.o: D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0AR9 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0AR9/H0AR9_gpio.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0AR9/H0AR9_i2c.o: D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_i2c.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0AR9 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0AR9/H0AR9_i2c.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0AR9/H0AR9_it.o: D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0AR9 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0AR9/H0AR9_it.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0AR9/H0AR9_rtc.o: D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_rtc.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0AR9 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0AR9/H0AR9_rtc.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0AR9/H0AR9_timers.o: D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0AR9 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0AR9/H0AR9_timers.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0AR9/H0AR9_uart.o: D:/Hexabitz\ release/H0AR9x/H0AR9/H0AR9_uart.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=c99 -g -DUSE_HAL_DRIVER -DDEBUG -DSTM32F091xB -DSTM32F091xC '-D_module=1' -DH0AR9 -c -I../../Thirdparty/CMSIS/Include -I../../Thirdparty/CMSIS/Device/ST/STM32F0xx/Include -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc/Legacy -I../../Thirdparty/STM32F0xx_HAL_Driver/Inc -I../../Thirdparty/Middleware/FreeRTOS/Source/include -I../../Thirdparty/Middleware/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../../Thirdparty/Middleware/FreeRTOS/Source/CMSIS_RTOS -I../../H0AR9 -I../../BOS -I../../User -O1 -ffunction-sections -fdata-sections -fstack-usage -MMD -MP -MF"H0AR9/H0AR9_uart.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@"
H0AR9/startup_stm32f091xc.o: D:/Hexabitz\ release/H0AR9x/H0AR9/startup_stm32f091xc.s
	arm-none-eabi-gcc -mcpu=cortex-m0 -g3 -c -x assembler-with-cpp -MMD -MP -MF"H0AR9/startup_stm32f091xc.d" -MT"$@"  -mfloat-abi=soft -mthumb -o "$@" "$<"

