################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU ARM Embedded (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/microros_transports/dma_transport.c 

OBJS += \
./Core/Src/microros_transports/dma_transport.o 

C_DEPS += \
./Core/Src/microros_transports/dma_transport.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/microros_transports/dma_transport.o: ../Core/Src/microros_transports/dma_transport.c Core/Src/microros_transports/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I"/home/aludey/Desktop/STM32IDEWS/project/Core/Inc/libmicroros/include" -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Src/microros_transports/dma_transport.d" -MT"$@"  -mfpu=fpv5-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

