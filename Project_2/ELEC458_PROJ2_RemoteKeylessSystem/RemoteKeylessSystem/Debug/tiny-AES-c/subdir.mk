################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tiny-AES-c/test.c 

OBJS += \
./tiny-AES-c/aes.o \
./tiny-AES-c/test.o 

C_DEPS += \
./tiny-AES-c/aes.d \
./tiny-AES-c/test.d 


# Each subdirectory must supply rules for building sources it contributes
tiny-AES-c/aes.o: ../tiny-AES-c/aes.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407xx -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I"D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/include" -I"D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/libs/CMSIS_5/CMSIS/Core/Include" -O0 -ffunction-sections -fdata-sections -Wall -pedantic -Wcast-align -Wsign-compare -fstack-usage -MMD -MP -MF"tiny-AES-c/aes.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
tiny-AES-c/test.o: ../tiny-AES-c/test.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407xx -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I"D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/include" -I"D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/libs/CMSIS_5/CMSIS/Core/Include" -O0 -ffunction-sections -fdata-sections -Wall -pedantic -Wcast-align -Wsign-compare -fstack-usage -MMD -MP -MF"tiny-AES-c/test.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

