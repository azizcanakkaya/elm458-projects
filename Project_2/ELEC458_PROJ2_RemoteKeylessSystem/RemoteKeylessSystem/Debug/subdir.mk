################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/include/startup_stm32f407vgtx.s 

C_SRCS += \
D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/tiny-AES-c/aes.c \
../main.c \
D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/include/system_stm32f4xx.c 

OBJS += \
./aes.o \
./main.o \
./startup_stm32f407vgtx.o \
./system_stm32f4xx.o 

C_DEPS += \
./aes.d \
./main.d \
./system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
aes.o: D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/tiny-AES-c/aes.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407xx -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I"D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/include" -I"D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/libs/CMSIS_5/CMSIS/Core/Include" -O0 -ffunction-sections -fdata-sections -Wall -pedantic -Wcast-align -Wsign-compare -fstack-usage -MMD -MP -MF"aes.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
main.o: ../main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407xx -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I"D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/include" -I"D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/libs/CMSIS_5/CMSIS/Core/Include" -O0 -ffunction-sections -fdata-sections -Wall -pedantic -Wcast-align -Wsign-compare -fstack-usage -MMD -MP -MF"main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
startup_stm32f407vgtx.o: D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/include/startup_stm32f407vgtx.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"
system_stm32f4xx.o: D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/include/system_stm32f4xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407xx -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I"D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/include" -I"D:/PROGRAMLAR/STM32CubeIDE/WorkSpace/libs/CMSIS_5/CMSIS/Core/Include" -O0 -ffunction-sections -fdata-sections -Wall -pedantic -Wcast-align -Wsign-compare -fstack-usage -MMD -MP -MF"system_stm32f4xx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

