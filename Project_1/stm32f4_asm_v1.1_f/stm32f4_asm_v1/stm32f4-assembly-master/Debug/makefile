################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
f4-asm.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
f4-asm.list \

OBJCOPY_BIN += \
f4-asm.bin \


# All Target
all: f4-asm.elf secondary-outputs

# Tool invocations
f4-asm.elf: $(OBJS) $(USER_OBJS) D:\Dosyalar\Ders\Embedded\ Systems\Projects\Project_1\stm32f4_asm_v1.1_f\stm32f4_asm_v1\stm32f4-assembly-master\stm32.ld
	arm-none-eabi-gcc -o "f4-asm.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Dosyalar\Ders\Embedded Systems\Projects\Project_1\stm32f4_asm_v1.1_f\stm32f4_asm_v1\stm32f4-assembly-master\stm32.ld" --specs=nosys.specs -Wl,-Map="f4-asm.map" -nostartfiles -static --specs=nano.specs -mfloat-abi=soft -mthumb
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

f4-asm.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "f4-asm.list"
	@echo 'Finished building: $@'
	@echo ' '

f4-asm.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "f4-asm.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
