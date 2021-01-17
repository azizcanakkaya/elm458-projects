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
Project3V0.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Project3V0.list \

OBJCOPY_BIN += \
Project3V0.bin \


# All Target
all: Project3V0.elf secondary-outputs

# Tool invocations
Project3V0.elf: $(OBJS) $(USER_OBJS) D:\PROGRAMLAR\STM32CubeIDE\WorkSpace/flash/STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "Project3V0.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\PROGRAMLAR\STM32CubeIDE\WorkSpace/flash/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Project3V0.map" -Wl,--cref -Wl,--gc-sections -static -L"D:\PROGRAMLAR\STM32CubeIDE\WorkSpace\STM32Cube_FW_H7_V1.7.0\Middlewares\ST\STM32_Audio\Addons\PDM\Lib" --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Project3V0.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Project3V0.list"
	@echo 'Finished building: $@'
	@echo ' '

Project3V0.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Project3V0.bin"
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
