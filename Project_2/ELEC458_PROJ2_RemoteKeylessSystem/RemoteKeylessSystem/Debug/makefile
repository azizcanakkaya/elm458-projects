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
RemoteKeylessSystem.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RemoteKeylessSystem.list \

OBJCOPY_BIN += \
RemoteKeylessSystem.bin \


# All Target
all: RemoteKeylessSystem.elf secondary-outputs

# Tool invocations
RemoteKeylessSystem.elf: $(OBJS) $(USER_OBJS) D:\PROGRAMLAR\STM32CubeIDE\WorkSpace/flash/STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "RemoteKeylessSystem.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\PROGRAMLAR\STM32CubeIDE\WorkSpace/flash/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="RemoteKeylessSystem.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RemoteKeylessSystem.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RemoteKeylessSystem.list"
	@echo 'Finished building: $@'
	@echo ' '

RemoteKeylessSystem.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "RemoteKeylessSystem.bin"
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
