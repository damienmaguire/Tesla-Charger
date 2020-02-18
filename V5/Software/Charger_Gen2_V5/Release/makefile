################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
Charger_Gen2_V5.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Charger_Gen2_V5.list \


# All Target
all: Charger_Gen2_V5.elf secondary-outputs

# Tool invocations
Charger_Gen2_V5.elf: $(OBJS) $(USER_OBJS) C:\Users\gsli_\STM32CubeIDE\workspace_1.1.0\Charger_Gen2_V5\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Charger_Gen2_V5.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\gsli_\STM32CubeIDE\workspace_1.1.0\Charger_Gen2_V5\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Charger_Gen2_V5.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Charger_Gen2_V5.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Charger_Gen2_V5.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
