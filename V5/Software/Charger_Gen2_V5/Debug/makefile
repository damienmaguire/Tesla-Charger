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

OBJCOPY_BIN += \
Charger_Gen2_V5.bin \


# All Target
all: Charger_Gen2_V5.elf secondary-outputs

# Tool invocations
Charger_Gen2_V5.elf: $(OBJS) $(USER_OBJS) C:\Users\gaming\Dropbox\damo\Tesla\Charger\V5_Software\Charger_Gen2_V5\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Charger_Gen2_V5.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\gaming\Dropbox\damo\Tesla\Charger\V5_Software\Charger_Gen2_V5\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Charger_Gen2_V5.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
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

Charger_Gen2_V5.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Charger_Gen2_V5.bin"
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
