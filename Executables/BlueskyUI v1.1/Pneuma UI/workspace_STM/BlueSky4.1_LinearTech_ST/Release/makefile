################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := BlueSky4.1_LinearTech_ST
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
BlueSky4.1_LinearTech_ST.elf \

MAP_FILES += \
BlueSky4.1_LinearTech_ST.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
BlueSky4.1_LinearTech_ST.list \

OBJCOPY_HEX += \
BlueSky4.1_LinearTech_ST.hex \

OBJCOPY_BIN += \
BlueSky4.1_LinearTech_ST.bin \


# All Target
all: main-build

# Main-build Target
main-build: BlueSky4.1_LinearTech_ST.elf secondary-outputs

# Tool invocations
BlueSky4.1_LinearTech_ST.elf BlueSky4.1_LinearTech_ST.map: $(OBJS) $(USER_OBJS) C:\Users\rapps\git\BlueSkyGUI\workspace_STM\BlueSky4.1_LinearTech_ST\STM32L452RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "BlueSky4.1_LinearTech_ST.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\rapps\git\BlueSkyGUI\workspace_STM\BlueSky4.1_LinearTech_ST\STM32L452RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="BlueSky4.1_LinearTech_ST.map" -Wl,--gc-sections -static -L"C:\Users\rapps\git\BlueSkyGUI\workspace_STM\BlueSky4.1_LinearTech_ST\libs" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

BlueSky4.1_LinearTech_ST.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "BlueSky4.1_LinearTech_ST.list"
	@echo 'Finished building: $@'
	@echo ' '

BlueSky4.1_LinearTech_ST.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "BlueSky4.1_LinearTech_ST.hex"
	@echo 'Finished building: $@'
	@echo ' '

BlueSky4.1_LinearTech_ST.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "BlueSky4.1_LinearTech_ST.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) BlueSky4.1_LinearTech_ST.bin BlueSky4.1_LinearTech_ST.elf BlueSky4.1_LinearTech_ST.hex BlueSky4.1_LinearTech_ST.list BlueSky4.1_LinearTech_ST.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
