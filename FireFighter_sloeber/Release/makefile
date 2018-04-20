################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ..\makefile.init

RM := del 

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include libraries\Wire\src\utility\subdir.mk
-include libraries\Wire\src\subdir.mk
-include libraries\Ultrasonic\src\subdir.mk
-include libraries\QTRSensors\subdir.mk
-include libraries\L3G\subdir.mk
-include libraries\Encoder\subdir.mk
-include core\core\subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ..\makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
C:\Users\Lingrui\Desktop\zlr\school-docs\WPI2017-18\D\RBE2002\Code\RBE2002\FireFighter_sloeber\Release\arduino.ar \

ELF += \
FireFighter_sloeber.elf \

HEX += \
FireFighter_sloeber.hex \


# All Target
all: FireFighter_sloeber

# Tool invocations
FireFighter_sloeber: $(HEX)
	@echo 'Building target: $@'
	@echo 'Printing size:'
	"C:\Users\Lingrui\Desktop\sloeber\/arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-size" -A "C:/Users/Lingrui/Desktop/zlr/school-docs/WPI2017-18/D/RBE2002/Code/RBE2002/FireFighter_sloeber/Release/FireFighter_sloeber.elf"
	@echo 'Finished building target: $@'
	@echo ' '

C:\Users\Lingrui\Desktop\zlr\school-docs\WPI2017-18\D\RBE2002\Code\RBE2002\FireFighter_sloeber\Release\arduino.ar: $(AR_OBJ)
	@echo 'Starting archiver'
	"C:\Users\Lingrui\Desktop\sloeber\/arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-gcc-ar" rcs    C:/Users/Lingrui/Desktop/zlr/school-docs/WPI2017-18/D/RBE2002/Code/RBE2002/FireFighter_sloeber/Release/arduino.ar $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

FireFighter_sloeber.elf: $(LINK_OBJ) $(AR)
	@echo 'Starting combiner'
	"C:\Users\Lingrui\Desktop\sloeber\/arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-gcc" -Wall -Wextra -Os -Wl,--gc-sections,--relax -mmcu=atmega2560 -o "C:\Users\Lingrui\Desktop\zlr\school-docs\WPI2017-18\D\RBE2002\Code\RBE2002\FireFighter_sloeber/Release/FireFighter_sloeber.elf"   $(LINK_OBJ)   C:/Users/Lingrui/Desktop/zlr/school-docs/WPI2017-18/D/RBE2002/Code/RBE2002/FireFighter_sloeber/Release/arduino.ar  "-LC:\Users\Lingrui\Desktop\zlr\school-docs\WPI2017-18\D\RBE2002\Code\RBE2002\FireFighter_sloeber/Release" -lm
	@echo 'Finished building: $@'
	@echo ' '

FireFighter_sloeber.hex: $(ELF)
	@echo 'Create eeprom image'
	${A.RECIPE.OBJCOPY.EEP.PATTERN}
	${A.RECIPE.OBJCOPY.HEX.PATTERN}
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(PDE_DEPS)$(C_DEPS)$(AR)$(CC_DEPS)$(AR_OBJ)$(C++_DEPS)$(LINK_OBJ)$(CXX_DEPS)$(ASM_DEPS)$(HEX)$(INO_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(ELF)$(CPP_DEPS)
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ..\makefile.targets
