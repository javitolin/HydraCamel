################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include Algos/blackGate2/subdir.mk
-include Algos/Utils/subdir.mk
-include Algos/Traffic/subdir.mk
-include Algos/Torpedo/subdir.mk
-include Algos/ThirdTask/subdir.mk
-include Algos/Shadow/subdir.mk
-include Algos/Path/subdir.mk
-include Algos/JaviTest/subdir.mk
-include Algos/Gate/subdir.mk
-include Algos/FirstTask/subdir.mk
-include Algos/Balls/subdir.mk
-include Algos/subdir.mk
-include subdir.mk
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
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: LocalHydroCamel

# Tool invocations
LocalHydroCamel: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++ -L/usr/local/lib -o "LocalHydroCamel" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(C_UPPER_DEPS)$(CXX_DEPS)$(OBJS)$(CPP_DEPS)$(C_DEPS) LocalHydroCamel
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
