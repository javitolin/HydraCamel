################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Algos/Shadow/ShadowAlgorithm.cpp 

OBJS += \
./Algos/Shadow/ShadowAlgorithm.o 

CPP_DEPS += \
./Algos/Shadow/ShadowAlgorithm.d 


# Each subdirectory must supply rules for building sources it contributes
Algos/Shadow/%.o: ../Algos/Shadow/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


