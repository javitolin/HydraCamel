################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Algos/Torpedo/TorpedoAlgo.cpp 

OBJS += \
./Algos/Torpedo/TorpedoAlgo.o 

CPP_DEPS += \
./Algos/Torpedo/TorpedoAlgo.d 


# Each subdirectory must supply rules for building sources it contributes
Algos/Torpedo/%.o: ../Algos/Torpedo/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -D__cplusplus=201103L -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


