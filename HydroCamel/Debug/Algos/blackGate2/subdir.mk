################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Algos/blackGate2/BlackGate_adaptive.cpp 

OBJS += \
./Algos/blackGate2/BlackGate_adaptive.o 

CPP_DEPS += \
./Algos/blackGate2/BlackGate_adaptive.d 


# Each subdirectory must supply rules for building sources it contributes
Algos/blackGate2/%.o: ../Algos/blackGate2/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++11 -D__cplusplus=201103L -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


