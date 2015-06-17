################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Algos/ThirdTask/ThirdTaskBlackBox.cpp 

OBJS += \
./Algos/ThirdTask/ThirdTaskBlackBox.o 

CPP_DEPS += \
./Algos/ThirdTask/ThirdTaskBlackBox.d 


# Each subdirectory must supply rules for building sources it contributes
Algos/ThirdTask/%.o: ../Algos/ThirdTask/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


