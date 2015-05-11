################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Algos/Utils/ParamUtils.cpp \
../Algos/Utils/Utils.cpp 

OBJS += \
./Algos/Utils/ParamUtils.o \
./Algos/Utils/Utils.o 

CPP_DEPS += \
./Algos/Utils/ParamUtils.d \
./Algos/Utils/Utils.d 


# Each subdirectory must supply rules for building sources it contributes
Algos/Utils/%.o: ../Algos/Utils/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++11 -D__cplusplus=201103L -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


