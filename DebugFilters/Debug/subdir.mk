################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../BaseAlgorithm.cpp \
../BlackBoxDebugAlgorithm.cpp \
../GateDebugAlgorithm.cpp \
../main.cpp 

OBJS += \
./BaseAlgorithm.o \
./BlackBoxDebugAlgorithm.o \
./GateDebugAlgorithm.o \
./main.o 

CPP_DEPS += \
./BaseAlgorithm.d \
./BlackBoxDebugAlgorithm.d \
./GateDebugAlgorithm.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


