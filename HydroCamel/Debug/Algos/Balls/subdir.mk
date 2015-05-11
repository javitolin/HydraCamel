################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Algos/Balls/Balls.cpp \
../Algos/Balls/onMouse.cpp 

OBJS += \
./Algos/Balls/Balls.o \
./Algos/Balls/onMouse.o 

CPP_DEPS += \
./Algos/Balls/Balls.d \
./Algos/Balls/onMouse.d 


# Each subdirectory must supply rules for building sources it contributes
Algos/Balls/%.o: ../Algos/Balls/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -D__cplusplus=201103L -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


