################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/CreatedFilter.cpp \
../src/FilterHandler.cpp \
../src/FilterRun.cpp \
../src/Log.cpp \
../src/VideoStream.cpp \
../src/main.cpp 

OBJS += \
./src/CreatedFilter.o \
./src/FilterHandler.o \
./src/FilterRun.o \
./src/Log.o \
./src/VideoStream.o \
./src/main.o 

CPP_DEPS += \
./src/CreatedFilter.d \
./src/FilterHandler.d \
./src/FilterRun.d \
./src/Log.d \
./src/VideoStream.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


