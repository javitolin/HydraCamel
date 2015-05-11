################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/CamerasController.cpp \
../src/CreatedFilter.cpp \
../src/FilterHandler.cpp \
../src/FilterRun.cpp \
../src/FilterRunThread.cpp \
../src/FrontCamera.cpp \
../src/Log.cpp \
../src/RosNetwork.cpp \
../src/VideoStream.cpp \
../src/main.cpp 

OBJS += \
./src/CamerasController.o \
./src/CreatedFilter.o \
./src/FilterHandler.o \
./src/FilterRun.o \
./src/FilterRunThread.o \
./src/FrontCamera.o \
./src/Log.o \
./src/RosNetwork.o \
./src/VideoStream.o \
./src/main.o 

CPP_DEPS += \
./src/CamerasController.d \
./src/CreatedFilter.d \
./src/FilterHandler.d \
./src/FilterRun.d \
./src/FilterRunThread.d \
./src/FrontCamera.d \
./src/Log.d \
./src/RosNetwork.d \
./src/VideoStream.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++11 -D__cplusplus=201103L -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


