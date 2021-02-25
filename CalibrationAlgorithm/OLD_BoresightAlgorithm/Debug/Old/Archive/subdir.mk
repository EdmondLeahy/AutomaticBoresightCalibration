################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Old/Archive/BoresightMain.cpp \
../Old/Archive/CalibrationFunctions.cpp 

OBJS += \
./Old/Archive/BoresightMain.o \
./Old/Archive/CalibrationFunctions.o 

CPP_DEPS += \
./Old/Archive/BoresightMain.d \
./Old/Archive/CalibrationFunctions.d 


# Each subdirectory must supply rules for building sources it contributes
Old/Archive/%.o: ../Old/Archive/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/mnt/BigSlowBoi/DOCUMENTS/Projects/Eclipse/Eigen/eigen-3.3.9 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


