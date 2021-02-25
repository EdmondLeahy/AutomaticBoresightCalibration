################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../BAFunctions.cpp \
../Boresight.cpp \
../BoresightMain.cpp \
../CalibrationFunctions.cpp \
../LS.cpp \
../SBA.cpp 

OBJS += \
./BAFunctions.o \
./Boresight.o \
./BoresightMain.o \
./CalibrationFunctions.o \
./LS.o \
./SBA.o 

CPP_DEPS += \
./BAFunctions.d \
./Boresight.d \
./BoresightMain.d \
./CalibrationFunctions.d \
./LS.d \
./SBA.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/mnt/BigSlowBoi/DOCUMENTS/Projects/Eclipse/Eigen/eigen-3.3.9 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


