################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Old/Build/CMakeFiles/3.13.2/CompilerIdCXX/CMakeCXXCompilerId.cpp 

OBJS += \
./Old/Build/CMakeFiles/3.13.2/CompilerIdCXX/CMakeCXXCompilerId.o 

CPP_DEPS += \
./Old/Build/CMakeFiles/3.13.2/CompilerIdCXX/CMakeCXXCompilerId.d 


# Each subdirectory must supply rules for building sources it contributes
Old/Build/CMakeFiles/3.13.2/CompilerIdCXX/%.o: ../Old/Build/CMakeFiles/3.13.2/CompilerIdCXX/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/mnt/BigSlowBoi/DOCUMENTS/Projects/Eclipse/Eigen/eigen-3.3.9 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


