################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../Old/Build/CMakeFiles/feature_tests.cxx 

C_SRCS += \
../Old/Build/CMakeFiles/feature_tests.c 

OBJS += \
./Old/Build/CMakeFiles/feature_tests.o 

CXX_DEPS += \
./Old/Build/CMakeFiles/feature_tests.d 

C_DEPS += \
./Old/Build/CMakeFiles/feature_tests.d 


# Each subdirectory must supply rules for building sources it contributes
Old/Build/CMakeFiles/%.o: ../Old/Build/CMakeFiles/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Old/Build/CMakeFiles/%.o: ../Old/Build/CMakeFiles/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I/mnt/BigSlowBoi/DOCUMENTS/Projects/Eclipse/Eigen/eigen-3.3.9 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


