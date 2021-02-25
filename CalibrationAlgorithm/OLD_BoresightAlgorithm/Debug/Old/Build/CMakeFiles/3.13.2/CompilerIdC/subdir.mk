################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Old/Build/CMakeFiles/3.13.2/CompilerIdC/CMakeCCompilerId.c 

OBJS += \
./Old/Build/CMakeFiles/3.13.2/CompilerIdC/CMakeCCompilerId.o 

C_DEPS += \
./Old/Build/CMakeFiles/3.13.2/CompilerIdC/CMakeCCompilerId.d 


# Each subdirectory must supply rules for building sources it contributes
Old/Build/CMakeFiles/3.13.2/CompilerIdC/%.o: ../Old/Build/CMakeFiles/3.13.2/CompilerIdC/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


