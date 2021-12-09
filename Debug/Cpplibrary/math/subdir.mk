################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Cpplibrary/math/geodesicgrid.cpp \
../Cpplibrary/math/location.cpp \
../Cpplibrary/math/math.cpp \
../Cpplibrary/math/matrix3.cpp \
../Cpplibrary/math/matrixN.cpp \
../Cpplibrary/math/matrix_alg.cpp \
../Cpplibrary/math/quaternion.cpp \
../Cpplibrary/math/vector2.cpp \
../Cpplibrary/math/vector3.cpp 

OBJS += \
./Cpplibrary/math/geodesicgrid.o \
./Cpplibrary/math/location.o \
./Cpplibrary/math/math.o \
./Cpplibrary/math/matrix3.o \
./Cpplibrary/math/matrixN.o \
./Cpplibrary/math/matrix_alg.o \
./Cpplibrary/math/quaternion.o \
./Cpplibrary/math/vector2.o \
./Cpplibrary/math/vector3.o 

CPP_DEPS += \
./Cpplibrary/math/geodesicgrid.d \
./Cpplibrary/math/location.d \
./Cpplibrary/math/math.d \
./Cpplibrary/math/matrix3.d \
./Cpplibrary/math/matrixN.d \
./Cpplibrary/math/matrix_alg.d \
./Cpplibrary/math/quaternion.d \
./Cpplibrary/math/vector2.d \
./Cpplibrary/math/vector3.d 


# Each subdirectory must supply rules for building sources it contributes
Cpplibrary/math/%.o: ../Cpplibrary/math/%.cpp Cpplibrary/math/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32F427xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../USB_DEVICE/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Maincontroller/inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Cpplibrary/include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Clibrary/include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/mavlink -I../USB_DEVICE/App -I../FATFS/Target -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Cpplibrary-2f-math

clean-Cpplibrary-2f-math:
	-$(RM) ./Cpplibrary/math/geodesicgrid.d ./Cpplibrary/math/geodesicgrid.o ./Cpplibrary/math/location.d ./Cpplibrary/math/location.o ./Cpplibrary/math/math.d ./Cpplibrary/math/math.o ./Cpplibrary/math/matrix3.d ./Cpplibrary/math/matrix3.o ./Cpplibrary/math/matrixN.d ./Cpplibrary/math/matrixN.o ./Cpplibrary/math/matrix_alg.d ./Cpplibrary/math/matrix_alg.o ./Cpplibrary/math/quaternion.d ./Cpplibrary/math/quaternion.o ./Cpplibrary/math/vector2.d ./Cpplibrary/math/vector2.o ./Cpplibrary/math/vector3.d ./Cpplibrary/math/vector3.o

.PHONY: clean-Cpplibrary-2f-math

