################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Cpplibrary/pid/p.cpp \
../Cpplibrary/pid/pid.cpp \
../Cpplibrary/pid/pid_2d.cpp 

OBJS += \
./Cpplibrary/pid/p.o \
./Cpplibrary/pid/pid.o \
./Cpplibrary/pid/pid_2d.o 

CPP_DEPS += \
./Cpplibrary/pid/p.d \
./Cpplibrary/pid/pid.d \
./Cpplibrary/pid/pid_2d.d 


# Each subdirectory must supply rules for building sources it contributes
Cpplibrary/pid/%.o: ../Cpplibrary/pid/%.cpp Cpplibrary/pid/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32F427xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../USB_DEVICE/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Maincontroller/inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Cpplibrary/include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Clibrary/include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/mavlink -I../USB_DEVICE/App -I../FATFS/Target -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Cpplibrary-2f-pid

clean-Cpplibrary-2f-pid:
	-$(RM) ./Cpplibrary/pid/p.d ./Cpplibrary/pid/p.o ./Cpplibrary/pid/pid.d ./Cpplibrary/pid/pid.o ./Cpplibrary/pid/pid_2d.d ./Cpplibrary/pid/pid_2d.o

.PHONY: clean-Cpplibrary-2f-pid

