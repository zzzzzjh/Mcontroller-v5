################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Clibrary/gnss/gps.c \
../Clibrary/gnss/ublox.c 

C_DEPS += \
./Clibrary/gnss/gps.d \
./Clibrary/gnss/ublox.d 

OBJS += \
./Clibrary/gnss/gps.o \
./Clibrary/gnss/ublox.o 


# Each subdirectory must supply rules for building sources it contributes
Clibrary/gnss/%.o: ../Clibrary/gnss/%.c Clibrary/gnss/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F427xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../USB_DEVICE/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Clibrary/include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/mavlink -I../USB_DEVICE/App -I../FATFS/Target -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -mslow-flash-data -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Clibrary-2f-gnss

clean-Clibrary-2f-gnss:
	-$(RM) ./Clibrary/gnss/gps.d ./Clibrary/gnss/gps.o ./Clibrary/gnss/ublox.d ./Clibrary/gnss/ublox.o

.PHONY: clean-Clibrary-2f-gnss

