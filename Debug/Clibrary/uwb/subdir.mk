################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Clibrary/uwb/deca_device.c \
../Clibrary/uwb/deca_params_init.c \
../Clibrary/uwb/deca_range_tables.c \
../Clibrary/uwb/deca_spi.c \
../Clibrary/uwb/trilateration.c \
../Clibrary/uwb/uwb.c 

OBJS += \
./Clibrary/uwb/deca_device.o \
./Clibrary/uwb/deca_params_init.o \
./Clibrary/uwb/deca_range_tables.o \
./Clibrary/uwb/deca_spi.o \
./Clibrary/uwb/trilateration.o \
./Clibrary/uwb/uwb.o 

C_DEPS += \
./Clibrary/uwb/deca_device.d \
./Clibrary/uwb/deca_params_init.d \
./Clibrary/uwb/deca_range_tables.d \
./Clibrary/uwb/deca_spi.d \
./Clibrary/uwb/trilateration.d \
./Clibrary/uwb/uwb.d 


# Each subdirectory must supply rules for building sources it contributes
Clibrary/uwb/%.o: ../Clibrary/uwb/%.c Clibrary/uwb/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F427xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../FATFS/App -I../USB_DEVICE/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Clibrary/include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/mavlink -I../USB_DEVICE/App -I../FATFS/Target -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -mslow-flash-data -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -o "$@"

clean: clean-Clibrary-2f-uwb

clean-Clibrary-2f-uwb:
	-$(RM) ./Clibrary/uwb/deca_device.d ./Clibrary/uwb/deca_device.o ./Clibrary/uwb/deca_params_init.d ./Clibrary/uwb/deca_params_init.o ./Clibrary/uwb/deca_range_tables.d ./Clibrary/uwb/deca_range_tables.o ./Clibrary/uwb/deca_spi.d ./Clibrary/uwb/deca_spi.o ./Clibrary/uwb/trilateration.d ./Clibrary/uwb/trilateration.o ./Clibrary/uwb/uwb.d ./Clibrary/uwb/uwb.o

.PHONY: clean-Clibrary-2f-uwb

