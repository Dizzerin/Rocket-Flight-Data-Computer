################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/lsm6dso/lsm6dso32_device.c \
../Middlewares/ST/lsm6dso/lsm6dso32_reg.c 

OBJS += \
./Middlewares/ST/lsm6dso/lsm6dso32_device.o \
./Middlewares/ST/lsm6dso/lsm6dso32_reg.o 

C_DEPS += \
./Middlewares/ST/lsm6dso/lsm6dso32_device.d \
./Middlewares/ST/lsm6dso/lsm6dso32_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/lsm6dso/%.o Middlewares/ST/lsm6dso/%.su Middlewares/ST/lsm6dso/%.cyclo: ../Middlewares/ST/lsm6dso/%.c Middlewares/ST/lsm6dso/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"/home/caleb/Documents/Work/Wyzant Tutoring/Nicholas Chang PCB/STM32_SPI_SD_Card_V11/Middlewares/ST/lsm6dso" -I"/home/caleb/Documents/Work/Wyzant Tutoring/Nicholas Chang PCB/STM32_SPI_SD_Card_V11/UserCode" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-lsm6dso

clean-Middlewares-2f-ST-2f-lsm6dso:
	-$(RM) ./Middlewares/ST/lsm6dso/lsm6dso32_device.cyclo ./Middlewares/ST/lsm6dso/lsm6dso32_device.d ./Middlewares/ST/lsm6dso/lsm6dso32_device.o ./Middlewares/ST/lsm6dso/lsm6dso32_device.su ./Middlewares/ST/lsm6dso/lsm6dso32_reg.cyclo ./Middlewares/ST/lsm6dso/lsm6dso32_reg.d ./Middlewares/ST/lsm6dso/lsm6dso32_reg.o ./Middlewares/ST/lsm6dso/lsm6dso32_reg.su

.PHONY: clean-Middlewares-2f-ST-2f-lsm6dso

