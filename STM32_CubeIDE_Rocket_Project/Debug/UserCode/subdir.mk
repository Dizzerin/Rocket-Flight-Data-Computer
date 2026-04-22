################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../UserCode/DataLogger.c \
../UserCode/SD_Card.c \
../UserCode/Scheduler.c \
../UserCode/bme680_device.c 

OBJS += \
./UserCode/DataLogger.o \
./UserCode/SD_Card.o \
./UserCode/Scheduler.o \
./UserCode/bme680_device.o 

C_DEPS += \
./UserCode/DataLogger.d \
./UserCode/SD_Card.d \
./UserCode/Scheduler.d \
./UserCode/bme680_device.d 


# Each subdirectory must supply rules for building sources it contributes
UserCode/%.o UserCode/%.su UserCode/%.cyclo: ../UserCode/%.c UserCode/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"/home/caleb/Documents/Work/Wyzant_Tutoring/Nicholas_Chang_PCB/Rocket-Flight-Data-Computer/STM32_CubeIDE_Rocket_Project/Middlewares/ST/lsm6dso" -I"/home/caleb/Documents/Work/Wyzant_Tutoring/Nicholas_Chang_PCB/Rocket-Flight-Data-Computer/STM32_CubeIDE_Rocket_Project/UserCode" -I"/home/caleb/Documents/Work/Wyzant_Tutoring/Nicholas_Chang_PCB/Rocket-Flight-Data-Computer/STM32_CubeIDE_Rocket_Project/Middlewares/Third_Party/BOSCH_BME" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-UserCode

clean-UserCode:
	-$(RM) ./UserCode/DataLogger.cyclo ./UserCode/DataLogger.d ./UserCode/DataLogger.o ./UserCode/DataLogger.su ./UserCode/SD_Card.cyclo ./UserCode/SD_Card.d ./UserCode/SD_Card.o ./UserCode/SD_Card.su ./UserCode/Scheduler.cyclo ./UserCode/Scheduler.d ./UserCode/Scheduler.o ./UserCode/Scheduler.su ./UserCode/bme680_device.cyclo ./UserCode/bme680_device.d ./UserCode/bme680_device.o ./UserCode/bme680_device.su

.PHONY: clean-UserCode

