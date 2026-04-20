################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../UserCode/SD_Card.c 

OBJS += \
./UserCode/SD_Card.o 

C_DEPS += \
./UserCode/SD_Card.d 


# Each subdirectory must supply rules for building sources it contributes
UserCode/%.o UserCode/%.su UserCode/%.cyclo: ../UserCode/%.c UserCode/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"/home/caleb/Documents/Work/Wyzant Tutoring/Nicholas Chang PCB/Rocket-Flight-Data-Computer/STM32_CubeIDE_Rocket_Project/Middlewares/ST/lsm6dso" -I"/home/caleb/Documents/Work/Wyzant Tutoring/Nicholas Chang PCB/Rocket-Flight-Data-Computer/STM32_CubeIDE_Rocket_Project/UserCode" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-UserCode

clean-UserCode:
	-$(RM) ./UserCode/SD_Card.cyclo ./UserCode/SD_Card.d ./UserCode/SD_Card.o ./UserCode/SD_Card.su

.PHONY: clean-UserCode

