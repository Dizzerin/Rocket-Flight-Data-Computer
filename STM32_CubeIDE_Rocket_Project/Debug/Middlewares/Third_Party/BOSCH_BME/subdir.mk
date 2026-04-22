################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/BOSCH_BME/bme68x.c 

OBJS += \
./Middlewares/Third_Party/BOSCH_BME/bme68x.o 

C_DEPS += \
./Middlewares/Third_Party/BOSCH_BME/bme68x.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/BOSCH_BME/%.o Middlewares/Third_Party/BOSCH_BME/%.su Middlewares/Third_Party/BOSCH_BME/%.cyclo: ../Middlewares/Third_Party/BOSCH_BME/%.c Middlewares/Third_Party/BOSCH_BME/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../FATFS/Target -I../FATFS/App -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"/home/caleb/Documents/Work/Wyzant_Tutoring/Nicholas_Chang_PCB/Rocket-Flight-Data-Computer/STM32_CubeIDE_Rocket_Project/Middlewares/ST/lsm6dso" -I"/home/caleb/Documents/Work/Wyzant_Tutoring/Nicholas_Chang_PCB/Rocket-Flight-Data-Computer/STM32_CubeIDE_Rocket_Project/UserCode" -I"/home/caleb/Documents/Work/Wyzant_Tutoring/Nicholas_Chang_PCB/Rocket-Flight-Data-Computer/STM32_CubeIDE_Rocket_Project/Middlewares/Third_Party/BOSCH_BME" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-BOSCH_BME

clean-Middlewares-2f-Third_Party-2f-BOSCH_BME:
	-$(RM) ./Middlewares/Third_Party/BOSCH_BME/bme68x.cyclo ./Middlewares/Third_Party/BOSCH_BME/bme68x.d ./Middlewares/Third_Party/BOSCH_BME/bme68x.o ./Middlewares/Third_Party/BOSCH_BME/bme68x.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-BOSCH_BME

