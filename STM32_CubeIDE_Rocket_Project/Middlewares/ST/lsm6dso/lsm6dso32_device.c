/*
 ******************************************************************************
 * @file    read_data_polling.c
 * @author  Sensors Software Solution Team
 * @brief   This file shows how to get data from the LSM6DSO32 sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * Used interfaces:
 * - Host side:   UART3
 * - Sensor side: SPI1
 */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lsm6dso32_reg.h"
#include "main.h"	// For LSM6DSO_CS pin defines and myprintf()
#include "stm32h7xx_hal.h"	// For HAL_Delay() and HAL_GetTick()
#include "stm32h7xx_hal_gpio.h"	// To control LSM6DSO_CS pin
#include "stm32h7xx_hal_spi.h"	// To communicate with the LSM6DSO Chip vs SPI1
#include "lsm6dso32_device.h"

// Specify which SPI bus to use to communicate with the LSM6DSO (SPI1)
extern SPI_HandleTypeDef hspi1;
#define SENSOR_BUS hspi1

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME              10

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static float_t temperature_degC;
static uint8_t whoamI, rst;
stmdev_ctx_t dev_ctx;
uint8_t isInitialized = 0;

/* Private functions ---------------------------------------------------------*/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);


/* Main Example --------------------------------------------------------------*/
uint8_t lsm6_init(void)
{
  // ---------- Device INITIALIZATION code -----------
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = HAL_Delay;
  dev_ctx.handle = &SENSOR_BUS;

  /* Wait sensor boot time */
  HAL_Delay(BOOT_TIME);

  /* Check device ID */
  lsm6dso32_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSO32_ID) {
	  myprintf("LSM6DSO32 Device ID Mismatch!\r\n");
	  return 1;
  }

  /* Restore default configuration */
  lsm6dso32_reset_set(&dev_ctx, PROPERTY_ENABLE);

  uint32_t timeout = HAL_GetTick() + 1000; // 1 second timeout
  do {
    lsm6dso32_reset_get(&dev_ctx, &rst);
    
    // Break out of loop if timeout exceeded
    if (HAL_GetTick() >= timeout) {
        myprintf("LSM6DSO32 Reset Timeout!\r\n");
        return 1;
    }
  } while (rst);

  /* Disable I3C interface */
  lsm6dso32_i3c_disable_set(&dev_ctx, LSM6DSO32_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dso32_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set full scale */
  lsm6dso32_xl_full_scale_set(&dev_ctx, LSM6DSO32_16g);
  lsm6dso32_gy_full_scale_set(&dev_ctx, LSM6DSO32_2000dps);
  /* Set ODR (Output Data Rate) and power mode*/
  lsm6dso32_xl_data_rate_set(&dev_ctx, LSM6DSO32_XL_ODR_104Hz_HIGH_PERF);
  lsm6dso32_gy_data_rate_set(&dev_ctx, LSM6DSO32_GY_ODR_104Hz_HIGH_PERF);
  // TODO Do we need to init temp data as well?

  isInitialized = 1;
  return 0; // To indicate success
}

/*
 * @brief  Read the latest sensor data into the provided struct.
 *
 * Checks the status register for each axis. Only reads axes that have fresh
 * data available; stale axes return the last successfully read value.
 * isAccelDataNew and isGyroDataNew indicate whether each axis had new data
 * this call, which is useful for detecting stale readings in the log file.
 *
 * @param  out  Pointer to LSM6DSO_Data_t struct to populate.
 * @return 0 on success, 1 if not initialized or out is NULL.
 */
uint8_t lsm6_readData(LSM6DSO_Data_t *out)
{
  if (!isInitialized || out == NULL) return 1;

  out->isAccelDataNew = 0;
  out->isGyroDataNew  = 0;
  out->isTempDataNew  = 0;
  out->timestamp_ms   = HAL_GetTick();

  // Get latest status register to check which axes have fresh data available
  lsm6dso32_reg_t reg;
  lsm6dso32_status_reg_get(&dev_ctx, &reg.status_reg);

  // If accelerometer data available, read fresh values
  if (reg.status_reg.xlda) {
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm6dso32_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] = lsm6dso32_from_fs16_to_mg(data_raw_acceleration[2]);
      out->isAccelDataNew = 1;
  }
  out->accel_mg[0] = acceleration_mg[0];
  out->accel_mg[1] = acceleration_mg[1];
  out->accel_mg[2] = acceleration_mg[2];

  // If gyro data available, read fresh values
  if (reg.status_reg.gda) {
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      lsm6dso32_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
      angular_rate_mdps[0] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[0]);
      angular_rate_mdps[1] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[1]);
      angular_rate_mdps[2] = lsm6dso32_from_fs2000_to_mdps(data_raw_angular_rate[2]);
      out->isGyroDataNew = 1;
  }
  out->gyro_mdps[0] = angular_rate_mdps[0];
  out->gyro_mdps[1] = angular_rate_mdps[1];
  out->gyro_mdps[2] = angular_rate_mdps[2];

  // If temperature data available, read fresh value
  if (reg.status_reg.tda) {
      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
      lsm6dso32_temperature_raw_get(&dev_ctx, &data_raw_temperature);
      temperature_degC = lsm6dso32_from_lsb_to_celsius(data_raw_temperature);
      out->isTempDataNew  = 1;  // TODO use this
  }
  out->temperature_degC = temperature_degC;

  return 0;
}

/* Read samples and print to UART via myprintf (debug/diagnostic use). */
uint8_t lsm6_getAndPrintData(void)
{
    if (!isInitialized) {
        myprintf("Cannot read data because device is not initialized.\r\n");
        return 1;
    }

    LSM6DSO_Data_t data;
    lsm6_readData(&data);

    myprintf("Accel [mg]: %4.2f\t%4.2f\t%4.2f\r\n",
             data.accel_mg[0], data.accel_mg[1], data.accel_mg[2]);
    myprintf("Gyro [mdps]: %4.2f\t%4.2f\t%4.2f\r\n",
             data.gyro_mdps[0], data.gyro_mdps[1], data.gyro_mdps[2]);
    myprintf("Temp [degC]: %6.2f\r\n", data.temperature_degC);

    return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_GPIO_WritePin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin, GPIO_PIN_SET);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  reg |= 0x80; // Mask off highest bit, ensure it is a 1
  HAL_GPIO_WritePin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin, GPIO_PIN_SET);
  return 0;
}

