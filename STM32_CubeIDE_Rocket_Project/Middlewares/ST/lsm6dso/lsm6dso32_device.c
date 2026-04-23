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
  /* Temperature sensor requires no separate init — it runs automatically
     whenever the accelerometer or gyroscope is active. */

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
      out->isTempDataNew  = 1;
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
 * @brief  Write generic device register (platform dependent).
 *
 * Assembles the register address and data into a single contiguous buffer and
 * issues one HAL_SPI_Transmit call, keeping CS asserted for the entire
 * transaction. This matches the LSM6DSO32 SPI write protocol (datasheet §5.1.2):
 * 8-bit address byte (bit7=0 for write, bits[6:0] = register address) followed
 * by one or more 8-bit data bytes.
 *
 * @param  handle  customizable argument, in this use case it is the SPI bus handle (SPI_HandleTypeDef *)
 * @param  reg     Register address to write (bit7 must be 0; caller's responsibility)
 * @param  bufp    pointer to data bytes to write in register reg
 * @param  len     Number of consecutive registers to write
 * @return 0 on success, non-zero on HAL error
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  if (len + 1U > LSM6DSO_SPI_BUF_SIZE) return 1;  // Buffer overflow

  // // Copy the register address and data into a single SPI transmit buffer so we can send it all in one transaction
  // // This is not required though, we can do it in two seprate ones if we want.
  // uint8_t spiTxBuf[16];
  // spiTxBuf[0] = reg;                       /* Address byte (bit7=0 for write) */
  // memcpy(&spiTxBuf[1], bufp, len);
  // // Send the Data
  // HAL_GPIO_WritePin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin, GPIO_PIN_RESET);
  // HAL_StatusTypeDef halStatus = HAL_SPI_Transmit(handle, spiTxBuf, (uint16_t)(len + 1U), 1000); // +1 for the address byte
  // HAL_GPIO_WritePin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin, GPIO_PIN_SET);

  // Simpler method, saves memcpy overhead and buffer allocation
  HAL_GPIO_WritePin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef halStatus = HAL_SPI_Transmit(handle, &reg, 1, 1000);
  if (halStatus != HAL_OK) {
      HAL_GPIO_WritePin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin, GPIO_PIN_SET);
      return 1;
  }
  halStatus = HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin, GPIO_PIN_SET);

  return (halStatus == HAL_OK) ? 0 : 1;
}

/*
 * @brief  Read generic device register (platform dependent).
 *
 * Uses HAL_SPI_TransmitReceive instead of the separate Transmit+Receive pattern.
 * On STM32H7, the SPI FIFO fills with stale dummy bytes during a Transmit call 
 * (since it is full duplex it is recieving data at the same time);
 * a subsequent Receive call may read from that stale FIFO instead of fresh sensor
 * data. TransmitReceive avoids this by combining both phases into one atomic transfer.
 *
 * TX buffer layout: [reg|0x80, 0xFF, 0xFF, ...] (address + dummy bytes)
 * RX buffer layout: [garbage, data[0], data[1], ...] (garbage while address sent)
 * Actual register data starts at spiRxBuf[1].
 *
 * @param  handle  Customizable argument, in this use case it is the SPI bus handle (SPI_HandleTypeDef *)
 * @param  reg     Register address to read (bit7 will be forced to 1 for read)
 * @param  bufp    Pointer to the buffer to store the read bytes
 * @param  len     Number of consecutive registers to read
 * @return 0 on success, non-zero on HAL error
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  if (len + 1U > LSM6DSO_SPI_BUF_SIZE) return 1;  // Buffer overflow

  // Use internal buffers for the SPI transaction since the caller's buffer may not be suitable for a TransmitReceive operation
  uint8_t spiRxBuf[16];
  spiTxBuf[0] = reg | 0x80;               /* Address byte (bit7=1 for read) */
  memset(&spiTxBuf[1], 0xFF, len);        /* Dummy bytes to clock in the response */

  HAL_GPIO_WritePin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef halStatus = HAL_SPI_TransmitReceive(handle, spiTxBuf, spiRxBuf,
                                                        (uint16_t)(len + 1U), 1000); // +1 for the address byte
  HAL_GPIO_WritePin(LSM6DSO_CS_GPIO_Port, LSM6DSO_CS_Pin, GPIO_PIN_SET);

  // Copy the data back out to the caller's buffer
  /* spiRxBuf[0] is the garbage byte received while the address was being sent;
   * actual register data starts at spiRxBuf[1]. */
  memcpy(bufp, &spiRxBuf[1], len);

  return (halStatus == HAL_OK) ? 0 : 1;
}

