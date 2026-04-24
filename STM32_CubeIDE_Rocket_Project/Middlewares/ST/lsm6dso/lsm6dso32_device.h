/*
 * lsm6dso32_device2.h
 *
 *  Created on: Mar 16, 2026
 *      Author: nicholaschang, Caleb Nelson
 */

#ifndef ST_LSM6DSO_LSM6DSO32_DEVICE_H_
#define ST_LSM6DSO_LSM6DSO32_DEVICE_H_

#include <stdint.h>
#include "lsm6dso32_reg.h"

/* =========================================================================
 * Full-scale configuration
 *
 * Change these two #defines to adjust the measurement ranges/scales.
 * The correct conversion function is selected automatically in lsm6_init() —
 * no other changes are needed.
 * ========================================================================= */

/*
 * Accelerometer full-scale range.
 *
 * Valid values: (MUST BE ONE OF THESE EXACT VALUES)
 *   LSM6DSO32_4g    — ±4 g    (0.122 mg/LSB, max ±4000 mg)
 *   LSM6DSO32_8g    — ±8 g    (0.244 mg/LSB, max ±8000 mg)
 *   LSM6DSO32_16g   — ±16 g   (0.488 mg/LSB, max ±16000 mg)
 *   LSM6DSO32_32g   — ±32 g   (0.976 mg/LSB, max ±32000 mg)
 */
#define LSM6DSO_ACCEL_FS    LSM6DSO32_16g

/*
 * Gyroscope full-scale range.
 *
 * Valid values: (MUST BE ONE OF THESE EXACT VALUES)
 *   LSM6DSO32_125dps    — ±125 dps    (4.375 mdps/LSB)
 *   LSM6DSO32_250dps    — ±250 dps    (8.75 mdps/LSB)
 *   LSM6DSO32_500dps    — ±500 dps    (17.5 mdps/LSB)
 *   LSM6DSO32_1000dps   — ±1000 dps   (35.0 mdps/LSB)
 *   LSM6DSO32_2000dps   — ±2000 dps   (70.0 mdps/LSB)
 */
#define LSM6DSO_GYRO_FS     LSM6DSO32_2000dps

/* Compensated measurement result from one polling cycle. */
typedef struct {
    float    accel_mg[3];        /* X, Y, Z linear acceleration in milli-g */
    float    gyro_mdps[3];       /* X, Y, Z angular rate in milli-dps */
    float    temperature_degC;
    uint32_t timestamp_ms;       /* HAL_GetTick() value at the time of this read */
    uint8_t  isAccelDataNew;     /* 1 = accelerometer had fresh data this read */
    uint8_t  isGyroDataNew;      /* 1 = gyroscope had fresh data this read */
    uint8_t  isTempDataNew;      /* 1 = temperature sensor had fresh data this read */
} LSM6DSO_Data_t;

uint8_t lsm6_init(void);                     /* Initializes the sensor and starts continuous measurement mode. Returns 0 on success. */
uint8_t lsm6_readData(LSM6DSO_Data_t *out);  /* 0 = success, does not print */
uint8_t lsm6_getAndPrintData(void);          /* reads + prints via myprintf */

#endif /* ST_LSM6DSO_LSM6DSO32_DEVICE_H_ */
