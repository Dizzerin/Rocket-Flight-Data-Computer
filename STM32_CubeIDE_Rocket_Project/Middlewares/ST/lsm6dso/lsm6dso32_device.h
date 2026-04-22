/*
 * lsm6dso32_device2.h
 *
 *  Created on: Mar 16, 2026
 *      Author: nicholaschang, Caleb Nelson
 */

#ifndef ST_LSM6DSO_LSM6DSO32_DEVICE_H_
#define ST_LSM6DSO_LSM6DSO32_DEVICE_H_

#include <stdint.h>

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
