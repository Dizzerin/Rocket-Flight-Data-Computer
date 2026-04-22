/*
 * bme680_spi.h
 *
 * BME680 barometer driver header (SPI3, BARO2_CS on PC9).
 * Provides temperature (C), pressure (hPa), humidity (%RH),
 * and gas resistance (Ohms) via a non-blocking state machine.
 *
 * Usage:
 *   1. Call bme680_init() once after MX_SPI3_Init() and MX_GPIO_Init().
 *   2. Call bme680_triggerMeasurement() to start a single-shot TPHG cycle.
 *   3. Call bme680_stateMachine() repeatedly from the main loop.
 *   4. When bme680_isDataReady() returns 1, call bme680_getData() for results.
 */

#ifndef BME680_SPI_H_
#define BME680_SPI_H_

#include <stdint.h>

/* Return codes */
#define BME680_OK    0
#define BME680_BUSY  1
#define BME680_ERROR 2

/* Driver state machine states */
typedef enum {
    BME680_STATE_IDLE,
    BME680_STATE_TRIGGER_PENDING,
    BME680_STATE_WAIT_MEAS,
    BME680_STATE_DATA_READY,
    BME680_STATE_ERROR
} BME680_State_t;

/* Compensated measurement results */
typedef struct {
    float    temperature_degC;      /* Temperature in degrees Celsius */
    float    pressure_hPa;          /* Pressure in hectopascals (millibars) */
    float    humidity_pctRH;        /* Relative humidity in percent (0-100) */
    float    gas_resistance_ohms;   /* Gas sensor resistance in Ohms */
    uint32_t timestamp_ms;          /* HAL_GetTick() value when this measurement completed */
    uint8_t  gas_valid;             /* 1 = gas reading is enabled and valid (real conversion) */
    uint8_t  heat_stab;             /* 1 = heater reached target temperature */
} BME680_Data_t;

/* Public API */
uint8_t       bme680_init(void);
void          bme680_setGasEnabled(uint8_t enabled); /* 1=on, 0=off (default) */
void          bme680_triggerMeasurement(void);
uint8_t       bme680_stateMachine(void);             /* Call repeatedly; drives the measurement state machine */
uint8_t       bme680_isDataReady(void);
BME680_Data_t bme680_getData(void);

#endif /* BME680_SPI_H_ */
