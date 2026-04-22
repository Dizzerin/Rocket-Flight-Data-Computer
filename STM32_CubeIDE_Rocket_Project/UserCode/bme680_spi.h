/*
 * bme680_spi.h
 *
 * BME680 barometer driver wrapper header (SPI3, BARO2_CS on PC9).
 * Wraps the official Bosch BME68x_SensorAPI (Middlewares/Third_Party/BOSCH_BME/)
 * with a non-blocking state machine.
 *
 * Provides temperature (°C), pressure (hPa), humidity (%RH), and optionally
 * gas resistance (Ohms) from the BME680 sensor.
 *
 * Usage:
 *   1. Call bme680_init() once after MX_SPI3_Init() and MX_GPIO_Init().
 *   2. Register bme680_stateMachine() with the scheduler (or call from DataLogger).
 *   3. Call bme680_triggerMeasurement() to start a forced-mode TPHG cycle.
 *   4. When bme680_isDataReady() returns 1, call bme680_getData() for results.
 *
 * Forced mode operation:
 *   Each measurement is triggered on demand. The sensor performs one TPHG
 *   conversion and returns to sleep automatically. bme680_stateMachine() polls
 *   for completion non-blockingly and populates the internal data cache when done.
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
    float    gas_resistance_ohms;   /* Gas sensor resistance in Ohms (0 if gas disabled) */
    uint32_t timestamp_ms;          /* HAL_GetTick() value when this measurement completed */
    uint8_t  gas_valid;             /* 1 = gas reading is enabled and valid */
    uint8_t  heat_stab;             /* 1 = heater reached target temperature */
} BME680_Data_t;

/* Public API */
uint8_t       bme680_init(void);
void          bme680_setGasEnabled(uint8_t isEnabled);  /* 1 = on, 0 = off (default) */
void          bme680_triggerMeasurement(void);
uint8_t       bme680_stateMachine(void);                /* Call repeatedly to drive the measurement pipeline */
uint8_t       bme680_isDataReady(void);
BME680_Data_t bme680_getData(void);

#endif /* BME680_SPI_H_ */
