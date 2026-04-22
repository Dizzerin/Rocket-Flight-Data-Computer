/*
 * bme680_example.c
 *
 * Example: periodic BME680 sampling and UART3 output.
 *
 * Triggers a forced-mode TPHG measurement every BME680_SAMPLE_INTERVAL_MS ms.
 * When data becomes ready, all four compensated values are printed via myprintf().
 *
 * The state machine in bme680_spi.c is non-blocking: bme680_stateMachine() returns
 * immediately on every call and only reads the sensor when it signals completion.
 * No busy-waiting occurs between trigger and data-ready.
 *
 * Expected output (example):
 *   BME680: T=23.45 C | P=1013.25 hPa | H=48.70 %RH | Gas=valid  85432 Ohm
 */

#include "bme680_example.h"
#include "bme680_spi.h"
#include "main.h"           /* myprintf(), HAL_GetTick() */

/* Interval between measurement triggers (milliseconds) */
#define BME680_SAMPLE_INTERVAL_MS   3000U

static uint32_t lastTriggerTick = 0;
static uint8_t  dataPrinted     = 0;   /* Prevents printing the same result twice */

void bme680_exampleInit(void)
{
    if (bme680_init() != BME680_OK) {
        myprintf("BME680 example: init failed, no further samples will be taken\r\n");
        return;
    }

    /* Fire the first measurement immediately */
    bme680_triggerMeasurement();
    lastTriggerTick = HAL_GetTick();
    dataPrinted = 0;
}

void bme680_exampleUpdate(void)
{
    /* Advance the non-blocking state machine */
    uint8_t result = bme680_stateMachine();

    /* Print data once when it becomes available */
    if (result == BME680_OK && bme680_isDataReady() && !dataPrinted) {
        BME680_Data_t data = bme680_getData();

        /* Gas status string: valid + heater stable is the ideal case */
        const char *gas_status;
        if (!data.gas_valid) {
            gas_status = "invalid/disabled";
        } else if (!data.heat_stab) {
            gas_status = "valid (heater unstable)";
        } else {
            gas_status = "valid";
        }

        myprintf("BME680: T=%.2f C | P=%.2f hPa | H=%.2f %%RH | Gas=%s  %.0f Ohm\r\n",
                 data.temperature_degC,
                 data.pressure_hPa,
                 data.humidity_pctRH,
                 gas_status,
                 data.gas_resistance_ohms);

        dataPrinted = 1;
    }

    /* Trigger next measurement after the interval expires */
    if ((HAL_GetTick() - lastTriggerTick) >= BME680_SAMPLE_INTERVAL_MS) {
        bme680_triggerMeasurement();    /* No-op if a measurement is already running */
        lastTriggerTick = HAL_GetTick();
        dataPrinted = 0;
    }
}
