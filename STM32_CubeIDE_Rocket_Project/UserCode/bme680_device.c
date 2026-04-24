/*
 * bme680_spi.c
 *
 * BME680 barometer driver — wrapper around the Bosch BME68x_SensorAPI v4.4.8.
 * Interface: SPI3 (hspi3), CS = BARO2_CS (PC9).
 *
 * This file owns:
 *   - Three platform callbacks required by the Bosch API (SPI read, SPI write,
 *     and a microsecond delay), which call internal STM32 HAL functions.
 *   - A non-blocking forced-mode state machine that drives the sensor
 *
 * Forced mode overview:
 *   The BME680 performs one measurement per trigger and returns to sleep.
 *   bme680_stateMachine() handles the trigger -> wait -> read sequence non-blockingly, with states:
 *     TRIGGER_PENDING -> configures/triggers forced mode -> WAIT_MEAS
 *     WAIT_MEAS       -> waits for expected duration, then polls new_data bit
 *                        -> DATA_READY once the sensor signals completion
 *     DATA_READY      -> holds results until the next trigger is requested
 *     ERROR           -> entered on communication failure or measurement timeout
 *
 * STM32H7 SPI note:
 *   The H7 SPI peripheral has a hardware FIFO. Calling HAL_SPI_Transmit then
 *   HAL_SPI_Receive in sequence (while holding CS low) can leave stale bytes in
 *   the RX FIFO and corrupt the received data. The read callback here uses
 *   HAL_SPI_TransmitReceive with a combined tx/rx buffer to avoid this entirely.
 *
 * Measurement config:
 *   Temperature oversampling: x2
 *   Pressure oversampling:    x16  (best altitude resolution, ~1.7 cm RMS noise)
 *   Humidity oversampling:    x1
 *   IIR filter:               off  (real-time data, no lag)
 *   Gas sensor:               off by default; enable with bme680_setGasEnabled(1)
 *                             Gas heater: 300 C target, 100 ms duration
 */

#include <bme680_device.h>
#include "bme68x.h"              /* Bosch BME68x SensorAPI */
#include "main.h"                /* BARO2_CS pin defines, myprintf() */
#include "stm32h7xx_hal.h"
#include <string.h>

/* =========================================================================
 * SPI bus handle
 * ========================================================================= */
extern SPI_HandleTypeDef hspi3;

/* =========================================================================
 * SPI buffer
 * The Bosch API's largest single transfer is calibration data (~25 bytes).
 * Write interleave buffer is at most (2*10)-1 = 19 data bytes + 1 address byte.
 * 64 bytes gives comfortable headroom.
 * ========================================================================= */
#define BME680_SPI_BUF_SIZE     64U

static uint8_t spiTxBuf[BME680_SPI_BUF_SIZE];
static uint8_t spiRxBuf[BME680_SPI_BUF_SIZE];

/* =========================================================================
 * Driver configuration
 * ========================================================================= */
#define HEATER_TARGET_DEGC      300U    /* Gas heater target temperature */
#define HEATER_DURATION_MS      100U    /* Gas heater duration in ms */
#define MEAS_TIMEOUT_MS         3000U   /* Measurement timeout (generous safety margin) */

/* =========================================================================
 * Module state
 * ========================================================================= */
static struct bme68x_dev        boschDev;
static struct bme68x_conf       boschConf;
static struct bme68x_heatr_conf boschHeatrConf;

static BME680_State_t  state             = BME680_STATE_IDLE;
static BME680_Data_t   latestData;
static uint8_t         isInitialized       = 0;
static uint8_t         isGasEnabled          = 0;     /* 0 = off (default), 1 = on */
static uint8_t         lastIsGasEnabledState = 0xFF;  /* 0xFF sentinel forces first gas enable/heater write */
static uint32_t        measStartTick       = 0;
static uint32_t        measDurationMs      = 50U;   /* Updated at init and when gas mode changes */

/* =========================================================================
 * Platform callbacks for the Bosch BME68x SensorAPI
 *
 * The Bosch API manages SPI memory page switching internally via set_mem_page()
 * before calling these callbacks. These functions only handle raw SPI transfer.
 * ========================================================================= */

/*
 * @brief  SPI read callback required by the Bosch API.
 *
 * Uses HAL_SPI_TransmitReceive instead of the separate Transmit+Receive pattern.
 * On STM32H7, the SPI FIFO fills with dummy bytes during a Transmit call; a
 * subsequent Receive call may read from that stale FIFO instead of fresh data.
 * TransmitReceive avoids this by combining both phases into one atomic transfer.
 *
 * The Bosch API sets bit7 of reg_addr before calling this function (bit7=1
 * signals a read on the BME680's SPI protocol), so we pass it through unchanged.
 *
 * @param reg_addr   Register address (bit7 already set to 1 by the Bosch API)
 * @param reg_data   Buffer to store the read bytes
 * @param length     Number of bytes to read
 * @param intf_ptr   Unused interface pointer
 * @return           0 on success, non-zero on failure
 */
static BME68X_INTF_RET_TYPE bme68x_spi_read(uint8_t reg_addr, uint8_t *reg_data,
                                              uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    if (length + 1U > BME680_SPI_BUF_SIZE) {
        return 1;   /* Requested length exceeds buffer -- should never happen */
    }

    spiTxBuf[0] = reg_addr;                  /* Address byte (bit7=1 for read, set by Bosch API) */
    memset(&spiTxBuf[1], 0xFF, length);      /* Dummy bytes to clock in the response */

    HAL_GPIO_WritePin(BARO2_CS_GPIO_Port, BARO2_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef halStatus = HAL_SPI_TransmitReceive(&hspi3, spiTxBuf, spiRxBuf,
                                                           (uint16_t)(length + 1U), 100);
    HAL_GPIO_WritePin(BARO2_CS_GPIO_Port, BARO2_CS_Pin, GPIO_PIN_SET);

    /* spiRxBuf[0] is the garbage byte received while the address was being sent;
     * actual register data starts at spiRxBuf[1]. */
    memcpy(reg_data, &spiRxBuf[1], length);

    return (halStatus == HAL_OK) ? BME68X_INTF_RET_SUCCESS : 1;
}

/*
 * @brief  SPI write callback required by the Bosch API.
 *
 * For multi-register writes the Bosch API pre-interleaves address/data pairs
 * into its internal buffer and passes:
 *   reg_addr  = first address byte (bit7 already cleared to 0 for SPI write)
 *   reg_data  = remaining bytes (may contain further addr/data pairs)
 *   length    = number of bytes in reg_data
 *
 * This function assembles them into one combined buffer and issues a single
 * HAL_SPI_Transmit call, keeping CS asserted for the entire transaction.
 *
 * @param reg_addr   First register address (bit7 cleared to 0 by the Bosch API)
 * @param reg_data   Data bytes to write (may include interleaved addresses)
 * @param length     Number of bytes in reg_data
 * @param intf_ptr   Unused interface pointer
 * @return           0 on success, non-zero on failure
 */
static BME68X_INTF_RET_TYPE bme68x_spi_write(uint8_t reg_addr, const uint8_t *reg_data,
                                               uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;

    if (length + 1U > BME680_SPI_BUF_SIZE) {
        return 1;   /* Requested length exceeds buffer -- should never happen */
    }

    spiTxBuf[0] = reg_addr;              /* First register address (bit7=0 for write) */
    memcpy(&spiTxBuf[1], reg_data, length);

    HAL_GPIO_WritePin(BARO2_CS_GPIO_Port, BARO2_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef halStatus = HAL_SPI_Transmit(&hspi3, spiTxBuf,
                                                    (uint16_t)(length + 1U), 100);
    HAL_GPIO_WritePin(BARO2_CS_GPIO_Port, BARO2_CS_Pin, GPIO_PIN_SET);

    return (halStatus == HAL_OK) ? BME68X_INTF_RET_SUCCESS : 1;
}

/*
 * @brief  Microsecond delay callback required by the Bosch API.
 *
 * HAL_Delay() has 1 ms resolution, so we round up. The delays requested by the
 * Bosch API (mainly the 10 ms PERIOD_POLL and 10 ms PERIOD_RESET) are not
 * timing-critical at sub-millisecond granularity for this application.
 *
 * @param period_us  Delay duration in microseconds
 * @param intf_ptr   Unused interface pointer
 */
static void bme68x_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    HAL_Delay((period_us + 999U) / 1000U);  /* Round up to nearest ms */
}

/* =========================================================================
 * Internal helper: recalculate expected measurement duration.
 * bme68x_get_meas_dur() returns microseconds for the T/P/H conversion time.
 * If gas is enabled, we add the heater duration on top of that.
 * ========================================================================= */
static void updateMeasDuration(void)
{
    uint32_t durUs = bme68x_get_meas_dur(BME68X_FORCED_MODE, &boschConf, &boschDev);
    measDurationMs = (durUs + 999U) / 1000U;    /* T/P/H conversion time, rounded up */
    if (isGasEnabled) {
        measDurationMs += HEATER_DURATION_MS;   /* Add gas heater wait time */
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

/*
 * @brief  Initialize the BME680 sensor using the Bosch BME68x SensorAPI.
 *
 * Populates the bme68x_dev struct with platform callbacks, then calls
 * bme68x_init() which performs a soft reset, verifies the chip ID (0x61),
 * and reads all factory calibration coefficients. Then configures oversampling,
 * filter, and heater settings.
 *
 * Must be called after MX_SPI3_Init() and MX_GPIO_Init().
 *
 * @return BME680_OK on success, BME680_ERROR on failure.
 */
uint8_t bme680_init(void)
{
    HAL_Delay(10);   /* Allow sensor startup after power-on */

    /* Populate the Bosch device struct with platform-specific callbacks */
    boschDev.intf     = BME68X_SPI_INTF;
    boschDev.intf_ptr = NULL;            /* Not needed; callbacks reference hspi3 directly */
    boschDev.read     = bme68x_spi_read;
    boschDev.write    = bme68x_spi_write;
    boschDev.delay_us = bme68x_delay_us;
    boschDev.amb_temp = 25;              /* Initial ambient temp estimate for heater calc */

    /* bme68x_init() performs: soft reset -> verify chip ID (0x61) -> load calibration */
    int8_t result = bme68x_init(&boschDev);
    if (result != BME68X_OK) {
        myprintf("BME680 init FAIL: Bosch API error %d (chip ID read: 0x%02X)\r\n",
                 result, boschDev.chip_id);
        state = BME680_STATE_ERROR;
        return BME680_ERROR;
    }

    /* Sensor measurement configuration */
    boschConf.os_hum  = BME68X_OS_1X;      /* Humidity oversampling x1 */
    boschConf.os_temp = BME68X_OS_2X;      /* Temperature oversampling x2 */
    boschConf.os_pres = BME68X_OS_16X;     /* Pressure oversampling x16 (best altitude res) */
    boschConf.filter  = BME68X_FILTER_OFF; /* IIR filter off -- real-time data, no lag */
    boschConf.odr     = BME68X_ODR_NONE;   /* Standby time/Output Data Rate ODR N/A in forced mode */

    result = bme68x_set_conf(&boschConf, &boschDev);
    if (result != BME68X_OK) {
        myprintf("BME680 init FAIL: bme68x_set_conf error %d\r\n", result);
        state = BME680_STATE_ERROR;
        return BME680_ERROR;
    }

    /* Heater configuration (gas off by default; reconfigured on first trigger if changed) */
    boschHeatrConf.enable     = BME68X_DISABLE;
    boschHeatrConf.heatr_temp = HEATER_TARGET_DEGC;
    boschHeatrConf.heatr_dur  = HEATER_DURATION_MS;

    result = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &boschHeatrConf, &boschDev);
    if (result != BME68X_OK) {
        myprintf("BME680 init FAIL: bme68x_set_heatr_conf error %d\r\n", result);
        state = BME680_STATE_ERROR;
        return BME680_ERROR;
    }

    /* Pre-calculate how long a forced-mode measurement will take */
    updateMeasDuration();
    lastIsGasEnabledState = isGasEnabled;   /* Sync state so first trigger skips unnecessary reconfig */

    isInitialized = 1;
    state         = BME680_STATE_IDLE;
    myprintf("BME680 init OK (chip ID 0x%02X, meas duration ~%lu ms)\r\n",
             boschDev.chip_id, (unsigned long)measDurationMs);
    return BME680_OK;
}

/*
 * Enable or disable the gas sensor. Takes effect on the next triggered measurement.
 * Disabling saves ~100 ms per measurement and ~12 mA of heater current.
 * When disabled, gas_resistance_ohms will be 0 and gas_valid will be 0.
 *
 * @param isEnabled  1 to enable gas measurements, 0 to disable (default)
 */
void bme680_setGasEnabled(uint8_t isEnabled)
{
    isGasEnabled = isEnabled ? 1U : 0U;
}

/*
 * Request a new forced-mode measurement. Safe to call from IDLE or DATA_READY.
 * Has no effect if a measurement is already in progress.
 */
void bme680_triggerMeasurement(void)
{
    if (!isInitialized) return;
    if (state == BME680_STATE_WAIT_MEAS || state == BME680_STATE_TRIGGER_PENDING)
    {
        // Note: This may be triggered from time to time during normal operation if:
        //  - A BME680 measurement takes longer than expected
        //  - We wait too long in the BME680 state machine (since we round up 
        //   the measurement duration) - we do this because if we don't wait 
        //   long enough and poll the sensor too early, we can trigger the 
        //   Bosch API's internal blocking retry loop of 10ms (up to 5 times),
        //   so triggering this on occasion is still better/less total delay. 
        myprintf("BME680: Note, trigger ignored, measurement already in progress\r\n");
        return;
    }
    else if (state == BME680_STATE_ERROR)
    {
        myprintf("BME680: Warning! Trigger ignored, sensor in error state\r\n");
        return;
    }
    
    // Set state machine state to trigger a new measurement
    state = BME680_STATE_TRIGGER_PENDING;
}

/*
 * @brief  Non-blocking BME680 measurement state machine. Call repeatedly.
 *
 * Drives the forced-mode pipeline without blocking:
 *   - TRIGGER_PENDING: reconfigures heater if isGasEnabled changed, issues forced
 *     mode trigger, then transitions to WAIT_MEAS.
 *   - WAIT_MEAS: returns BUSY immediately until the expected measurement duration
 *     has elapsed. After that, polls the new_data status bit directly with
 *     bme68x_get_regs() before calling bme68x_get_data(). This avoids the
 *     blocking 10 ms retry loop inside bme68x's read_field_data() that would
 *     fire if we called bme68x_get_data() before data was ready.
 *   - DATA_READY: returns BME680_OK immediately until next trigger.
 *
 * @return BME680_OK    -- idle or data ready (check bme680_isDataReady())
 *         BME680_BUSY  -- measurement in progress
 *         BME680_ERROR -- communication failure or measurement timeout
 */
uint8_t bme680_stateMachine(void)
{
    if (!isInitialized) return BME680_ERROR;

    switch (state)
    {
        case BME680_STATE_IDLE:
        case BME680_STATE_DATA_READY:
            return BME680_OK;

        case BME680_STATE_TRIGGER_PENDING:
        {
            /* Reconfigure heater only when isGasEnabled has changed since last trigger */
            if (isGasEnabled != lastIsGasEnabledState) {
                boschHeatrConf.enable = isGasEnabled ? BME68X_ENABLE : BME68X_DISABLE;
                int8_t result = bme68x_set_heatr_conf(BME68X_FORCED_MODE,
                                                       &boschHeatrConf, &boschDev);
                if (result != BME68X_OK) {
                    myprintf("BME680: set_heatr_conf failed (%d)\r\n", result);
                    state = BME680_STATE_ERROR;
                    return BME680_ERROR;
                }
                updateMeasDuration();
                lastIsGasEnabledState = isGasEnabled;
            }

            /* Issue the forced-mode trigger.
             * bme68x_set_op_mode() confirms the sensor is in sleep mode first
             * (it should be, having returned to sleep after the previous forced
             * measurement), then sets the forced-mode bits. */
            int8_t result = bme68x_set_op_mode(BME68X_FORCED_MODE, &boschDev);
            if (result != BME68X_OK) {
                myprintf("BME680: set_op_mode failed (%d)\r\n", result);
                state = BME680_STATE_ERROR;
                return BME680_ERROR;
            }

            measStartTick = HAL_GetTick();
            state = BME680_STATE_WAIT_MEAS;
            return BME680_BUSY;
        }

        case BME680_STATE_WAIT_MEAS:
        {
            /* Snapshot the tick counter once for all comparisons in this case.
             * Unsigned subtraction (now - measStartTick) is inherently wraparound-safe:
             * modular arithmetic produces the correct elapsed time even when the
             * 32-bit counter rolls over from 0xFFFFFFFF back to 0. The only
             * constraint is that the two timestamps are within ~24.8 days of each
             * other, which is trivially true for a measurement timeout of 3 seconds.
             */
            uint32_t now = HAL_GetTick();

            /* Guard against a stuck measurement */
            if ((now - measStartTick) > MEAS_TIMEOUT_MS) {
                myprintf("BME680: measurement timeout after %lu ms\r\n",
                         (unsigned long)MEAS_TIMEOUT_MS);
                state = BME680_STATE_ERROR;
                return BME680_ERROR;
            }

            /* Don't poll the sensor until the expected measurement duration has
             * elapsed. Avoids redundant SPI traffic and nearly guarantees data
             * will be ready on the first status check. */
            if ((now - measStartTick) < measDurationMs) {
                return BME680_BUSY;
            }

            /* Call bme68x_get_data() directly now that measDurationMs has elapsed.
             * read_field_data() reads the 17-byte field block (starting at
             * BME68X_REG_FIELD0 = 0x1D) and checks the new_data bit. Since we
             * waited measDurationMs, the bit should be set on the first read and
             * read_field_data() returns immediately.
             *
             * NOTE: We deliberately do NOT pre-read the status register to check
             * new_data before calling bme68x_get_data(). Reading register 0x1D
             * clears the new_data flag in hardware. If we consumed it in a
             * pre-read, bme68x_get_data()'s internal read_field_data() would see
             * new_data = 0, trigger its 5-retry × 10 ms blocking loop (50 ms
             * total), and return BME68X_W_NO_NEW_DATA — the opposite of the
             * intended non-blocking behavior. */
            struct bme68x_data measData;
            uint8_t numFields = 0;
            int8_t readResult = bme68x_get_data(BME68X_FORCED_MODE, &measData, &numFields, &boschDev);

            if (readResult == BME68X_W_NO_NEW_DATA || numFields == 0) {
                /* Data not ready yet despite elapsed measDurationMs — timing jitter.
                 * Stay in WAIT_MEAS and retry on the next tick rather than erroring. */
                return BME680_BUSY;
                // TODO implement a max number of retries here and eventually fall out to the error state
            }

            if (readResult != BME68X_OK) {
                myprintf("BME680: get_data failed (%d)\r\n", readResult);
                state = BME680_STATE_ERROR;
                return BME680_ERROR;
            }

            /* Map Bosch API output to our BME680_Data_t.
             * Bosch API returns pressure in Pascals; we store hPa (divide by 100). */
            latestData.temperature_degC    = measData.temperature;
            latestData.pressure_hPa        = measData.pressure / 100.0f;
            latestData.humidity_pctRH      = measData.humidity;
            latestData.gas_valid           = (measData.status & BME68X_GASM_VALID_MSK) ? 1U : 0U;
            latestData.heat_stab           = (measData.status & BME68X_HEAT_STAB_MSK)  ? 1U : 0U;
            latestData.gas_resistance_ohms = latestData.gas_valid ? measData.gas_resistance : 0.0f;
            latestData.timestamp_ms        = HAL_GetTick();

            /* Update the Bosch driver's ambient temperature estimate for
             * more accurate heater resistance calculations on the next trigger */
            boschDev.amb_temp = (int8_t)latestData.temperature_degC;

            state = BME680_STATE_DATA_READY;
            return BME680_OK;
        }

        case BME680_STATE_ERROR:
            return BME680_ERROR;

        default:
            return BME680_ERROR;
    }
}

/* Returns 1 if compensated data is available, 0 otherwise. */
uint8_t bme680_isDataReady(void)
{
    return (state == BME680_STATE_DATA_READY) ? 1U : 0U;
}

/* Returns the most recently compensated measurement data. */
BME680_Data_t bme680_getData(void)
{
    return latestData;
}
