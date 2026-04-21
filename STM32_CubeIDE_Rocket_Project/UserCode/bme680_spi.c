/*
 * bme680_spi.c
 *
 * BME680 barometer driver implementation.
 * Interface: SPI3 (hspi3), CS = BARO2_CS (PC9).
 *
 * SPI memory page system:
 *   The BME680 uses 7-bit SPI addressing (bit 7 = R/W). The 256-byte register
 *   space is split into two 128-byte pages selected by spi_mem_page (bit 4 of
 *   the status register at SPI address 0x73, accessible from both pages):
 *     Page 0 (spi_mem_page=0, default after power-on): I2C 0x80-0xFF
 *       SPI address sent = I2C_address & 0x7F
 *     Page 1 (spi_mem_page=1): I2C 0x00-0x7F
 *       SPI address sent = I2C_address directly
 *
 * Measurement config:
 *   Temperature oversampling: x2
 *   Pressure oversampling:    x16 (best altitude resolution ~1.7 cm RMS)
 *   Humidity oversampling:    x1
 *   IIR filter:               off (real-time data, no lag)
 *   Gas sensor:               enabled, 300 C target, 100 ms heater wait
 */

#include "bme680_spi.h"
#include "main.h"           /* BARO2_CS pin defines, myprintf() */
#include "stm32h7xx_hal.h"

/* =========================================================================
 * SPI bus handle
 * ========================================================================= */
extern SPI_HandleTypeDef hspi3;

/* =========================================================================
 * Register definitions
 * All addresses listed are the SPI addresses actually transmitted on the bus.
 * ========================================================================= */

/* Page 0 registers (I2C 0x80-0xFF, SPI addr = I2C & 0x7F) */
#define REG_CHIP_ID         0x50    /* I2C 0xD0, expected value 0x61 */
#define REG_RESET           0x60    /* I2C 0xE0, write 0xB6 for soft reset */
#define BME680_CHIP_ID_VAL  0x61
#define SOFT_RESET_CMD      0xB6

/* Calibration group 1: I2C 0x89-0xA1, SPI Page 0, addr 0x09, 25 bytes */
#define CALIB1_SPI_ADDR     0x09
#define CALIB1_LEN          25

/* Calibration group 2: I2C 0xE1-0xEE, SPI Page 0, addr 0x61, 14 bytes */
#define CALIB2_SPI_ADDR     0x61
#define CALIB2_LEN          14

/* Page 1 registers (I2C 0x00-0x7F, SPI addr = I2C addr directly) */
#define REG_RES_HEAT_VAL    0x00    /* Heater resistance calibration (int8_t) */
#define REG_RES_HEAT_RNG    0x02    /* bits [5:4]: res_heat_range */
#define REG_RANGE_SW_ERR    0x04    /* bits [7:4]: range switching error (int4) */

#define REG_MEAS_STATUS     0x1D    /* bit7=new_data, bit6=gas_meas, bit5=measuring */
#define REG_PRESS_MSB       0x1F    /* Start of 15-byte burst: 0x1D-0x2B */

#define REG_RES_HEAT0       0x5A    /* Heater resistance target for profile 0 */
#define REG_GAS_WAIT0       0x64    /* Heater wait time for profile 0 */

#define REG_CTRL_GAS0       0x70    /* bit3: heat_off */
#define REG_CTRL_GAS1       0x71    /* bit4: run_gas, bits[3:0]: nb_conv */
#define REG_CTRL_HUM        0x72    /* bits[2:0]: osrs_h */
#define REG_STATUS          0x73    /* bit4: spi_mem_page (accessible both pages) */
#define REG_CTRL_MEAS       0x74    /* bits[7:5]: osrs_t, bits[4:2]: osrs_p, bits[1:0]: mode */
#define REG_CONFIG          0x75    /* bits[4:2]: filter coefficient */

/* =========================================================================
 * Measurement configuration
 * ========================================================================= */
#define OSRS_T              0x02    /* Temperature oversampling x2  (0b010) */
#define OSRS_P              0x05    /* Pressure oversampling x16    (0b101) */
#define OSRS_H              0x01    /* Humidity oversampling x1     (0b001) */
#define FILTER_OFF          0x00    /* IIR filter off               (0b000) */
#define HEATER_TARGET_DEGC  300     /* Gas heater target temperature */
/* Gas wait: 25 timer steps x4 multiplier = 100 ms
 * gas_wait register: bits[7:6]=0b01 (x4), bits[5:0]=0b011001 (25) => 0x59 */
#define GAS_WAIT_VAL        0x59
#define MEAS_TIMEOUT_MS     3000    /* Measurement timeout (must exceed main loop period) */

/* =========================================================================
 * Calibration data structure
 * ========================================================================= */
typedef struct {
    /* Temperature */
    uint16_t par_t1;
    int16_t  par_t2;
    int8_t   par_t3;
    /* Pressure */
    uint16_t par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int16_t  par_p4;
    int16_t  par_p5;
    int8_t   par_p6;
    int8_t   par_p7;
    int16_t  par_p8;
    int16_t  par_p9;
    uint8_t  par_p10;
    /* Humidity */
    uint16_t par_h1;
    uint16_t par_h2;
    int8_t   par_h3;
    int8_t   par_h4;
    int8_t   par_h5;
    uint8_t  par_h6;
    int8_t   par_h7;
    /* Gas */
    int8_t   par_g1;
    int16_t  par_g2;
    int8_t   par_g3;
    uint8_t  res_heat_range;
    int8_t   res_heat_val;
    int8_t   range_sw_err;
    /* Intermediate (shared between compensation functions) */
    float    t_fine;
} BME680_CalibData_t;

/* =========================================================================
 * Module state
 * ========================================================================= */
static BME680_State_t    state        = BME680_STATE_IDLE;
static BME680_CalibData_t calib;
static BME680_Data_t     latestData;
static uint8_t           isInitialized = 0;
static uint8_t           gasEnabled    = 0;   /* 0 = gas sensor off (default), 1 = on */
static uint8_t           currentPage   = 0;   /* Tracks active SPI page */
static uint32_t          measStartTick = 0;
static float             lastTempC     = 25.0f; /* Ambient temp estimate for heater calc */

/* =========================================================================
 * Low-level SPI primitives
 * ========================================================================= */

static inline void cs_low(void)
{
    HAL_GPIO_WritePin(BARO2_CS_GPIO_Port, BARO2_CS_Pin, GPIO_PIN_RESET);
}

static inline void cs_high(void)
{
    HAL_GPIO_WritePin(BARO2_CS_GPIO_Port, BARO2_CS_Pin, GPIO_PIN_SET);
}

/* Read len bytes starting at reg (SPI address). Read auto-increments. */
static void spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    reg |= 0x80;    /* bit 7 = 1 signals a read */
    cs_low();
    HAL_SPI_Transmit(&hspi3, &reg, 1, 100);
    HAL_SPI_Receive(&hspi3, buf, len, 100);
    cs_high();
}

/* Write a single byte to reg (SPI address). Write does NOT auto-increment. */
static void spi_write(uint8_t reg, uint8_t data)
{
    reg &= 0x7F;    /* bit 7 = 0 signals a write */
    cs_low();
    HAL_SPI_Transmit(&hspi3, &reg, 1, 100);
    HAL_SPI_Transmit(&hspi3, &data, 1, 100);
    cs_high();
}

/* Switch SPI memory page.
 * The status register (0x73) is accessible from both pages.
 * Writes are ignored for read-only status bits; only bit 4 is R/W. */
static void set_page(uint8_t page)
{
    if (currentPage == page) return;
    uint8_t status;
    spi_read(REG_STATUS, &status, 1);
    if (page == 0)
        status &= ~(1U << 4);
    else
        status |=  (1U << 4);
    spi_write(REG_STATUS, status);
    currentPage = page;
}

/* =========================================================================
 * Compensation formulas
 * (Floating-point implementation from Bosch BME680 datasheet Section 3.5)
 * ========================================================================= */

/* Temperature (°C). Also computes calib.t_fine used by pressure and humidity. */
static float compensate_temperature(uint32_t adc_temp)
{
    float var1 = ((float)adc_temp / 16384.0f) - ((float)calib.par_t1 / 1024.0f);
    var1 *= (float)calib.par_t2;
    float var2 = ((float)adc_temp / 131072.0f) - ((float)calib.par_t1 / 8192.0f);
    var2 = (var2 * var2) * ((float)calib.par_t3 * 16.0f);
    calib.t_fine = var1 + var2;
    return calib.t_fine / 5120.0f;
}

/* Pressure (hPa). Requires t_fine set by compensate_temperature(). */
static float compensate_pressure(uint32_t adc_pres)
{
    float var1 = (calib.t_fine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * ((float)calib.par_p6 / 131072.0f);
    var2 += var1 * (float)calib.par_p5 * 2.0f;
    var2 = (var2 / 4.0f) + ((float)calib.par_p4 * 65536.0f);
    var1 = (((float)calib.par_p3 * var1 * var1) / 16384.0f
            + ((float)calib.par_p2 * var1)) / 524288.0f;
    var1 = (1.0f + (var1 / 32768.0f)) * (float)calib.par_p1;
    float pres = 1048576.0f - (float)adc_pres;
    pres = ((pres - (var2 / 4096.0f)) * 6250.0f) / var1;
    var1 = ((float)calib.par_p9 * pres * pres) / 2147483648.0f;
    var2 = pres * ((float)calib.par_p8 / 32768.0f);
    float var3 = (pres / 256.0f) * (pres / 256.0f) * (pres / 256.0f)
                 * ((float)calib.par_p10 / 131072.0f);
    pres += (var1 + var2 + var3 + ((float)calib.par_p7 * 128.0f)) / 16.0f;
    return pres / 100.0f;   /* Pa -> hPa */
}

/* Humidity (%RH, clamped 0-100). Requires t_fine set by compensate_temperature(). */
static float compensate_humidity(uint16_t adc_hum)
{
    float temp_scaled = calib.t_fine / 5120.0f;
    float var1 = (float)adc_hum
                 - ((float)calib.par_h1 * 16.0f)
                 - (((float)calib.par_h3 / 2.0f) * temp_scaled);
    float var2 = var1 * ((float)calib.par_h2 / 262144.0f)
                 * (1.0f + (((float)calib.par_h4 / 16384.0f) * temp_scaled)
                    + (((float)calib.par_h5 / 1048576.0f) * temp_scaled * temp_scaled));
    float var3 = (float)calib.par_h6 / 16384.0f;
    float var4 = (float)calib.par_h7 / 2097152.0f;
    float hum  = var2 + ((var3 + (var4 * temp_scaled)) * var2 * var2);
    if (hum > 100.0f) hum = 100.0f;
    if (hum <   0.0f) hum = 0.0f;
    return hum;
}

/* Gas resistance (Ohms). Uses lookup tables from BME680 datasheet Table 12. */
static float compensate_gas_resistance(uint16_t adc_gas, uint8_t gas_range)
{
    static const float lookup1[16] = {
        1.0f,     1.0f,     1.0f,     1.0f,
        1.0f,     0.99f,    1.0f,     0.992f,
        1.0f,     1.0f,     0.998f,   0.995f,
        1.0f,     0.99f,    1.0f,     1.0f
    };
    static const float lookup2[16] = {
        8000000.0f,   4000000.0f,   2000000.0f,   1000000.0f,
        499500.4995f, 248262.1648f, 125000.0f,    63004.03226f,
        31281.28128f, 15625.0f,     7812.5f,       3906.25f,
        1953.125f,    976.5625f,    488.28125f,    244.140625f
    };
    float var1 = (1340.0f + 5.0f * (float)calib.range_sw_err) * lookup1[gas_range];
    return var1 * lookup2[gas_range] / ((float)adc_gas - 512.0f + var1);
}

/* Calculate heater resistance register value for a given target temperature.
 * Uses lastTempC as ambient temperature estimate. */
static uint8_t calc_res_heat(uint16_t target_temp)
{
    float var1 = ((float)calib.par_g1 / 16.0f) + 49.0f;
    float var2 = (((float)calib.par_g2 / 32768.0f) * 0.0005f) + 0.00235f;
    float var3 = (float)calib.par_g3 / 1024.0f;
    float var4 = var1 * (1.0f + (var2 * (float)target_temp));
    float var5 = var4 + (var3 * lastTempC);
    return (uint8_t)(3.4f * ((var5 * (4.0f / (4.0f + (float)calib.res_heat_range))
                               * (1.0f / (1.0f + ((float)calib.res_heat_val * 0.002f))))
                              - 25.0f));
}

/* =========================================================================
 * Calibration data read
 * Reads two groups of factory-programmed calibration registers.
 * Group 1: I2C 0x89-0xA1 (Page 0, SPI 0x09), 25 bytes
 * Group 2: I2C 0xE1-0xEE (Page 0, SPI 0x61), 14 bytes
 * Plus three single-byte reads from Page 1.
 * ========================================================================= */
static void read_calibration(void)
{
    uint8_t b[25];

    /* --- Calibration group 1 (Page 0, SPI addr 0x09) --- */
    set_page(0);
    spi_read(CALIB1_SPI_ADDR, b, CALIB1_LEN);

    /* b[0]  = I2C 0x89 (unused)
     * b[1]  = I2C 0x8A = par_t2 LSB
     * b[2]  = I2C 0x8B = par_t2 MSB
     * b[3]  = I2C 0x8C = par_t3
     * b[4]  = I2C 0x8D (unused)
     * b[5]  = I2C 0x8E = par_p1 LSB
     * b[6]  = I2C 0x8F = par_p1 MSB
     * b[7]  = I2C 0x90 = par_p2 LSB
     * b[8]  = I2C 0x91 = par_p2 MSB
     * b[9]  = I2C 0x92 = par_p3
     * b[10] = I2C 0x93 (unused)
     * b[11] = I2C 0x94 = par_p4 LSB
     * b[12] = I2C 0x95 = par_p4 MSB
     * b[13] = I2C 0x96 = par_p5 LSB
     * b[14] = I2C 0x97 = par_p5 MSB
     * b[15] = I2C 0x98 = par_p7
     * b[16] = I2C 0x99 = par_p6
     * b[17] = I2C 0x9A (unused)
     * b[18] = I2C 0x9B (unused)
     * b[19] = I2C 0x9C = par_p8 LSB
     * b[20] = I2C 0x9D = par_p8 MSB
     * b[21] = I2C 0x9E = par_p9 LSB
     * b[22] = I2C 0x9F = par_p9 MSB
     * b[23] = I2C 0xA0 = par_p10
     * b[24] = I2C 0xA1 (unused) */
    calib.par_t2  = (int16_t)((uint16_t)b[2]  << 8 | b[1]);
    calib.par_t3  = (int8_t)b[3];
    calib.par_p1  = (uint16_t)b[6]  << 8 | b[5];
    calib.par_p2  = (int16_t)((uint16_t)b[8]  << 8 | b[7]);
    calib.par_p3  = (int8_t)b[9];
    calib.par_p4  = (int16_t)((uint16_t)b[12] << 8 | b[11]);
    calib.par_p5  = (int16_t)((uint16_t)b[14] << 8 | b[13]);
    calib.par_p7  = (int8_t)b[15];
    calib.par_p6  = (int8_t)b[16];
    calib.par_p8  = (int16_t)((uint16_t)b[20] << 8 | b[19]);
    calib.par_p9  = (int16_t)((uint16_t)b[22] << 8 | b[21]);
    calib.par_p10 = b[23];

    /* --- Calibration group 2 (Page 0, SPI addr 0x61) --- */
    uint8_t b2[14];
    spi_read(CALIB2_SPI_ADDR, b2, CALIB2_LEN);

    /* b2[0]  = I2C 0xE1 = par_h2 bits [11:4]
     * b2[1]  = I2C 0xE2 = par_h2 bits [3:0] | par_h1 bits [3:0]
     * b2[2]  = I2C 0xE3 = par_h1 bits [11:4]
     * b2[3]  = I2C 0xE4 = par_h3
     * b2[4]  = I2C 0xE5 = par_h4
     * b2[5]  = I2C 0xE6 = par_h5
     * b2[6]  = I2C 0xE7 = par_h6
     * b2[7]  = I2C 0xE8 = par_h7
     * b2[8]  = I2C 0xE9 = par_t1 LSB
     * b2[9]  = I2C 0xEA = par_t1 MSB
     * b2[10] = I2C 0xEB = par_g2 LSB
     * b2[11] = I2C 0xEC = par_g2 MSB
     * b2[12] = I2C 0xED = par_g1
     * b2[13] = I2C 0xEE = par_g3 */
    calib.par_h2  = (uint16_t)((uint16_t)b2[0] << 4 | b2[1] >> 4);
    calib.par_h1  = (uint16_t)((uint16_t)b2[2] << 4 | (b2[1] & 0x0F));
    calib.par_h3  = (int8_t)b2[3];
    calib.par_h4  = (int8_t)b2[4];
    calib.par_h5  = (int8_t)b2[5];
    calib.par_h6  = b2[6];
    calib.par_h7  = (int8_t)b2[7];
    calib.par_t1  = (uint16_t)b2[9] << 8 | b2[8];
    calib.par_g2  = (int16_t)((uint16_t)b2[11] << 8 | b2[10]);
    calib.par_g1  = (int8_t)b2[12];
    calib.par_g3  = (int8_t)b2[13];

    /* --- Page 1: heater calibration bytes --- */
    set_page(1);

    uint8_t tmp;
    spi_read(REG_RES_HEAT_VAL, &tmp, 1);
    calib.res_heat_val = (int8_t)tmp;

    spi_read(REG_RES_HEAT_RNG, &tmp, 1);
    calib.res_heat_range = (tmp >> 4) & 0x03;   /* bits [5:4] */

    spi_read(REG_RANGE_SW_ERR, &tmp, 1);
    calib.range_sw_err = (int8_t)((int8_t)(tmp & 0xF0) >> 4);  /* bits [7:4], signed */
}

/* =========================================================================
 * Public API
 * ========================================================================= */

uint8_t bme680_init(void)
{
    HAL_Delay(10);      /* Sensor startup time after power-on */

    /* Force page 0 (sensor default, but ensure driver tracks it correctly) */
    currentPage = 0xFF; /* Invalidate so set_page() always writes */
    set_page(0);

    /* Verify chip identity */
    uint8_t chip_id = 0;
    spi_read(REG_CHIP_ID, &chip_id, 1);
    if (chip_id != BME680_CHIP_ID_VAL) {
        myprintf("BME680 init FAIL: chip ID 0x%02X (expected 0x%02X)\r\n",
                 chip_id, BME680_CHIP_ID_VAL);
        state = BME680_STATE_ERROR;
        return BME680_ERROR;
    }

    /* Soft reset - brings all registers to power-on defaults, returns to page 0 */
    spi_write(REG_RESET, SOFT_RESET_CMD);
    HAL_Delay(10);
    currentPage = 0;    /* Reset restores page 0 */

    /* Read factory calibration data */
    read_calibration();

    /* Set IIR filter off (config register on page 1) */
    set_page(1);
    spi_write(REG_CONFIG, FILTER_OFF << 2);

    isInitialized = 1;
    state = BME680_STATE_IDLE;
    myprintf("BME680 init OK (chip ID 0x%02X)\r\n", BME680_CHIP_ID_VAL);
    return BME680_OK;
}

/* Enable or disable the gas sensor. Takes effect on the next triggered measurement.
 * Disabling saves ~100 ms per measurement and ~12 mA heater current.
 * When disabled, gas_resistance_ohms, gas_valid, and heat_stab in BME680_Data_t
 * will be 0 / invalid. */
void bme680_setGasEnabled(uint8_t enabled)
{
    gasEnabled = enabled ? 1 : 0;
}

/* Request a new measurement. Safe to call from idle or after data is ready.
 * Ignored if a measurement is already in progress. */
void bme680_triggerMeasurement(void)
{
    if (!isInitialized) return;
    if (state == BME680_STATE_WAIT_MEAS ||
        state == BME680_STATE_TRIGGER_PENDING) return;
    state = BME680_STATE_TRIGGER_PENDING;
}

/* Advance the state machine. Call repeatedly from the main loop.
 * Returns: BME680_OK    - idle or data ready
 *          BME680_BUSY  - measurement in progress
 *          BME680_ERROR - hardware or timeout fault */
uint8_t bme680_update(void)
{
    if (!isInitialized) return BME680_ERROR;

    switch (state)
    {
        case BME680_STATE_IDLE:
        case BME680_STATE_DATA_READY:
            return BME680_OK;

        case BME680_STATE_TRIGGER_PENDING:
        {
            set_page(1);

            /* Humidity oversampling must be written before ctrl_meas */
            spi_write(REG_CTRL_HUM, OSRS_H & 0x07);

            if (gasEnabled) {
                /* Heater on, gas conversion enabled, heater profile 0 */
                spi_write(REG_CTRL_GAS0, 0x00);
                spi_write(REG_CTRL_GAS1, (1U << 4) | 0x00);
                spi_write(REG_GAS_WAIT0, GAS_WAIT_VAL);
                spi_write(REG_RES_HEAT0, calc_res_heat(HEATER_TARGET_DEGC));
            } else {
                /* Heater forced off, gas conversion disabled */
                spi_write(REG_CTRL_GAS0, (1U << 3));   /* heat_off = 1 */
                spi_write(REG_CTRL_GAS1, 0x00);        /* run_gas = 0 */
            }

            /* Temperature/pressure oversampling + forced mode trigger (single write) */
            uint8_t ctrl_meas = (uint8_t)(((OSRS_T & 0x07U) << 5)
                                          | ((OSRS_P & 0x07U) << 2)
                                          | 0x01U);   /* 0x01 = forced mode */
            spi_write(REG_CTRL_MEAS, ctrl_meas);

            measStartTick = HAL_GetTick();
            state = BME680_STATE_WAIT_MEAS;
            return BME680_BUSY;
        }

        case BME680_STATE_WAIT_MEAS:
        {
            /* Timeout guard */
            if ((HAL_GetTick() - measStartTick) > MEAS_TIMEOUT_MS) {
                myprintf("BME680: measurement timeout\r\n");
                state = BME680_STATE_ERROR;
                return BME680_ERROR;
            }

            /* Poll new_data_0 flag (bit 7 of meas_status_0 at 0x1D, page 1) */
            set_page(1);
            uint8_t meas_status;
            spi_read(REG_MEAS_STATUS, &meas_status, 1);
            if (!(meas_status & 0x80)) {
                return BME680_BUSY;     /* Measurement not yet complete */
            }

            /* Burst-read all measurement registers 0x1D-0x2B (15 bytes) */
            uint8_t raw[15];
            spi_read(REG_MEAS_STATUS, raw, 15);

            /* Parse raw ADC values from burst buffer:
             * raw[0]       = meas_status_0 (0x1D)
             * raw[1]       = unused (0x1E)
             * raw[2..4]    = press_msb/lsb/xlsb (0x1F-0x21), 20-bit
             * raw[5..7]    = temp_msb/lsb/xlsb (0x22-0x24), 20-bit
             * raw[8..9]    = hum_msb/lsb (0x25-0x26), 16-bit
             * raw[10..12]  = unused (0x27-0x29)
             * raw[13]      = gas_r_msb (0x2A), gas_r[9:2]
             * raw[14]      = gas_r_lsb (0x2B), gas_r[1:0] | valid | stab | range */
            uint32_t adc_pres  = ((uint32_t)raw[2] << 12) | ((uint32_t)raw[3] << 4) | (raw[4] >> 4);
            uint32_t adc_temp  = ((uint32_t)raw[5] << 12) | ((uint32_t)raw[6] << 4) | (raw[7] >> 4);
            uint16_t adc_hum   = ((uint16_t)raw[8] << 8)  | raw[9];
            uint16_t adc_gas   = ((uint16_t)raw[13] << 2) | (raw[14] >> 6);
            uint8_t  gas_range = raw[14] & 0x0F;
            uint8_t  gas_valid = (raw[14] >> 5) & 0x01;
            uint8_t  heat_stab = (raw[14] >> 4) & 0x01;

            /* Compensate - temperature first (populates calib.t_fine for others) */
            latestData.temperature_degC = compensate_temperature(adc_temp);
            latestData.pressure_hPa     = compensate_pressure(adc_pres);
            latestData.humidity_pctRH   = compensate_humidity(adc_hum);
            latestData.gas_valid        = gas_valid;
            latestData.heat_stab        = heat_stab;
            /* Only compute gas resistance when the hardware confirms a valid reading.
             * When gas is disabled, gas_valid=0 and the ADC registers hold garbage. */
            latestData.gas_resistance_ohms = gas_valid
                                             ? compensate_gas_resistance(adc_gas, gas_range)
                                             : 0.0f;

            lastTempC = latestData.temperature_degC;    /* Update ambient estimate */

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
    return (state == BME680_STATE_DATA_READY) ? 1 : 0;
}

/* Returns the most recently compensated measurement data. */
BME680_Data_t bme680_getData(void)
{
    return latestData;
}
