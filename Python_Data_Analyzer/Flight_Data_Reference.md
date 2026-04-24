# Rocket Flight Computer — Post-Flight Data Analysis Reference

This document explains how to interpret and analyze the CSV log files produced by the rocket flight computer. It covers every column in the log format, the expected error budgets for each sensor, how to compute altitude from pressure, and practical tips for working with the data.

---

## Table of Contents

1. [Log File Overview](#1-log-file-overview)
2. [CSV Column Reference](#2-csv-column-reference)
3. [Accelerometer Data — Interpreting & Error Budget](#3-accelerometer-data--interpreting--error-budget)
4. [Gyroscope Data — Interpreting & Error Budget](#4-gyroscope-data--interpreting--error-budget)
5. [BME680 Environmental Data — Interpreting & Error Budget](#5-bme680-environmental-data--interpreting--error-budget)
6. [Computing Altitude from Pressure](#6-computing-altitude-from-pressure)
7. [Practical Analysis Tips](#7-practical-analysis-tips)

---

## 1. Log File Overview

Each power-on or SD card insertion creates a new file named `LOG_XXXX.CSV` (e.g., `LOG_0001.CSV`). The first line is a header row; all subsequent lines are data rows.

**IMU (LSM6DSO32)** data is written at **100 Hz** (one row every 10 ms).  
**Barometer (BME680)** data updates at **20 Hz** (every 50 ms trigger), so BME columns repeat their last reading for the 4 IMU rows between each BME update.

**Sensor configuration (default firmware):**

| Sensor | Setting | Value |
|---|---|---|
| LSM6DSO32 accel full-scale | `LSM6DSO_ACCEL_FS` in `lsm6dso32_device.h` | ±32 g |
| LSM6DSO32 gyro full-scale | `LSM6DSO_GYRO_FS` in `lsm6dso32_device.h` | ±2000 dps |
| LSM6DSO32 ODR | Both accel and gyro | 104 Hz |
| LSM6DSO32 power mode | Both accel and gyro | High-performance |
| BME680 pressure oversampling | Firmware `boschConf` | x16 |
| BME680 temperature oversampling | Firmware `boschConf` | x2 |
| BME680 humidity oversampling | Firmware `boschConf` | x1 |
| BME680 IIR filter | Firmware `boschConf` | Off |

---

## 2. CSV Column Reference

### Header

```
WriteTime_ms,IMU_Timestamp_ms,Accel_New,Accel_X_mg,Accel_Y_mg,Accel_Z_mg,Gyro_New,Gyro_X_mdps,Gyro_Y_mdps,Gyro_Z_mdps,Temp_New,IMU_Temp_C,BME_Timestamp_ms,Pressure_hPa,BME_Temp_C,Humidity_pctRH,Altitude_AGL_ft
```

### Column Definitions

| # | Name | Unit | Format | Range | Notes |
|---|---|---|---|---|---|
| 1 | `WriteTime_ms` | ms | `%lu` | 0 – 4,294,967,295 | `HAL_GetTick()` at the moment the row is written to the file. Rolls over after ~49.7 days. |
| 2 | `IMU_Timestamp_ms` | ms | `%lu` | 0 – 4,294,967,295 | `HAL_GetTick()` captured at the SPI read instant, just before the data is retrieved from the sensor. Typically 1–2 ms earlier than `WriteTime_ms` due to SPI overhead. Use this column for precise IMU timing. |
| 3 | `Accel_New` | flag | `%u` | 0 or 1 | **1** = the accelerometer status register reported fresh data this read. **0** = the status register was not ready; the X/Y/Z columns contain the most recent valid reading (repeated). See note below for why 0s appear. |
| 4 | `Accel_X_mg` | milli-g | `%.2f` | ±32000 | Linear acceleration, X-axis. 1000 mg = 1 g = 9.81 m/s². Range is dependent on `LSM6DSO_ACCEL_FS` scale setting. |
| 5 | `Accel_Y_mg` | milli-g | `%.2f` | ±32000 | Linear acceleration, Y-axis. Range is dependent on `LSM6DSO_ACCEL_FS` scale setting. |
| 6 | `Accel_Z_mg` | milli-g | `%.2f` | ±32000 | Linear acceleration, Z-axis. Range is dependent on `LSM6DSO_ACCEL_FS` scale setting. At rest on a level surface, this reads ~+1000 mg (Earth gravity). |
| 7 | `Gyro_New` | flag | `%u` | 0 or 1 | Same as `Accel_New` but for the gyroscope. |
| 8 | `Gyro_X_mdps` | milli-dps | `%.2f` | ±2,000,000 | Angular rate, X-axis. Range is dependent on `LSM6DSO_GYRO_FS` scale setting. |
| 9 | `Gyro_Y_mdps` | milli-dps | `%.2f` | ±2,000,000 | Angular rate, Y-axis. Range is dependent on `LSM6DSO_GYRO_FS` scale setting. |
| 10 | `Gyro_Z_mdps` | milli-dps | `%.2f` | ±2,000,000 | Angular rate, Z-axis. Range is dependent on `LSM6DSO_GYRO_FS` scale setting. |
| 11 | `Temp_New` | flag | `%u` | 0 or 1 | Same as `Accel_New` but for the IMU die temperature sensor. |
| 12 | `IMU_Temp_C` | °C | `%.2f` | −40 to +85 | LSM6DSO32 die temperature. Reflects the chip's internal temperature, which is slightly above true ambient due to self-heating. Useful for tracking thermal effects on gyro bias. |
| 13 | `BME_Timestamp_ms` | ms | `%lu` | 0 – 4,294,967,295 | `HAL_GetTick()` captured when the BME680 measurement completed. **Empty until the first BME reading is available** (see Section 7). This timestamp stays the same across the ~4 IMU rows between BME updates. |
| 14 | `Pressure_hPa` | hPa | `%.2f` | 300 – 1100 | Compensated absolute pressure. ~1013 hPa at sea level; ~982 hPa at ~250 m altitude. **Empty until first BME reading.** |
| 15 | `BME_Temp_C` | °C | `%.2f` | −40 to +85 | BME680 compensated temperature. Typically reads 1–3°C above true ambient due to sensor self-heating from the PCB. **Empty until first BME reading.** |
| 16 | `Humidity_pctRH` | %RH | `%.2f` | 0 – 100 | Compensated relative humidity. Typical indoors: 30–60% RH. **Empty until first BME reading.** |
| 17 | `Altitude_AGL_ft` | ft | `%.2f` | — | AGL altitude computed on-board from `Pressure_hPa` using the barometric formula (see Section 6). Reads 0.00 ft during the ground pressure calibration window; altitude computation begins once `GROUND_PRESSURE_NUM_SAMPLES` BME680 readings have been averaged into P₀. Updates at 20 Hz alongside the BME columns. **Empty until first BME reading.** |

### The LSM6DSO `Accel_New`, `Gryo_New`, and `Temp_New` Flags — Why They Happen

The firmware reads the IMU every 10 ms (100 Hz), but the sensor produces new data at 104 Hz (~9.615 ms per sample). These two clocks are independent and drift relative to each other at a **beat frequency** of:

```
104 Hz − 100 Hz = 4 Hz
```

Roughly 4 times per second, the firmware's 10 ms read lands in the brief window *before* the sensor's next 9.615 ms sample is ready. When this happens, the status register reports no new data, and the firmware uses the previous reading — flagged with `[value]_New = 0`. This is expected, harmless behavior. The data in the value columns is still valid (it is the most recent sample, at most ~9.6 ms old).

Use the `BME_Timestamp_ms` value to detect when new BME data was obtained, see notes below for more details on this.

### Timing Relationships

- **`WriteTime_ms` vs `IMU_Timestamp_ms`**: `IMU_Timestamp_ms` is captured right before the SPI transaction to the IMU, and `WriteTime_ms` is captured just before the `f_write` call. They differ by 1–2 ms (SPI transaction time). For computing velocities or angles by integration, use `IMU_Timestamp_ms`.

- **`BME_Timestamp_ms`**: This is the tick at which the BME680 measurement completed internally — i.e., when `bme680_stateMachine()` read the data out. It is not updated until the next BME measurement completes (every 50 ms), so for the 4–5 IMU rows between BME updates, `BME_Timestamp_ms`, `Pressure_hPa`, `BME_Temp_C`, `Humidity_pctRH`, and `Altitude_AGL_ft` are all identical (the cached previous reading).

---

## 3. Accelerometer Data — Interpreting & Error Budget

### Configuration

- **Full-scale range**: ±32 g (configurable via `LSM6DSO_ACCEL_FS` in `lsm6dso32_device.h`)
- **ODR**: 104 Hz (one new sample every ~9.6 ms)
- **Power mode**: High-performance
- **LSB size (min resolution) at ±16g**: 0.488 mg/LSB (from LSM6DSO32 datasheet Table 3) <- default
- **LSB size (min resolution) at ±32g**: 0.976 mg/LSB (from LSM6DSO32 datasheet Table 3)

### Expected Values at Rest

| Axis | Expected reading | Why |
|---|---|---|
| Z | ~+1000 mg | Gravity vector (1 g = 1000 mg). If the board is tilted, some gravity projects onto X and Y. |
| X, Y | ~0 mg (±20–50 mg typical) | Should be near zero on a level surface. Offset from true zero is the zero-g level offset. |

### Error Budget

**Noise (high-performance mode):**

The datasheet specifies acceleration noise density of **220 µg/√Hz** at FS=±32g in high-performance mode. To find the RMS noise in a single sample:

```
BW = ODR / 2 = 104 / 2 = 52 Hz   (the internal LPF1 cutoff)
σ_noise = 220 µg/√Hz × √52 Hz ≈ 1586 µg ≈ 1.6 mg RMS
```

At FS=±32g, the LSB is 0.976 mg, so 1.6 mg RMS ≈ 1.6 LSB of analog noise. The output will typically vary by ±2–3 LSBs (~2–3 mg) sample-to-sample at rest.

**Zero-g offset:**

Every MEMS accelerometer has a constant DC offset at true zero acceleration. The LSM6DSO32 datasheet specifies ±20 mg typical. This means the X or Y axis can read up to ±20 mg even when perfectly level. This can be calibrated out the same way as gyro bias (see Section 4).

**Summary table:**

| Error source | Value | Notes |
|---|---|---|
| Noise density | 220 µg/√Hz | High-performance mode, FS=±32g |
| RMS noise per sample | ~1.6 mg | At 104 Hz ODR, BW=52 Hz |
| Peak-to-peak noise (3σ) | ~5 mg | Expected at-rest variation |
| Zero-g offset | ±20 mg | Fixed DC offset per device, calibratable |
| Temp sensitivity change | ±0.007 %/°C | Negligible for short flights |

---

## 4. Gyroscope Data — Interpreting & Error Budget

### Configuration

- **Full-scale range**: ±2000 dps (configurable via `LSM6DSO_GYRO_FS` in `lsm6dso32_device.h`)
- **ODR**: 104 Hz
- **Power mode**: High-performance
- **LSB size (min resolution) at ±2000 dps**: 70 mdps/LSB (from LSM6DSO32 datasheet Table 3)

### Expected Values at Rest

All three axes should read near zero. However, every MEMS gyro has a **zero-rate offset** (DC bias) that shifts the readings by a constant amount. The LSM6DSO32 specifies ±500 mdps typical. So at rest you might see X = +210 mdps, Y = −420 mdps, Z = +350 mdps — none of these indicate actual rotation; they are DC offsets.

### Error Budget: Noise

**Analog noise (high-performance mode):**

The datasheet specifies gyro rate noise density of **3.8 mdps/√Hz** in high-performance mode (independent of ODR and FS).

```
BW = ODR / 2 = 52 Hz
σ_analog = 3.8 mdps/√Hz × √52 Hz = 3.8 × 7.21 ≈ 27 mdps RMS
```

**Quantization noise:**

The ADC rounds each measurement to the nearest LSB. The true analog value is uniformly distributed within ±0.5 LSB of the reported discrete value. The RMS of a uniform distribution over a range of width `w` is `w / √12` — this is a standard result from probability theory. The variance of a uniform distribution from `a` to `b` is `(b−a)² / 12`, so `σ = (b−a) / √12`. Here the width is 1 LSB:

```
σ_quantization = 70 mdps / √12 = 70 / 3.464 ≈ 20 mdps RMS
```

**Total noise (combined in quadrature):**

Analog noise and quantization noise are independent, so they add as root-sum-of-squares:

```
σ_total = √(σ_analog² + σ_quantization²) = √(27² + 20²) = √(729 + 400) ≈ 34 mdps RMS
```

**Key insight — quantization dominates at FS=±2000 dps:**

The analog noise (~27 mdps RMS) is *smaller than one LSB* (70 mdps). This means the output register can only move in 70 mdps steps. In practice, at rest you will see the gyro output hopping between adjacent quantization levels: values like 140, 210, 280, 350 mdps — always exact multiples of 70 mdps. This is not excessive noise; it is the ADC grid. The underlying analog accuracy is actually better than one LSB.

If you needed finer angular rate resolution, you would reduce the full-scale (e.g., FS=±250 dps → 8.75 mdps/LSB), and then the ~27 mdps analog noise would dominate and show up as several LSBs of variation.

**Expected at-rest variation summary:**

| Error source | Value | Notes |
|---|---|---|
| Analog noise density | 3.8 mdps/√Hz | High-performance mode, independent of FS |
| Analog RMS noise (104 Hz) | ~27 mdps | BW = ODR/2 = 52 Hz |
| Quantization noise | ~20 mdps RMS | 70 mdps LSB / √12 |
| Quantization error | 70 mdps | This is the value of the LSB at full scale |
| Total RMS noise | ~34 mdps | Quadrature sum |
| 3σ peak-to-peak envelope | ~102 mdps | Typically ±1–2 LSBs observed |
| Zero-rate DC offset | ±500 mdps typical | Per device, calibratable (see below) |
| Temp drift of offset | ±10 mdps/°C | 0.01 dps/°C from datasheet |

### DC Bias Subtraction (Gyro Calibration)

The zero-rate offset is a fixed DC bias that shifts all readings by a constant per axis. It is **stable within a single power-on session** and changes only slightly with temperature (±10 mdps/°C). It is **not reliable across power cycles** — the bias will be different each time the sensor powers on.

**Calibration procedure:**

1. Average N samples while the rocket is stationary on the pad (before launch)
2. Store the mean X, Y, Z as bias offsets
3. Subtract from all subsequent readings

With 200 samples at 100 Hz (2 seconds), the calibration uncertainty is:

```
σ_mean = σ_single / √N = 27 mdps / √200 ≈ 1.9 mdps
```

After subtraction, at-rest readings should sit within ±1 LSB (±70 mdps if at full scale) of zero.

**Residual integration drift:**

Even after bias calibration, any remaining residual bias integrates when computing angles. With a residual of ~10 mdps after good calibration:

```
Drift = 0.010 dps × flight_duration_seconds
```

For a 10-second burn: 0.1° of drift. For a 30-second coast: 0.3° total. Small enough for most model rocket analysis.

---

## 5. BME680 Environmental Data — Interpreting & Error Budget

### Configuration (firmware defaults in `bme680_device.c`)

- Pressure oversampling: x16
- Temperature oversampling: x2
- Humidity oversampling: x1
- IIR filter: Off (real-time, no lag)
- Gas sensor: Disabled (not needed for this project)
- Sample rate: 20 Hz (triggered every 50 ms by DataLogger)

### Pressure

| Parameter | Value | Source |
|---|---|---|
| Operating range | 300–1100 hPa | BME680 datasheet Table 7 |
| Absolute accuracy | ±0.6 hPa | 300–1100 hPa, 0–65°C |
| Relative accuracy | ±0.12 hPa | 700–1100 hPa, 25–40°C |
| RMS noise | 0.12 Pa = 0.0012 hPa | Full BW, x16 oversampling → equiv. ~1.7 cm altitude |
| Temperature coefficient | ±1.3 Pa/K (±10.9 cm/K) | Pressure reading shifts with temperature |
| Long-term stability | ±1.0 hPa/year | Slow drift over months |

#### Standard hPa to MSL conversions
- At sea level (0 m): ~1013 hPa.
- At 250 m altitude: ~982 hPa.
- Every 1 hPa ≈ 8.5 m altitude near sea level.

### Temperature (BME680)

| Parameter | Value | Notes |
|---|---|---|
| Absolute accuracy at 25°C | ±0.5°C | |
| Accuracy over 0–65°C | ±1.0°C | |
| Resolution | 0.01°C | |
| RMS noise | 0.005°C | |

**Important:** The BME680 temperature is measured by an internal sensor on the chip. It reflects the package temperature, which is influenced by self-heating from the PCB and neighboring components. Expect BME_Temp_C to read 1–3°C above true ambient air temperature. The IMU_Temp_C column has the same characteristic.

### Humidity

| Parameter | Value | Notes |
|---|---|---|
| Operating range | 0–100% RH | |
| Absolute accuracy | ±3% RH | At 20–80% RH, 25°C |
| Hysteresis | ±1.5% RH | |
| Resolution | 0.008% RH | |
| Response time | ~8 seconds | 63% response to step change |

The 8-second response time means rapid humidity changes (like flying through a cloud) will be smeared over several seconds in the data.

---

## 6. Computing Altitude from Pressure

### On-Board Altitude Logging

The firmware computes an estimated AGL altitude in real-time and logs it directly as the `Altitude_AGL_ft` column (column 17). Note that you can get a more accurate altitude in post-processing by using a different averaging window, filtering, or accounting for non-standard temperature lapse rates.

The ground reference pressure (P₀) is computed by averaging the **first `GROUND_PRESSURE_NUM_SAMPLES` valid BME680 readings** after power-on or SD card insertion. During this calibration window, `Altitude_AGL_ft` reads 0.00 (the firmware is assuming the rocket is at 0 ft AGL on the pad). Once the average is complete, altitude is computed and updated on every subsequent BME reading.

### Barometric Formula

The standard International Standard Atmosphere (ISA) hypsometric equation converts pressure to altitude (assuming standard atmospheric conditions and standard temperature lapse rate). The following is the formula used by the firmware:

```
h_meters = 44330 × (1 − (P / P₀)^0.1903)
h_feet   = h_meters × 3.28084
```

Where:
- `h` = altitude in meters
- `P` = measured pressure in hPa
- `P₀` = average of the first `GROUND_PRESSURE_NUM_SAMPLES` BME680 pressure readings
- `0.1903` = `(R × L) / (g × M)` using ISA constants (lapse rate L = 0.0065 K/m, R = 8.314 J/mol·K, g = 9.807 m/s², M = 0.02896 kg/mol)

### Computing AGL Altitude in Post-Processing

The altitude computation used in the firmware provides a good estimated altitude but assumes a standard temperature lapse rate and doesn't account for actual atmospheric conditions.

To get an AGL altitude, `P₀` should be the pressure at the launch site at the time of launch, **not** sea-level standard pressure (1013.25 hPa). Using standard sea-level pressure will give estimated MSL (mean sea level) altitude, not AGL (above ground level).  To get actual MSL altitude you would need to the know the equivalent sea level pressure at the launch site at the time of launch, or compute this using the known launch pad altitude, but for model rocket analysis we mostly care about AGL altitude anyways.

**How to get P₀ from the log file:**

Average the first several (20-200) valid `Pressure_hPa` readings (after the BME data appears and before any significant motion). This gives your launch-pad pressure baseline.

**Example Code**

```python
import pandas as pd
import numpy as np

df = pd.read_csv('LOG_0001.CSV')
# Drop rows with empty BME data
df_bme = df.dropna(subset=['Pressure_hPa'])
# Average the first 100 readings as the ground reference
P0 = df_bme['Pressure_hPa'].head(100).mean()
print(f"Ground reference pressure: {P0:.2f} hPa")

# Compute AGL altitude (meters and feet) for all rows
df['Altitude_AGL_m']  = 44330 * (1 - (df['Pressure_hPa'] / P0) ** 0.1903)
df['Altitude_AGL_ft'] = df['Altitude_AGL_m'] * 3.28084
```

### Altitude Accuracy

| Source | Contribution |
|---|---|
| Pressure RMS noise (x16 oversampling) | ~1.7 cm altitude noise |
| Absolute pressure accuracy (±0.6 hPa) | ~±5 m absolute altitude error |
| Relative accuracy within a flight (±0.12 hPa) | ~±1 m AGL accuracy |
| Temperature coefficient (±1.3 Pa/K) | ~±11 cm per 1°C change |

For AGL altitude during a flight, the **relative accuracy** is what matters — and at ±0.12 hPa, you get roughly **±1 m** AGL resolution for a typical flight. The absolute accuracy (±0.6 hPa) only affects absolute altitude, not the AGL delta from launch.

### Temperature Correction

The barometric formula above uses a fixed lapse rate (temperature decreasing linearly with altitude at 6.5°C/km). For more accurate results over longer altitude ranges, use the full hypsometric equation with the measured temperature:

```
h = (T_avg / L) × [(P₀ / P)^(R×L / (g×M)) − 1]
```

Where `T_avg` is the average temperature in Kelvin between ground and altitude. In practice, for model rocket flights under 1000 m AGL, the simple formula above is accurate to well under 1 m.

### Note on Dynamic Pressure Effects

If the BME680 is in a **sealed or partially-sealed** enclosure on the rocket, ram air pressure at high velocity can add a stagnation pressure on top of the static pressure, making the sensor read *higher* than true static pressure and therefore *lower* than true altitude. For a rocket traveling at 100 m/s (360 km/h), the dynamic pressure is:

```
q = 0.5 × ρ × v² = 0.5 × 1.225 × 100² ≈ 6125 Pa = 61 hPa
```

This would cause a significant altitude error. To avoid this, the sensor enclosure should be vented with a small port facing *perpendicular* to the airflow (not forward-facing), so it sees only static pressure.

---

## 7. Practical Analysis Tips

### Empty BME Columns at Log Start

The first few rows of some log files will have empty `BME_Timestamp_ms`, `Pressure_hPa`, `BME_Temp_C`, `Humidity_pctRH`, and `Altitude_AGL_ft` columns. This is expected and happens due to how the BME680 operates.

The BME680 runs in **forced mode** — it only performs a measurement when explicitly triggered, then returns to sleep. At startup, `DataLogger_Init()` triggers the first measurement, but it takes approximately 40–50 ms to complete (x16 pressure oversampling + x2 temperature + x1 humidity require multiple ADC cycles). Meanwhile, the scheduler is already writing CSV rows at 100 Hz. Until `bme680_stateMachine()` detects that the measurement is complete and sets an internal `bmeHasReturnedFirstReading` flag, the CSV writer outputs empty fields for those columns.

Typically, the first 3–5 rows will have empty BME columns. If the SD card took longer to mount (e.g., slow card settle time) or was inserted after the system had already been running for a bit, the BME may have already completed its first measurement before logging started — in which case all rows will have data

**In analysis**, drop rows where `Pressure_hPa` is blank or NaN before any barometric calculations. `Altitude_AGL_ft` will also be blank or NaN in the same rows.

### Timestamp Gaps from SD Card Sync

Every 1 second, the firmware calls `f_sync()` to flush buffered data to the physical SD card. This can take a variable amount of time (typically 5–40 ms depending on the card). During this flush, the scheduler is blocked, and you may see a gap in `WriteTime_ms` of ~30–40 ms instead of the normal 10 ms between rows.

These gaps are normal. When computing velocities or accelerations by differencing, check for and handle these gaps explicitly (use `IMU_Timestamp_ms` differences rather than assuming uniform 10 ms spacing).

```python
# Detect sync gaps
dt = df['IMU_Timestamp_ms'].diff()
sync_gaps = df[dt > 15]  # rows where gap exceeded 1.5× the normal 10 ms period
print(f"Found {len(sync_gaps)} sync gaps")
```

### Stale Data Flags (`*_New = 0`)

When `Accel_New`, `Gyro_New`, or `Temp_New` is 0, the value columns contain a repeated reading from the previous valid sample. The data is still usable — it is simply at most ~9.6 ms old rather than fresh. However, if you are computing derivatives (acceleration from velocity, etc.), you may want to skip or interpolate rows with stale flags.

```python
# Filter to only fresh IMU samples
df_fresh = df[df['Accel_New'] == 1].copy()
```

You may also want to determine when new BME680 data was obtained (vs old data repeated) via the `BME_Timestamp_ms` column data.

### Gyro Bias Calibration

Before integrating gyro data to compute angles, subtract the zero-rate bias. Use stationary data from the beginning of the log (before any motion):

```python
# Use the first 2 seconds of data as calibration window
# (an even better solution would be to first determine a stationary period from the data and use that)
cal_mask = df['WriteTime_ms'] < (df['WriteTime_ms'].iloc[0] + 2000)
gyro_bias_x = df.loc[cal_mask, 'Gyro_X_mdps'].mean()
gyro_bias_y = df.loc[cal_mask, 'Gyro_Y_mdps'].mean()
gyro_bias_z = df.loc[cal_mask, 'Gyro_Z_mdps'].mean()

df['Gyro_X_cal_mdps'] = df['Gyro_X_mdps'] - gyro_bias_x
df['Gyro_Y_cal_mdps'] = df['Gyro_Y_mdps'] - gyro_bias_y
df['Gyro_Z_cal_mdps'] = df['Gyro_Z_mdps'] - gyro_bias_z
```

With 200 samples (2 seconds at 100 Hz), the calibration residual is ~2 mdps, which causes less than 0.1° of integrated angle error over a 10-second burn.

### Detecting Launch

Look for a sustained spike in `Accel_Z_mg` (or whichever axis is the thrust axis) well above the 1000 mg at-rest gravity baseline:

```python
# Simple launch detection: Z accel exceeds 2g for more than 50 ms
LAUNCH_THRESHOLD_MG = 2000  # 2g
LAUNCH_DURATION_MS  = 50

df['is_launch_candidate'] = df['Accel_Z_mg'] > LAUNCH_THRESHOLD_MG
# Find first sustained window
launch_idx = None
for i in range(len(df)):
    window = df.iloc[i:i+5]  # 5 rows = 50 ms at 100 Hz
    if window['is_launch_candidate'].all():
        launch_idx = i
        break

if launch_idx is not None:
    launch_time_ms = df['IMU_Timestamp_ms'].iloc[launch_idx]
    print(f"Launch detected at {launch_time_ms} ms")
```

### Time Alignment: IMU vs BME

The IMU upates at 104Hz and logs at 100 Hz; the BME updates and logs at 20 Hz. For analysis that needs both (e.g., altitude + acceleration), be aware that:

- `WriteTime_ms` tells you when that line was written to the file
- `IMU_Timestamp_ms` tells you when that IMU reading was captured
- `BME_Timestamp_ms` tells you when that BME reading was captured
- Between BME updates, the pressure/temp/humidity columns repeat the last value
- For time-series plots, forward-fill BME data or resample to a common time base

```python
# Get unique BME readings (drop duplicates by BME timestamp)
df_bme_unique = df.drop_duplicates(subset='BME_Timestamp_ms').dropna(subset=['Pressure_hPa'])
```

---