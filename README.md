# Rocket Flight Data Computer

A flight data logger for model rockets built around the **STM32H743VGT6** microcontroller. It records inertial (accelerometer + gyroscope) data at 100Hz and environmental (pressure, temperature, humidity) data at 20Hz to an SD card in CSV format for post-flight analysis, graphing, and plotting.

---

## Table of Contents

0. [Future Improvements and Known Bugs](#future-improvements-and-known-bugs)
1. [Hardware Overview](#hardware-overview)
2. [Project Structure](#project-structure)
3. [Third-Party Libraries](#third-party-libraries)
4. [Module Architecture](#module-architecture)
5. [LED Behavior](#led-behavior)
6. [Cooperative Scheduler](#cooperative-scheduler)
7. [DataLogger](#datalogger)
8. [SD Card Driver](#sd-card-driver)
9. [Log File Format](#log-file-format)
10. [Debug UART Connection](#debug-uart-connection)
11. [SPI Bus & Pin Assignments](#spi-bus--pin-assignments)

---

## Future Improvements and Known Bugs

**Known Limitations and Bugs**
- Limitation/Bug: Currently the system doesn't seem to support SD card removal and re-insertion while the system remains powered and running.  In this case, the SD card seems to fail to mount on the second, and any subsequent insertions, however it should usually work on the first insertion or if the system is booted with the SD card already in.
- Limitation: This code currently only supports SD cards that support SPI mode, typically these cards tend to be less than 8GB.  Furthermore, only some cards and card manufacturers seems to work.  You will likely have the best luck with cards around 1-2GB.
- Limitation: Currently this code doesn't have LFN (long file name) support enabled, so filenames are limited to using the 8.3 format (8 characters for filename, period, then 3 characters for extension).
- Potential Bug: Sometimes the SD card will fail to mount the first time, this is probably because SD cards in SPI mode are very sensitive to clocking etc.  This issue seems to be very common for people trying to use SD cards in SPI mode with small microcontrollers such as this.

**Future Improvements**
- General
     - Fix known bugs listed above.
     - Reduce limitations listed above.
- SPI
     - Could create a custom SPI communication state machine that wraps the lower level HAL library code and change the SPI bus to be non-blocking using DMA or interrupts instead of the current blocking nature.  This would require a decent amount of work though and is not necessary at this time.
- LSM6DSO
     - Could experiment with increasing the SPI clock rate if a higher ODR (output data rate) is desired, and determine the max communication rate with the sensor.
- SD Card
     - Could experiment with increasing the SPI clock rate, and determine the max communication rate supported.
     - Verify support for having multiple files open and writing to multiple files at a time.  This could be useful in the future if we add a camera module and want to save/record pictures or videos during a flight.
- UART3 Debug Console
     - Could implement much better buffering and data handling, consider implementing software ring buffer.
     - Could rename the myprintf() function to something better and optimize it more.
     - Could create a debugPrint() function which is compiler optimized and will only print if “#define DEBUG” is defined.
- Camera
     - Add support for a camera module such as the ArduCam
- PCB improvements next iteration
     - Consider adding a button to start and stop data logging
     - Remove unused/unnecessary sensors
     - Add series current limiting resistors to RGB LED
     - Fix USB issue
     - Consider adding active control capabilities
- Python Data Analysis (post processing the log file)
     - Create the python data processor.  For more details on this see [Python Data Processor README](/Python_Data_Analyzer/README.md) and the [Flight Data Reference](/Python_Data_Analyzer/Flight_Data_Reference.md) documents.

## Hardware Overview

**MCU:** STM32H743VGT6 — ARM Cortex-M7, running at 64 MHz (internal HSI oscillator, no PLL)

| Peripheral | Interface | User Module | Description |
|---|---|---|---|
| [LSM6DSO32](STM32_CubeIDE_Rocket_Project/Middlewares/ST/lsm6dso/lsm6dso32_device.h) IMU | SPI1 (16-bit) | `Middlewares/ST/lsm6dso/lsm6dso32_device.c/.h` | 6-axis IMU — accelerometer up to ±32 g (scale configurable via `LSM6DSO_ACCEL_FS`), gyroscope up to ±2000 dps (scale configurable via `LSM6DSO_GYRO_FS`), 104 Hz ODR — see `lsm6dso32_device.h` |
| [BME680](STM32_CubeIDE_Rocket_Project/UserCode/bme680_device.h) Barometer | SPI3 (8-bit) | `UserCode/bme680_device.c/.h` | Pressure, temperature, humidity — forced-mode, 20 Hz triggered |
| SD Card | SPI2 (8-bit) | `UserCode/SD_Card.c/.h` | FAT filesystem via FatFs — CSV data logging |
| LED | GPIO (PB3) | `Core/Src/main.c` | Heartbeat blink (1 Hz); fast-blink on fatal error |

> **Note on LED:** PB3 is a test pad only in the current PCB schematic — however the code is setup for an LED to be connected to this pin.

> **IMU scale configuration:** The accelerometer and gyroscope full-scale ranges are set by two `#define`s at the top of [`lsm6dso32_device.h`](STM32_CubeIDE_Rocket_Project/Middlewares/ST/lsm6dso/lsm6dso32_device.h). Change only the `#define` — the correct conversion function is selected automatically at init time.
>
> | Define | Default | Options |
> |---|---|---|
> | `LSM6DSO_ACCEL_FS` | `LSM6DSO32_16g` | `LSM6DSO32_4g` / `_8g` / `_16g` / `_32g` |
> | `LSM6DSO_GYRO_FS` | `LSM6DSO32_2000dps` | `LSM6DSO32_125dps` / `_250dps` / `_500dps` / `_1000dps` / `_2000dps` |

---

## Project Structure

```
Rocket-Flight-Data-Computer/
├── STM32_CubeIDE_Rocket_Project/       # STM32CubeIDE firmware project
│   ├── Core/
│   │   ├── Inc/main.h                  # CubeMX-generated pin/peripheral definitions
│   │   └── Src/main.c                  # Startup, init, scheduler registration
│   ├── UserCode/                       # User-written application code
│   │   ├── DataLogger.c/.h             # Ties IMU + BME680 → SD card CSV rows
│   │   ├── Scheduler.c/.h              # Cooperative task scheduler
│   │   ├── SD_Card.c/.h                # SD card state machine + FatFs wrappers
│   │   └── bme680_device.c/.h          # BME680 wrapper around Bosch API
│   ├── Middlewares/
│   │   ├── Third_Party/BOSCH_BME/      # Bosch BME68x SensorAPI v4.4.8
│   │   │   ├── bme68x.c
│   │   │   ├── bme68x.h
│   │   │   └── bme68x_defs.h
│   │   ├── Third_Party/FatFs/          # Chan's FatFs (bundled via STM32CubeMX)
│   │   └── ST/lsm6dso/                 # ST LSM6DSO32 register-level driver
│   │       ├── lsm6dso32_device.c/.h   # High-level init/read functions
│   │       └── lsm6dso32_reg.c/.h      # Low-level register map (ST-provided)
│   └── Drivers/                        # STM32 HAL (CubeMX-generated)
├── PCB_Schematics_KiCAD/               # KiCad PCB schematic files
├── Python_Data_Analyzer/               # Post-flight data analysis scripts
├── Datasheets/                         # Component datasheets
└── README.md
```

> **CubeMX note:** Files in `Core/` are managed by STM32CubeMX. All user edits inside those files must stay within `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks to survive code regeneration.

---

## Third-Party Libraries

| Library | Version | Location in Repo | Source |
|---|---|---|---|
| Bosch BME68x SensorAPI | v4.4.8 | `Middlewares/Third_Party/BOSCH_BME/` | [github.com/boschsensortec/BME68x_SensorAPI](https://github.com/boschsensortec/BME68x_SensorAPI) |
| ST LSM6DSO32 driver | (STM32CubeH7) | `Middlewares/ST/lsm6dso/` | [github.com/STMicroelectronics/lsm6dso32-pid](https://github.com/STMicroelectronics/lsm6dso32-pid) |
| FatFs | R0.12c | `Middlewares/Third_Party/FatFs/` | Bundled by STM32CubeMX (Chan's FatFs) |
| STM32 HAL / BSP | STM32CubeH7 | `Drivers/` | Bundled by STM32CubeMX |

> The Bosch BME68x SensorAPI and ST LSM6DSO32 driver files are **not modified**. Custom wrappers (`bme680_device.c/.h` and `lsm6dso32_device.c/.h`) sit on top of them.

---

## Module Architecture

The firmware uses a non-blocking cooperative scheduler. There is no RTOS — all modules are polled from a single main loop via `Scheduler_Run()`. No `HAL_Delay()` calls exist in the main loop or in any task function.

```
main.c  (init + main loop)
  └── Scheduler_Run()
        │
        ├── DataLogger_StateMachine_Task  [every 5 ms]
        │     │
        │     ├── bme680_device           (BME680 wrapper)
        │     │     └── Bosch BME68x API  (bme68x.c)
        │     │
        │     ├── lsm6dso32_device        (IMU high-level driver)
        │     │     └── lsm6dso32_reg     (ST register-level driver)
        │     │
        │     └── SD_Card                 (file write wrappers)
        │           └── FatFs             (ff.c)
        │
        ├── SD_StateMachine               [every 100 ms]
        │     └── SD_Card (mount/detect state machine)
        │
        └── LED_Toggle                    [every 500 ms]
```

---

## LED Behavior

**Pin:** PB3 (`LED_Pin`, `LED_GPIO_Port`) — active-high. Note: PB3 is a test pad only in the current PCB schematic and is not connected to a discrete LED component.

The LED flash pattern indicates current system state at a glance:

| State | Pattern | Period | Rate |
|---|---|---|---|
| Running — not logging (no SD card or SD not yet mounted) | 500 ms on, 250 ms off | 750 ms | ~1.3 Hz |
| Running — actively logging to SD card | 250 ms on, 250 ms off | 500 ms | 2 Hz |
| Fatal error (`Error_Handler()`) | 100 ms on, 100 ms off | 200 ms | 5 Hz |
| Locked up, infinite loop, unpowered | Stuck on or stuck off | — | — |

The LED task (`LED_Task` in `main.c`) is registered with the scheduler at a 50 ms poll interval and manages its own asymmetric on/off timing internally using `HAL_GetTick()`. It queries `DataLogger_IsLogging()` each cycle to select the correct flash pattern. The `Error_Handler()` fast-blink runs as a busy-wait loop after interrupts are disabled, so it does not depend on the scheduler or SysTick.

---

## Cooperative Scheduler

**Files:** [`UserCode/Scheduler.c`](STM32_CubeIDE_Rocket_Project/UserCode/Scheduler.c) / [`Scheduler.h`](STM32_CubeIDE_Rocket_Project/UserCode/Scheduler.h)

A lightweight cooperative task scheduler based on `HAL_GetTick()` (1 ms SysTick resolution). Each task is a `void fn(void)` function pointer with an associated period in milliseconds.

### How It Works

1. `Scheduler_Init()` — resets the task list at startup.
2. `Scheduler_RegisterTask(fn, periodMs)` — registers a task; records `HAL_GetTick()` as the initial baseline so the first execution is deferred by one full period.
3. `Scheduler_Run()` — called in the `while(1)` main loop. Iterates all registered tasks; executes any whose elapsed time (`HAL_GetTick() - lastRunTick`) has reached or exceeded their period.

All elapsed-time comparisons use **unsigned 32-bit subtraction**, which is inherently wrap-around safe across the full 49.7-day `HAL_GetTick()` rollover range — the same pattern used throughout the codebase.

### Limits

- Maximum registered tasks: **8** (`SCHEDULER_MAX_TASKS`)
- Timing resolution: **1 ms** (HAL SysTick)
- Tasks must be **non-blocking** — any task that stalls blocks all other tasks

### Registered Tasks

| Task Function | Period | Effective Rate | Description |
|---|---|---|---|
| `DataLogger_StateMachine_Task` | 5 ms | 200 Hz | Polls BME680 state machine; reads IMU and writes CSV at 100 Hz sub-interval |
| `SD_StateMachine` | 100 ms | 10 Hz | Processes SD card insert/remove events, manages mount state |
| `LED_Toggle` | 500 ms | 2 Hz (1 Hz blink) | Heartbeat LED on PB3 |

---

## DataLogger

**Files:** [`UserCode/DataLogger.c`](STM32_CubeIDE_Rocket_Project/UserCode/DataLogger.c) / [`DataLogger.h`](STM32_CubeIDE_Rocket_Project/UserCode/DataLogger.h)

The DataLogger is the central orchestrator. It reads from both sensors and writes a row to the CSV log file on every IMU read cycle.

### Timing

| Interval Constant | Value | Action |
|---|---|---|
| Task schedule period | 5 ms | `DataLogger_StateMachine_Task` is called by the scheduler |
| `DATALOGGER_BME_TRIGGER_MS` | 50 ms (20 Hz) | Triggers a new BME680 forced-mode measurement |
| `DATALOGGER_CSV_WRITE_MS` | 10 ms (100 Hz) | Reads IMU + writes one CSV row |
| `DATALOGGER_SYNC_MS` | 1000 ms (1 Hz) | Calls `f_sync` to flush data to physical SD card media |

### Measurement Flow (per 5 ms call)

1. **Every call (5 ms):** `bme680_stateMachine()` is polled. If the BME680 has completed a measurement, the result is cached in `bmeCache`.
2. **Every 50 ms:** `bme680_triggerMeasurement()` starts a new forced-mode TPHG conversion.
3. **Every 10 ms:** `lsm6_readData()` reads fresh IMU data (accel, gyro, temperature), then `writeCSVRow()` writes one row to the open log file using the latest IMU data and the most recently cached BME680 data.
4. **Every 1000 ms:** `SD_FileSync()` flushes the file to physical media. Worst-case data loss on sudden power loss is 1 second of data.

### DataLogger State Machine

```
DL_IDLE ──(SD mounts)──► DL_OPENING ──(file created)──► DL_LOGGING
   ▲                                                          │
   └────────────────(SD card removed)─────────────────────────┘
                                          │
                                     (f_write error)
                                          ▼
                                      DL_ERROR
```

| State | Description |
|---|---|
| `DL_IDLE` | Sensors polled, but SD card not mounted or no file open |
| `DL_OPENING` | SD just mounted; scanning for next available log filename |
| `DL_LOGGING` | File open and active; writing CSV rows every 10 ms |
| `DL_ERROR` | File operation failed; waits for card removal and re-insertion |

> Before the BME680 completes its first measurement, the four BME columns in the CSV are written as empty fields (consecutive commas). Normal data appears from the second or third row onward.

---

## SD Card Driver

**Files:** [`UserCode/SD_Card.c`](STM32_CubeIDE_Rocket_Project/UserCode/SD_Card.c) / [`SD_Card.h`](STM32_CubeIDE_Rocket_Project/UserCode/SD_Card.h)

Provides a thin state-machine wrapper around FatFs for safer hot-plug card handling and simple file I/O.

### Card Detection

- **Pin:** `SD_CARD_DETECT` (PA8), active-low, hardware pull-up on the PCB
- **Mechanism:** EXTI interrupt fires on both rising and falling edges; `SD_StateMachine()` processes the event flag and reads the current pin level to distinguish insertion from removal

### State Machine

```
SD_NOT_PRESENT ──(card inserted)──► SD_PRESENT_UNMOUNTED
                                          │ (500 ms settle)
                                          ▼
                                     SD_MOUNTING
                                    /            \
                             (f_mount OK)    (f_mount fail)
                                  │                │
                                  ▼                ▼
                             SD_MOUNTED        SD_ERROR
                                  │
                        (card removed from any state)
                                  │
                                  ▼
                           SD_NOT_PRESENT
```

| State | Description |
|---|---|
| `SD_NOT_PRESENT` | No card detected |
| `SD_PRESENT_UNMOUNTED` | Card inserted; waiting 500 ms for power/signal settling |
| `SD_MOUNTING` | `f_mount()` in progress |
| `SD_MOUNTED` | Filesystem mounted; file operations available |
| `SD_ERROR` | Mount failed; waits for card removal before retrying |

### File Naming

Log files are named `LOG_XXXX.CSV` in **8.3 FAT format** (LFN disabled):

- Range: `LOG_0001.CSV` through `LOG_9999.CSV`
- On each new mount, the driver scans the root directory for the highest existing log number and creates the next one (e.g., if `LOG_0003.CSV` exists, creates `LOG_0004.CSV`)
- If 9999 is reached, `LOG_9999.CSV` is overwritten
- Files are never deleted automatically

### File API

All functions return `FR_NOT_READY` if the card is not in `SD_MOUNTED` state.

| Function | Description |
|---|---|
| `SD_FileOpen(fp, path, mode)` | Open or create a file |
| `SD_FileClose(fp)` | Close a file |
| `SD_FileWrite(fp, buf, len, bw)` | Write data |
| `SD_FileSync(fp)` | Flush data sector, FAT, and directory entry to physical media |
| `SD_DirOpen / SD_DirRead / SD_DirClose` | Directory traversal (used for log number scan) |
| `SD_GetState()` | Returns current `SD_State_t` |

---

## Log File Format

Each power-on or SD card insertion creates a new `LOG_XXXX.CSV` file. The first line is the CSV header; subsequent lines are data rows written at **100 Hz** (one row every 10 ms).

### CSV Header

```
WriteTime_ms,IMU_Timestamp_ms,Accel_New,Accel_X_mg,Accel_Y_mg,Accel_Z_mg,Gyro_New,Gyro_X_mdps,Gyro_Y_mdps,Gyro_Z_mdps,Temp_New,IMU_Temp_C,BME_Timestamp_ms,Pressure_hPa,BME_Temp_C,Humidity_pctRH,Altitude_AGL_ft
```

### Column Definitions

| # | Column Name | Unit | Format | Min | Max | Typical at Rest / Notes |
|---|---|---|---|---|---|---|
| 1 | `WriteTime_ms` | ms | `%lu` | 0 | 4,294,967,295 | Wall-clock tick at time of row write (rolls over after ~49.7 days) |
| 2 | `IMU_Timestamp_ms` | ms | `%lu` | 0 | 4,294,967,295 | `HAL_GetTick()` captured at SPI read time |
| 3 | `Accel_New` | flag | `%u` | 0 | 1 | 1 = fresh accelerometer data this row; 0 = repeated from previous read |
| 4 | `Accel_X_mg` | milli-g | `%.2f` | −32000 | +32000 | ~0 mg at rest (sensor X-axis horizontal); actual range depends on `LSM6DSO_ACCEL_FS` |
| 5 | `Accel_Y_mg` | milli-g | `%.2f` | −32000 | +32000 | ~0 mg at rest (sensor Y-axis horizontal); actual range depends on `LSM6DSO_ACCEL_FS` |
| 6 | `Accel_Z_mg` | milli-g | `%.2f` | −32000 | +32000 | ~+1000 mg at rest (1 g gravity on Z-axis); actual range depends on `LSM6DSO_ACCEL_FS` |
| 7 | `Gyro_New` | flag | `%u` | 0 | 1 | 1 = fresh gyroscope data this row |
| 8 | `Gyro_X_mdps` | milli-dps | `%.2f` | −2,000,000 | +2,000,000 | ~0 mdps at rest; actual range depends on `LSM6DSO_GRYO_FS` |
| 9 | `Gyro_Y_mdps` | milli-dps | `%.2f` | −2,000,000 | +2,000,000 | ~0 mdps at rest; actual range depends on `LSM6DSO_GRYO_FS` |
| 10 | `Gyro_Z_mdps` | milli-dps | `%.2f` | −2,000,000 | +2,000,000 | ~0 mdps at rest; actual range depends on `LSM6DSO_GRYO_FS` |
| 11 | `Temp_New` | flag | `%u` | 0 | 1 | 1 = fresh IMU die temperature this row |
| 12 | `IMU_Temp_C` | °C | `%.2f` | −40 | +85 | ~25 °C at room temperature |
| 13 | `BME_Timestamp_ms` | ms | `%lu` | 0 | 4,294,967,295 | `HAL_GetTick()` when BME680 measurement completed; **empty until first reading** |
| 14 | `Pressure_hPa` | hPa | `%.2f` | 300 | 1100 | ~1013 hPa at sea level; ~950 hPa at 500 m altitude; **empty until first reading** |
| 15 | `BME_Temp_C` | °C | `%.2f` | −40 | +85 | ~20–30 °C indoors; **empty until first reading** |
| 16 | `Humidity_pctRH` | %RH | `%.2f` | 0 | 100 | ~30–60 %RH indoors; **empty until first reading** |
| 17 | `Altitude_AGL_ft` | ft | `%.2f` | — | — | AGL altitude from barometric formula; reads 0.00 during the ground pressure calibration window; altitude computation begins once `GROUND_PRESSURE_NUM_SAMPLES` BME680 readings have been averaged into P₀; **empty until first BME reading** |

**Notes:**
- Columns 13–17 are blank (empty fields) until the BME680 completes its first measurement.
- The `Accel_New`, `Gyro_New`, and `Temp_New` flags indicate whether the LSM6DSO's data-ready status register reported fresh data at the time of that specific SPI read. Even when `0`, the value columns contain the most recent valid reading.
- `WriteTime_ms` and `IMU_Timestamp_ms` could differ by 1–2 ms due to SPI read overhead.
- `Altitude_AGL_ft` is computed on-board using `h = 44330 × (1 − (P/P₀)^0.1903) × 3.28084`. P₀ is the average of the first `GROUND_PRESSURE_NUM_SAMPLES` valid BME680 pressure readings after power-on or SD card insertion.
- Maximum CSV row size is approximately 147 characters (160-byte buffer allocated with 13-byte margin).

### Example Row

```
12345,12344,1,23.50,-15.25,998.00,1,120.30,-45.10,30.20,1,27.43,12300,1013.25,24.61,52.30,125.48
```

For even more details on this log file, see the [Flight Data Reference](/Python_Data_Analyzer/Flight_Data_Reference.md) document.

---

## Debug UART Connection

Debug messages are sent over **UART3** at **115200 baud, 8N1** using the `myprintf()` function (works like `printf`, outputs to UART3). This is useful for verifying sensor initialization and logging state transitions.

### Connecting a USB-to-UART/TTL Bridge

> **Important:** The PCB runs at **3.3 V logic levels**. Use a bridge that supports 3.3 V (or is 3.3 V tolerant). Do **not** use a 5 V TTL bridge without a level shifter.

| PCB Pin | PCB Signal | Connect To |
|---|---|---|
| PB10 | UART3 TX (PCB transmits here) | **RX** on USB bridge |
| PB11 | UART3 RX (PCB receives here) | **TX** on USB bridge |
| GND | Ground | **GND** on USB bridge |
| 3.3 V | Power reference | **3.3 V** (VCC) on USB bridge |

TX and RX are always crossed: the PCB's TX connects to the bridge's RX, and the PCB's RX connects to the bridge's TX.

On the host PC, open the bridge's COM/tty port at **115200 baud, 8 data bits, no parity, 1 stop bit** using any serial terminal (e.g., PuTTY, RealTerm, Tera Term, `screen`, `minicom`).

### Live CSV Echo to UART

To see CSV rows printed live to the debug UART during bench testing, set `DATALOGGER_UART_ECHO` to `1` in [`DataLogger.h`](STM32_CubeIDE_Rocket_Project/UserCode/DataLogger.h):

```c
#define DATALOGGER_UART_ECHO     1    /* 0 = disabled (default), 1 = enabled */
#define DATALOGGER_UART_ECHO_MS  500U /* print interval in ms (default 2 Hz) */
```

**Why the rate limit?** `myprintf()` is blocking — a worst-case 147-byte row takes ~12.8 ms to transmit at 115200 baud, which is longer than the 10 ms CSV write period. Printing at 100 Hz would also require ~14,700 bytes/sec, exceeding the UART's ~11,520 byte/sec capacity. At the default 500 ms interval, UART load is ~294 bytes/sec and the brief blocking window (~12.8 ms every 500 ms) has negligible impact on logging. Set `DATALOGGER_UART_ECHO` back to `0` before flight — the debug printing code is disabled by default and doesn't even compile when `0`.

### Startup Messages

On a successful boot you should see messages similar to:

```
LSM6DSO init OK
BME680 init OK (chip ID 0x61)
DataLogger init OK
SD card not present
```

When a card is inserted:

```
SD card inserted — mounting...
SD mounted OK
DataLogger: opened LOG_0005.CSV
```

---

## SPI Bus & Pin Assignments

### SPI Bus Configuration

| Peripheral | SPI Handle | Data Size | Clock Speed | Mode |
|---|---|---|---|---|
| LSM6DSO32 IMU | `hspi1` | 8-bit | ~4 MHz (64 MHz / 16) | Mode 0 (CPOL=Low, CPHA=1Edge) |
| SD Card | `hspi2` | 8-bit | ~250 kHz at init (64 MHz/256) → ~4 MHz at runtime (64 MHz/16) | Mode 0 (CPOL=Low, CPHA=1Edge) |
| BME680 | `hspi3` | 8-bit | ~4 MHz (64 MHz / 16) | Mode 0 (CPOL=Low, CPHA=1Edge) |

### Primary Pin Mappings

| Signal | CubeMX Name | GPIO | Pin | Active |
|---|---|---|---|---|
| LSM6DSO32 chip select | `LSM6DSO_CS_Pin` | GPIOA | PA3 | Low |
| SD card chip select | `SD_CS_Pin` | GPIOB | PB12 | Low |
| BME680 chip select | `BARO2_CS_Pin` | GPIOC | PC9 | Low |
| SD card detect | `SD_CARD_DETECT_Pin` | GPIOA | PA8 | Low (card present) |
| LED | `LED_Pin` | GPIOB | PB3 | High |

> All CS pins are configured as push-pull GPIO outputs. CS lines are driven by user code (not by the SPI peripheral hardware NSS). The SD card detect pin uses a hardware pull-up on the PCB and is connected to an EXTI line for interrupt-driven hot-plug detection.

#### UART3 Serial Debug Pins
| PCB Pin | PCB Signal | Connect To |
|---|---|---|
| PB10 | UART3 TX (PCB transmits here) | **RX** on USB bridge |
| PB11 | UART3 RX (PCB receives here) | **TX** on USB bridge |
| GND | Ground | **GND** on USB bridge |
| 3.3 V | Power reference | **3.3 V** (VCC) on USB bridge |
