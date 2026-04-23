This project uses an STM32H743VGT6 MCU along with several sensors.

The ones of particular interest that I’m working on right now are the LSM6DSO IMU (connected via SPI) and an SD card (connected via SPI) and the BME680 barometer (also connected via SPI).

## Directory Layout

- `PCB_Schematics_KiCAD/` — PCB schematics
- `STM32_CubeIDE_Rocket_Project/` — all STM32CubeIDE project code
  - `Core/Src/` and `Core/Inc/` — CubeMX-generated code; user edits go inside `/* USER CODE BEGIN/END */` blocks
  - `UserCode/` — additional user-written source files (e.g. SD_Card.c)
  - `Middlewares/ST/lsm6dso/` — LSM6DSO driver (not in `Drivers/`)
  - `Middlewares/Third_Party/BOSCH_BME/` — Bosch BME68x SensorAPI v4.4.8 (bme68x.c/h/defs.h)

## SPI Bus Assignments

| Peripheral | SPI Handle | Notes |
|---|---|---|
| LSM6DSO IMU | `hspi1` | 16-bit data size |
| SD Card | `hspi2` | 8-bit, used via FATFS; handle aliased as `SD_SPI_HANDLE` in main.h |
| BME680 | `hspi3` | 8-bit, Mode 0 (CPOL=Low, CPHA=1Edge) |

## CS Pin Mappings (defined in main.h)

| Signal | CubeMX name | GPIO | Pin |
|---|---|---|---|
| `LSM6DSO_CS` | `LSM6DSO_CS_Pin` | GPIOA | GPIO_PIN_3 |
| `SD_CS` | `SD_CS_Pin` | GPIOB | GPIO_PIN_12 |
| `BARO2_CS` | `BARO2_CS_Pin` | GPIOC | GPIO_PIN_9 | for BME680
| `CAM_CS` | `CAM_CS_Pin` | GPIOD | GPIO_PIN_0 |
| `SD_CARD_DETECT` | `SD_CARD_DETECT_Pin` | GPIOA | GPIO_PIN_8 |
| `BUZZER_PIN` | `BUZZER_PIN_Pin` | GPIOA | GPIO_PIN_10 |
| `LED` | `LED_Pin` | GPIOB | GPIO_PIN_3 |

## Debug / Logging

`myprintf()` (declared in `main.h`, defined in `main.c`) works like `printf` but sends output over UART3 at 115200 baud.

## CubeMX Code Generation

All edits to CubeMX-managed files (`main.c`, `main.h`, etc.) must be placed inside `/* USER CODE BEGIN <tag> */` / `/* USER CODE END <tag> */` blocks. Code outside these blocks will be overwritten the next time CubeMX regenerates the project.

## Status

- SD card driver (`UserCode/SD_Card.c`): functional
- LSM6DSO IMU driver (`Middlewares/ST/lsm6dso/`): functional
- BME680 barometer driver (`UserCode/bme680_device.c/h`): functional
  - Wraps the Bosch BME68x SensorAPI v4.4.8 (see `Middlewares/Third_Party/BOSCH_BME/`)
  - Forced mode, non-blocking state machine
  - SPI callbacks use `HAL_SPI_TransmitReceive` for STM32H7 FIFO safety
  - Gas sensor disabled by default (not needed for rocketry)

## More code generation notes
I like decently descriptive variable names, i.e. "filePointer" instead of "fp", or "numBytesWritten" instead of just "nbw" or "written".
I also like booleans to be named starting with "is" or "has" to make it clear they are booleans, especially in c where uint8_t is often used for booleans instead of a clear bool type.
I also like decent commenting, not overboard, but not super minimal either.
I also like decent function documentation explaining what the function does, its parameters, return values etc.  Sometimes it can be short 1 line, sometimes a full long explanation with @brief, @param, @return etc. tags.  Use your judgement, if the function is simple, obvious and straight forward, don't waste the space, but if its more complex or less clear/obvious what it does, consider being more verbose with its description.
