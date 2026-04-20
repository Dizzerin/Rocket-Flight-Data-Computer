This project uses an STM32H743VGT6 MCU along with several sensors.

The ones of particular interest that I’m working on right now are the LSM6DSO IMU (connected via SPI) and an SD card (connected via SPI) and the BME680 barometer (also connected via SPI).

## Directory Layout

- `PCB_Schematics_KiCAD/` — PCB schematics
- `STM32_CubeIDE_Rocket_Project/` — all STM32CubeIDE project code
  - `Core/Src/` and `Core/Inc/` — CubeMX-generated code; user edits go inside `/* USER CODE BEGIN/END */` blocks
  - `UserCode/` — additional user-written source files (e.g. SD_Card.c)
  - `Middlewares/ST/lsm6dso/` — LSM6DSO driver (not in `Drivers/`)

## SPI Bus Assignments

| Peripheral | SPI Handle | Notes |
|---|---|---|
| LSM6DSO IMU | `hspi1` | 16-bit data size |
| SD Card | `hspi2` | 8-bit, used via FATFS; handle aliased as `SD_SPI_HANDLE` in main.h |
| SPI3 | `hspi3` | 8-bit, purpose TBD (possibly BME680 or camera) |

## CS Pin Mappings (defined in main.h)

| Signal | GPIO | Pin |
|---|---|---|
| `LSM6DSO_CS` | GPIOA | GPIO_PIN_3 |
| `SD_CS` | GPIOB | GPIO_PIN_12 |
| `BME680_CS` | GPIOA | GPIO_PIN_9 |
| `CAM_CS` | GPIOD | GPIO_PIN_0 |
| `SD_CARD_DETECT` | GPIOA | GPIO_PIN_8 |
| `BUZZER_PIN` | GPIOA | GPIO_PIN_10 |
| `LED` | GPIOB | GPIO_PIN_3 |

## Debug / Logging

`myprintf()` (declared in `main.h`, defined in `main.c`) works like `printf` but sends output over UART3 at 115200 baud.

## CubeMX Code Generation

All edits to CubeMX-managed files (`main.c`, `main.h`, etc.) must be placed inside `/* USER CODE BEGIN <tag> */` / `/* USER CODE END <tag> */` blocks. Code outside these blocks will be overwritten the next time CubeMX regenerates the project.

## Status

- SD card driver (`UserCode/SD_Card.c`): functional
- LSM6DSO IMU driver (`Middlewares/ST/lsm6dso/`): functional
- BME680 barometer driver: not yet written
