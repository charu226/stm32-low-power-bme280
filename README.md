# Low-Power Environmental Monitoring Node (STM32F401 + BME280)

This project implements a low-power environmental sensing node on the STM32 Nucleo-F401RE.  
The firmware is designed around **STOP mode** operation, where the MCU remains in low-power sleep by default and wakes periodically using an **RTC wake-up interrupt** to perform a single environmental measurement before returning to sleep.

The project focuses on **power-aware firmware design**, correct clock recovery after STOP mode, and a **custom BME280 driver** implemented without vendor-provided sensor libraries (HAL used for I²C transport).

---

## Key Features

- RTC wake-up interrupt–driven architecture (no polling loops)
- STOP mode power management with clock and SysTick reinitialization
- Forced-mode BME280 measurements for minimal active time
- Custom BME280 driver:
  - Direct register access over I²C
  - Calibration data extraction from sensor NVM
  - Fixed-point compensation (Bosch reference, `t_fine`)
- UART output for logging/debugging
- Clock gating before sleep to reduce dynamic power

---

## Hardware

- **Board:** STM32 Nucleo-F401RE  
- **Sensor:** BME280 (I²C mode, address `0x76`)  
- **I²C:** I2C1 @ 100 kHz  
- **RTC Clock Source:** LSI  
- **UART:** USART2 (PA2 / PA3)

The BME280 module is powered at 3.3 V.  
I²C pull-ups are provided by the sensor module.

---

## Firmware Architecture

1. System boots and initializes clocks, peripherals, and RTC.
2. BME280 calibration coefficients are read once at startup.
3. RTC wake-up timer generates periodic interrupts.
4. On wake-up:
   - System clock is reconfigured (PLL restored)
   - A forced-mode BME280 measurement is triggered
   - Raw ADC values are read and compensated
   - Temperature, pressure, and humidity are printed over UART
5. Clocks and SysTick are disabled.
6. MCU re-enters STOP mode.

The RTC ISR only sets a flag; all processing occurs in the main context.

---

## Power Strategy

- STOP mode used between measurements
- SysTick suspended before sleep
- Peripheral clocks disabled before entering STOP
- Short, bounded delay used only for sensor conversion timing

---

## Design Constraints & Trade-offs

This project is intentionally built on the STM32F401 platform to focus on firmware-level low-power design concepts rather than absolute minimum current consumption.

- The STM32F401 is **not a low-power–optimized MCU** (no LPTIM, no STOP2 mode, limited power domains).
- The Nucleo-F401RE board includes on-board regulators, ST-Link, and LEDs that significantly limit achievable low-power current.
- Dynamic clock scaling is not implemented; the PLL is restored to its default configuration after each STOP-mode wake-up.
- UART is used strictly for debug visibility and development validation, not as a production telemetry interface.
- No I²C error recovery or bus fault handling is implemented.

These constraints were accepted to keep the design focused on **correct STOP-mode usage, RTC wake-up handling, and deterministic sensor acquisition**.

---

## Build & Flash

- **Toolchain:** STM32CubeIDE  
- Open `p1.ioc` and generate code  
- Build and flash to STM32 Nucleo-F401RE  
- View output via UART (115200 baud)
