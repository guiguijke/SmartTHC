# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

SmartTHC — Arduino-based Torch Height Controller (THC) for CNC plasma cutting. Adjusts torch height via PID control on plasma arc voltage, with LCD menu and rotary encoder interface.

**Hardware target:** Arduino Uno R4 Minima (Renesas RA4M1)

## Build Commands (PlatformIO)

```bash
# Build (metric units, default environment)
pio build -e uno_r4_minima

# Build with imperial units (IPM)
pio build -e uno_r4_minima_imperial

# Upload to board
pio upload -e uno_r4_minima

# Serial monitor
pio device monitor --port COM3 --baud 115200
```

Cross-compilation environments for Raspberry Pi also available: `uno_r4_minima_RPi`, `uno_r4_minima_RPi_imperial`.

## Architecture

**Modular design** — each subsystem is a separate class, orchestrated by `main.cpp`.

| Module | File | Role |
|---|---|---|
| **THCController** | `src/THCController.h/cpp` | PID control, voltage ADC reading (dual-filter: fast EMA + slow circular buffer with 10x oversampling), anti-dive protection, stepper motor commands |
| **DisplayManager** | `src/DisplayManager.h/cpp` | 16x2 I2C LCD with 8 menu screens (setpoint, PID tuning, speed, correction factor), selective redraw to avoid flicker |
| **EncoderManager** | `src/EncoderManager.h/cpp` | KY-040 rotary encoder with state machine debouncing (IDLE→PRESSED→HELD→RELEASED) |
| **SpeedMonitor** | `src/SpeedMonitor.h/cpp` | Torch travel speed from X/Y step pulse interrupts, position history buffer for anti-dive |
| **EEPROMManager** | `src/EEPROMManager.h/cpp` | Persistent storage of 7 parameters with deferred writes (1s batching) and validation |
| **SerialCommand** | `src/SerialCommand.h/cpp` | Debug serial interface (115200 baud), status logging, `RESET_EEPROM` command |
| **Config.h** | `src/Config.h` | All pin definitions, timing intervals, default PID coefficients, thresholds — no magic numbers elsewhere |

**Main loop order** (`main.cpp`): encoder input → screen navigation → parameter adjustment → speed update → position history → THC/PID update (1kHz) → stepper run → LCD update (250ms) → serial commands → deferred EEPROM writes → loop stats.

## Key Configuration

- **Pin assignments and timing constants** are in `src/Config.h`
- **Mechanical constants** (steps/mm, voltage divider) are build flags in `platformio.ini`
- **Unit system** is compile-time: `USE_IMPERIAL=0` (metric) or `1` (imperial)
- PID runs at **1kHz**; display updates at **250ms**; speed calc at **50ms**

## Conventions

- CamelCase classes, camelCase methods/variables, UPPER_CASE constants
- Comments and code are in **English**
- Real-time constraints: non-blocking patterns throughout (no `delay()`), interrupt-based step counting, µs-precision timing
- Memory-conscious: circular buffers, cached LCD values, deferred EEPROM writes to minimize flash wear
- External libraries: `LiquidCrystal_I2C`, `AccelStepper`, `ArduPID` (managed by PlatformIO)
