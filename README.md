# Smart THC

![Smart THC Logo](https://github.com/guiguijke/SmartTHC/blob/main/SmartTHC_LOGO_PNG.png)

**Smart THC** is an open-source Arduino-based Torch Height Controller (THC) for CNC plasma cutting machines. It dynamically adjusts torch height based on plasma arc voltage using PID control, delivering clean and accurate cuts. Features a user-friendly LCD interface, real-time parameter tuning via rotary encoder, and robust protection mechanisms.

Licensed under the [GNU General Public License v3 (GPL v3)](#license).

## Features

- **PID-based height control** running at 1kHz for responsive torch adjustment
- **Anti-dive protection** with adaptive timing — prevents torch diving on corners and small features
- **THC_OFF re-stabilization** (300ms delay) — prevents torch dive on small oblong holes
- **Motion-gated THC activation** — THC starts only after confirmed XY cut motion, with a configurable post-motion delay to avoid pierce-phase triggering
- **Watchdog Timer (WDT)** — auto-reboot on EMI-induced hangs in plasma environment
- **8-screen LCD menu** with rotary encoder navigation (setpoint, PID tuning, speed, correction factor)
- **Real-time monitoring** — arc voltage, torch speed, and system status on 16x2 LCD
- **Dual voltage filtering** — fast EMA for PID input + slow 200-sample average for anti-dive reference
- **Speed monitoring** via X/Y step pulse interrupts with threshold-based activation
- **Persistent EEPROM storage** with deferred writes (1s batching) to minimize flash wear
- **Metric / Imperial support** (compile-time configurable)
- **Serial debug interface** at 115200 baud with live status logging

## Architecture

Modular design — each subsystem is a separate class, orchestrated by `main.cpp`:

| Module | Role |
|---|---|
| **THCController** | PID control, voltage ADC reading, anti-dive protection, stepper motor commands |
| **DisplayManager** | 16x2 I2C LCD with 8 screens, selective redraw, blinking anti-dive message |
| **EncoderManager** | KY-040 rotary encoder with state machine debouncing |
| **SpeedMonitor** | Torch speed from X/Y step interrupts, hysteresis-based motion detection, and cut-start confirmation for THC gating |
| **EEPROMManager** | Persistent storage of 7 parameters with deferred writes and validation |
| **SerialCommand** | Debug serial interface, status logging, `RESET_EEPROM` command |
| **Config.h** | All pin definitions, timing intervals, defaults — zero magic numbers |

## Hardware Requirements

- **Arduino Uno R4 Minima** (or WiFi variant)
- **16x2 LCD** with I2C interface (default address: 0x27)
- **Stepper motor + driver** (e.g., A4988) for Z-axis torch movement
- **KY-040 rotary encoder** for menu navigation
- **Plasma arc voltage input** via voltage divider on A0
- **X/Y step signals** from CNC controller (interrupt pins 2, 3)
- **THC control signals**: plasma arc OK (pin 12), THC enable (pin 10), THC off (pin 11)

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/guiguijke/SmartTHC.git
   ```

2. **Open in VS Code** with the [PlatformIO extension](https://platformio.org/install/ide?install=vscode). PlatformIO will auto-detect `platformio.ini` and install dependencies.

3. **Build and upload**:
   ```bash
   # Build (metric units, default)
   pio build -e uno_r4_minima

   # Build with imperial units
   pio build -e uno_r4_minima_imperial

   # Upload to board
   pio upload -e uno_r4_minima

   # Serial monitor
   pio device monitor --baud 115200
   ```

4. **Configure** pin assignments and mechanical constants in `src/Config.h` and `platformio.ini` build flags.

## Configuration

All constants are centralized in `src/Config.h`. Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `STABILIZATION_DELAY` | 750ms | Plasma stabilization before PID activates |
| `THC_ON_RESTAB_DELAY` | 300ms | Re-stabilization after THC_OFF → THC_ON |
| `CUT_MOTION_CONFIRM_DELAY` | 200ms | Continuous XY motion validation before cut is considered started |
| `THC_AFTER_CUT_START_DELAY` | 500ms | Additional fixed delay after confirmed cut start before THC can activate |
| `CUT_SPEED_HYSTERESIS_RATIO` | 0.1 | Speed threshold hysteresis ratio to avoid ON/OFF chatter |
| `DEFAULT_SETPOINT` | 110V | Target arc voltage |
| `DEFAULT_KP / KI / KD` | 30 / 7.5 / 2.0 | PID coefficients |
| `DROP_THRESHOLD` | 5V | Anti-dive activation threshold |
| `STEPPER_DEADZONE` | 1V | PID dead zone |

Mechanical constants (`STEPS_PER_MM_X/Y/Z`, `DEFAULT_VOLTAGEDIVIDER`) are set as build flags in `platformio.ini`.

- **THC timing model** combines plasma stabilization, THC_OFF re-stabilization, confirmed XY motion, and a fixed post-motion delay before PID engagement

## Dependencies

Managed automatically by PlatformIO:
- [LiquidCrystal_I2C](https://github.com/johnrickman/LiquidCrystal_I2C) ^1.1.4
- [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) ^1.64
- [ArduPID](https://github.com/PowerBroker2/ArduPID) ^0.2.1

## Premium Documentation

While the code is open source, a premium documentation package is available with step-by-step hardware setup guides, tuning tips, and CNC integration guides.

[Purchase Premium Documentation](https://shop.aplasma.fr/b/zClKk)

## License

Smart THC is licensed under the [GNU General Public License v3 (GPL v3)](LICENSE). You are free to use, modify, and distribute the code, provided all derivative works are also licensed under GPL v3.

## Contact

Questions or feedback? Open an issue on [GitHub Issues](https://github.com/guiguijke/SmartTHC/issues) or join the [Discord](https://discord.gg/Z9JJdjPDb4) server.

---

**Acknowledgments**: Thanks to the authors of `LiquidCrystal_I2C`, `AccelStepper`, and `ArduPID` for their excellent libraries.
