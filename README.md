# Smart THC

![Smart THC Logo](https://github.com/guiguijke/SmartTHC/blob/main/SmartTHC_LOGO_PNG.png)

**Smart THC** is an open-source Arduino-based Torch Height Controller (THC) for CNC plasma cutting machines. It dynamically adjusts torch height based on plasma arc voltage using PID control, delivering clean and accurate cuts. Features a user-friendly LCD interface, real-time parameter tuning via rotary encoder, and robust protection mechanisms.

Licensed under the [GNU General Public License v3 (GPL v3)](#license).

## Features

- **PID-based height control** running at 1 kHz for responsive torch adjustment
- **Explicit anti-dive Z lift** — when plasma voltage spikes (typically over a void or a previously cut piece), the firmware commands a fast, parameterizable Z retract (`ANTI_DIVE_LIFT_MM`, default 3 mm) on an aggressive speed/accel envelope (~120 ms for 3 mm) so the torch clears the work before re-entering metal. Prevents the classic "dive into the cut piece" crash.
- **THC_OFF re-stabilization** (300 ms delay) — prevents torch dive on small oblong holes
- **Motion-gated THC activation** — THC starts only after confirmed XY cut motion, with a configurable post-motion delay to avoid pierce-phase triggering
- **Plasma-gated voltage filter** — the anti-dive slow reference is frozen and re-seeded on plasma transitions, keeping it clean through extinction transients and arc-over-void events
- **Configurable Z polarity** — `Z_DIR_INVERT` build flag flips the DIR output at the pin level so the firmware's "positive = up" contract holds regardless of driver wiring (affects both PID and lift in one shot)
- **Watchdog Timer (WDT)** — auto-reboot on EMI-induced hangs in plasma environment (5.5 s timeout with a 3 s boot DFU window so flashing isn't blocked; robust against the common Uno R4 FSP-enum pitfall)
- **8-screen LCD menu** with rotary encoder navigation (setpoint, PID tuning, speed, correction factor)
- **Real-time monitoring** — arc voltage, torch speed, and system status on 16x2 LCD
- **Dual voltage filtering** — fast EMA for PID input + slow 200-sample average for anti-dive reference
- **Speed monitoring** via X/Y step pulse interrupts with threshold-based activation
- **Persistent EEPROM storage** with deferred writes (1 s batching) to minimize flash wear
- **Metric / Imperial support** (compile-time configurable)
- **Structured serial logging** at 115200 baud — compact `key=value` status lines plus edge-triggered event lines on every state transition, designed to be grep/awk friendly and directly plottable
- **Case-insensitive serial commands** with explicit "unknown command" feedback — `help`, `Help`, `HELP` all work; typos echo back with a usage hint instead of silently dropping

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

   # Serial monitor (baud/echo/EOL are pre-configured in platformio.ini)
   pio device monitor
   ```

4. **Configure** pin assignments and mechanical constants in `src/Config.h` and `platformio.ini` build flags.

## Configuration

All firmware constants are centralized in `src/Config.h`. Hardware-dependent values are exposed as `platformio.ini` build flags so you can adjust them without forking the source.

### Build-flag overrides (`platformio.ini`)

| Flag | Default | Description |
|---|---|---|
| `STEPS_PER_MM_X` | 200.0 | X axis steps/mm (matches your CNC controller) |
| `STEPS_PER_MM_Y` | 200.0 | Y axis steps/mm |
| `STEPS_PER_MM_Z` | 50.0 | Z axis steps/mm (defaults to NEMA17 1.8° / 8 mm leadscrew / 1/8 µstep) |
| `ANTI_DIVE_LIFT_MM` | 3.0 | Emergency Z retract height (mm) when anti-dive triggers |
| `Z_DIR_INVERT` | 0 | Set to `1` if your Z stepper turns the wrong way on bench test — fixes PID and lift polarity together |
| `DEFAULT_VOLTAGEDIVIDER` | 83.27 | Plasma voltage attenuation ratio (e.g. 50:1 divider feeding 0–5 V into A0) |
| `USE_IMPERIAL` | 0 | Switch to imperial units (IPM, inches) at compile time |

Three additional motor-envelope flags (`STEPPER_MAX_SPEED`, `STEPPER_ACCELERATION`, `MAX_CUT_SPEED`) are available in `platformio.ini` but **commented out by default** and gated behind a hard warning. They are tightly coupled to the PID gains — changing them invalidates your tuning. See the "DANGER ZONE" block in `platformio.ini` for details.

### In-firmware constants (`src/Config.h`)

| Parameter | Default | Description |
|---|---|---|
| `STABILIZATION_DELAY` | 750 ms | Plasma stabilization before PID activates |
| `THC_ON_RESTAB_DELAY` | 300 ms | Re-stabilization after THC_OFF → THC_ON |
| `CUT_MOTION_CONFIRM_DELAY` | 200 ms | Continuous XY motion validation before cut is considered started |
| `THC_AFTER_CUT_START_DELAY` | 500 ms | Additional fixed delay after confirmed cut start before THC can activate |
| `CUT_SPEED_HYSTERESIS_RATIO` | 0.1 | Speed threshold hysteresis ratio to avoid ON/OFF chatter |
| `DEFAULT_SETPOINT` | 110 V | Target arc voltage |
| `DEFAULT_KP / KI / KD` | 30 / 7.5 / 2.0 | PID coefficients (encoder-tunable up to `KP_MAX=1500`, `KI_MAX=50`, `KD_MAX=100`) |
| `DROP_THRESHOLD` / `RETURN_THRESHOLD` | 5 V / 3 V | Anti-dive activation / deactivation thresholds |
| `MAX_ANTI_DIVE_DURATION` | 1000 ms | Upper bound on anti-dive duration (force-deactivate even if voltage hasn't recovered) |
| `ANTI_DIVE_LIFT_SPEED / _ACCEL` | 5000 / 20000 | Emergency lift motion envelope (faster than PID) |
| `STEPPER_DEADZONE` | 1 V | PID dead zone |

**THC timing model** combines plasma stabilization, THC_OFF re-stabilization, confirmed XY motion, and a fixed post-motion delay before PID engagement.

## Bench testing & first-cut checklist

The anti-dive lift trusts that positive Z steps move the torch **up**. Before any cut on real material, verify this on the bench:

1. Power the controller. Wire `ENABLE` (pin 10) to GND, `THC_OFF` (pin 11) to +5 V (or driven HIGH by your CNC controller), `ARC_OK` (pin 12) HIGH. Feed `STEP_X` / `STEP_Y` from a function generator at ≥ 1 kHz so the speed gate clears (1 kHz at `STEPS_PER_MM_X = 40` ≈ 1500 mm/min — adjust to your own steps/mm).
2. Drive the plasma voltage input (50:1 attenuated) from a bench PSU. Send `STATUS` over serial — you should see `state=THC_ACTIVE` once all gates clear.
3. Set voltage **below setpoint** (e.g. 90 V vs 110 V). Z should drive **up**.
4. Set voltage **above setpoint** (e.g. 130 V). Z should drive **down**.
5. Jump voltage sharply (110 V → 130 V) to trigger anti-dive. Z should **lift by `ANTI_DIVE_LIFT_MM`** at the emergency speed.

If any direction is wrong, set `-D Z_DIR_INVERT=1` in `platformio.ini`, rebuild, re-flash. Both PID and lift will correct in one shot.

A polarity-inverted anti-dive doesn't just fail to help — it actively drives the torch **into** the cut piece. Don't skip this check.

## Serial logging

Serial output is gated by the `DEBUG` command (toggle). Two kinds of lines are emitted:

- **Periodic status** (one line per `LOG_INTERVAL`):
  ```
  t=162412 st=THC_ACTIVE v=121.2/120.9 tgt=122.0 out=+0 spd=590 z=+1 ad=off
  ```
  `t` = ms since boot · `st` = state label · `v` = fast/slow voltage · `tgt` = setpoint ·
  `out` = PID output in steps/s · `spd` = torch speed · `z` = Z step position ·
  `ad` = anti-dive flag.

- **Edge-triggered events** on every real state transition:
  ```
  EV: t=159926 plasma stabilized (v=137.3V)
  EV: t=162104 THC_SIG asserted
  EV: t=162404 THC re-stab delay DONE
  EV: t=162404 THC active (v=120.7 tgt=122.0)
  EV: t=176705 THC inactive reason=THC_SIG_LOW
  EV: t=180908 anti-dive TRIGGERED fast=128.5 slow=118.2 drop=+10.3V
  ```

State labels mirror the THC gating chain, so the current label is also the answer to "why isn't THC running right now":
`PLASMA_OFF`, `WAIT_STAB`, `THC_SIG_OFF`, `ENABLE_OFF`, `ARC_LOST`,
`WAIT_RESTAB`, `WAIT_MOTION`, `WAIT_SPEED`, `ANTI_DIVE`, `THC_ACTIVE`, `ARMED`.

Available serial commands: `DEBUG` (toggle logging), `STATUS` (instant snapshot), `RESET_EEPROM`, `HELP`. Commands are **case-insensitive** — `help`, `Help`, `HELP` all work — and any unrecognized input echoes back `unknown command: <x> — type HELP` so you never have to wonder whether the link is alive.

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
