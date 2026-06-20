# SmartTHC — Agent Guide

This document is written for AI coding agents. It assumes no prior knowledge of the project. All comments, documentation, and identifiers in the codebase are in English.

---

## Project overview

SmartTHC is an open-source **Torch Height Controller (THC)** for CNC plasma cutting machines. It runs on an **Arduino Uno R4 Minima** (Renesas RA4M1 / ARM Cortex-M4) and uses a PID loop on the plasma arc voltage to command a Z-axis stepper motor, keeping the torch at the correct height during a cut.

Key features:

- 1 kHz PID control loop
- Dual voltage filtering: fast EMA (PID input) + slow 200-sample average (anti-dive reference)
- Explicit anti-dive Z-lift when arc voltage spikes (e.g. crossing a void or a previously cut path)
- Motion-gated THC activation to avoid triggering during the pierce phase
- THC_OFF re-stabilization delay
- Watchdog Timer (WDT) for EMI-induced hang recovery
- 16x2 I2C LCD menu with KY-040 rotary encoder
- Persistent EEPROM parameter storage with deferred writes
- Compile-time metric/imperial unit selection
- Structured `key=value` serial logging and case-insensitive commands

License: **GNU General Public License v3 (GPL v3)**. Any modifications or derivative works must remain under GPL v3.

---

## Technology stack

- **Build system:** PlatformIO Core / PlatformIO IDE for VS Code
- **Target platform:** `renesas-ra` (Arduino Uno R4 Minima)
- **Framework:** Arduino (Arduino Renesas UNO core, `framework-arduinorenesas-uno`)
- **Language:** C++ (Arduino), compiled as `gnu++17` / `gnu11`
- **MCU:** Renesas RA4M1 (ARM Cortex-M4 @ 48 MHz, hardware FPU)
- **External libraries (managed by PlatformIO):**
  - `marcoschwartz/LiquidCrystal_I2C @ ^1.1.4`
  - `waspinator/AccelStepper @ ^1.64`
  - `powerbroker2/ArduPID @ ^0.2.1` (pulls in `FireTimer`)
- **System libraries used:** `EEPROM`, `Wire`, `WDT` (Arduino Uno R4 WDT library)

---

## Repository layout

```text
.
├── platformio.ini          # Build environments, monitor settings, build flags
├── README.md               # Human-facing project documentation
├── CHANGELOG.md            # Release history
├── AGENTS.md               # This file: guidance for AI coding agents
├── LICENSE                 # GPL v3 license
├── include/                # Intentionally unused (generic PlatformIO template README)
├── lib/                    # Intentionally unused (generic PlatformIO template README)
├── src/                    # All application source
│   ├── Config.h            # Pins, constants, defaults, parameter limits
│   ├── main.cpp            # Setup, main loop, orchestration
│   ├── THCController.h/cpp # Voltage reading, filtering, PID, anti-dive, motor control
│   ├── DisplayManager.h/cpp# 16x2 I2C LCD menu and status icons
│   ├── EncoderManager.h/cpp# KY-040 rotary encoder + button debounce
│   ├── SpeedMonitor.h/cpp  # X/Y step-pulse interrupts, speed, motion-gate logic
│   ├── EEPROMManager.h/cpp # Parameter persistence with deferred writes
│   └── SerialCommand.h/cpp # Serial CLI, status logging, event stream
└── test/
    ├── README              # Generic PlatformIO unit-test README
    └── main copy.cpp       # Legacy monolithic version (reference snapshot, not a test)
```

`test/main copy.cpp` is a pre-refactor, single-file snapshot. It is **not** an automated unit test and is **not** built by PlatformIO test commands. Do not modify it unintentionally.

---

## Build and test commands

> Note: `pio` (PlatformIO Core) is required. If it is not installed, follow the [PlatformIO installation guide](https://platformio.org/install).

### Build

```bash
# Default environment — metric units
pio build -e uno_r4_minima

# Imperial units (IPM)
pio build -e uno_r4_minima_imperial

# Raspberry Pi cross-compilation environments (custom toolchain path)
pio build -e uno_r4_minima_RPi
pio build -e uno_r4_minima_RPi_imperial
```

The default environment is `uno_r4_minima` (see `default_envs` in `platformio.ini`).

### Upload / flash

```bash
# Flash the default build to a connected Arduino Uno R4 Minima
pio upload -e uno_r4_minima
```

The Uno R4 Minima uses a **DFU bootloader**. `setup()` intentionally waits **3 seconds** before arming the watchdog so the host has a stable window to complete the 1200-baud touch + `dfu-util` transfer. See `CHANGELOG.md` and `src/main.cpp` for details.

### Serial monitor

```bash
# Default port (auto-detect)
pio device monitor

# Explicit port example (Windows)
pio device monitor --port COM3 --baud 115200
```

Serial settings are already configured in `platformio.ini`:

- Baud: `115200`
- Filter: `send_on_enter`
- End-of-line: `LF`
- Echo: `yes`

### Testing

There is **no configured PlatformIO unit-test suite** at this time. `pio test` will run but there are no test files other than the legacy snapshot in `test/main copy.cpp`.

If you add tests, place them under `test/` following the [PlatformIO Unit Testing](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html) conventions. Keep hardware-dependent code behind native/native-test guards where possible.

---

## Configuration

### Hardware constants (build flags in `platformio.ini`)

These are exposed as `-D` build flags so they can be changed without editing source code:

| Flag | Default | Description |
|------|---------|-------------|
| `STEPS_PER_MM_X` | `200.0` | X-axis steps per mm |
| `STEPS_PER_MM_Y` | `200.0` | Y-axis steps per mm |
| `STEPS_PER_MM_Z` | `50.0` | Z-axis steps per mm (default targets NEMA17 1.8° / 1/8 µstep / 8 mm leadscrew) |
| `ANTI_DIVE_LIFT_MM` | `3.0` | Emergency Z retract height when anti-dive triggers |
| `Z_DIR_INVERT` | `0` | Set to `1` to invert the DIR pin if the Z stepper moves the wrong way on bench test |
| `DEFAULT_VOLTAGEDIVIDER` | `83.27` | Plasma voltage attenuation ratio applied to the ADC reading |
| `USE_IMPERIAL` | `0` | `1` = imperial units (IPM/inches) |

There are also three commented-out, **safety-critical** overrides in `platformio.ini`:

- `STEPPER_MAX_SPEED`
- `STEPPER_ACCELERATION`
- `MAX_CUT_SPEED`

Do not uncomment them unless you are prepared to re-tune the PID gains from scratch. They are tightly coupled to `KP / KI / KD`.

### Firmware constants (`src/Config.h`)

- Pin assignments for the Uno R4 Minima
- Timing intervals (PID at 1 kHz, display at 250 ms, speed at 50 ms, serial log at 150 ms)
- Default PID coefficients (`Kp=30.0`, `Ki=7.5`, `Kd=2.0`)
- Anti-dive thresholds and motion-gate delays
- Parameter limits (`SETPOINT_MIN/MAX`, `KP_MAX`, etc.)

Avoid magic numbers outside `Config.h`. Any `#define` constants are `UPPER_CASE`; runtime constants are usually `camelCase` or `UPPER_CASE` where they are global fixed values.

---

## Architecture and module divisions

The firmware is **modular object-oriented C++** orchestrated by `src/main.cpp`.

### Main loop order (`src/main.cpp`)

1. Refresh the watchdog.
2. Update the rotary encoder and detect button clicks.
3. Handle screen transitions (8-screen menu).
4. Apply parameter adjustments from encoder deltas.
5. Update the speed monitor and record Z position history.
6. Run the THC/PID update at 1 kHz.
7. Pump the stepper motor (`runMotor()`).
8. Update the LCD every 250 ms.
9. Process serial commands and log status.
10. Flush deferred EEPROM writes.
11. Track loop execution time and emit stats every 10 s.

### Modules

| Module | File(s) | Responsibility |
|--------|---------|----------------|
| `THCController` | `src/THCController.h/cpp` | ADC voltage reading, dual filtering, PID compute, anti-dive detection, Z stepper commands, THC gating chain |
| `DisplayManager` | `src/DisplayManager.h/cpp` | 16x2 I2C LCD, 8-screen menu, status icons, blinking anti-dive message, selective redraw to reduce flicker |
| `EncoderManager` | `src/EncoderManager.h/cpp` | KY-040 rotary encoder decoding, button state-machine debounce, click lockout during rotation |
| `SpeedMonitor` | `src/SpeedMonitor.h/cpp` | X/Y step-pulse ISRs, torch speed calculation, hysteresis-based motion detection, cut-motion gate, position history |
| `EEPROMManager` | `src/EEPROMManager.h/cpp` | Load/save 7 tunable parameters, validation, deferred writes (1 s batching) to reduce flash wear |
| `SerialCommand` | `src/SerialCommand.h/cpp` | 115200 serial CLI, `DEBUG`/`STATUS`/`RESET_EEPROM`/`HELP`, structured `key=value` logging, edge-triggered event stream |
| `Config` | `src/Config.h` | All pin definitions, timing intervals, default PID coefficients, thresholds — no magic numbers elsewhere |

### THC gating chain

The firmware labels the exact gate preventing THC activation. The chain (in order) is:

`PLASMA_OFF` → `WAIT_STAB` → `THC_SIG_OFF` → `ENABLE_OFF` → `ARC_LOST` → `WAIT_RESTAB` → `WAIT_MOTION` → `WAIT_SPEED` → `ANTI_DIVE` → `THC_ACTIVE` → `ARMED`

A serial state label therefore answers the question: *“Why isn’t THC running right now?”*

---

## Code style guidelines

Follow the existing style. Do not introduce new conventions without a good reason.

- **Classes:** `PascalCase` (`THCController`, `DisplayManager`)
- **Methods and variables:** `camelCase` (`currentScreen`, `lastPidTime`)
- **Constants / macros:** `UPPER_CASE` (`PID_INTERVAL_US`, `SETPOINT_MIN`)
- **Comments and documentation:** English
- **Indentation:** 4 spaces
- **Braces:** K&R-style opening brace on the same line
- **No `delay()` in the main loop.** The only permitted blocking delay is the 3 s DFU boot window in `setup()`.
- Keep code **non-blocking** and **interrupt-safe**:
  - Use `noInterrupts()` / `interrupts()` around reads/writes to volatile ISR-shared counters.
  - Keep ISRs short (they only increment counters and timestamps).
- Centralize constants in `src/Config.h` or `platformio.ini` build flags.
- Prefer explicit state machines over implicit state flags.
- Memory-conscious patterns: circular buffers, cached LCD values, deferred EEPROM writes to minimize flash wear.

---

## Testing instructions

There is no automated test harness today. Verify changes manually:

1. **Build all relevant environments:**
   ```bash
   pio build -e uno_r4_minima
   pio build -e uno_r4_minima_imperial
   ```
2. **Check compiler warnings.** The project should build cleanly. Resolve warnings rather than suppressing them without cause.
3. **Hardware-in-the-loop bench test** before any real cut:
   - Tie `ENABLE` LOW, `THC_OFF` HIGH, `ARC_OK` HIGH.
   - Feed `STEP_X` / `STEP_Y` with a function generator or pulse source.
   - Apply a known plasma voltage through a 50:1 divider to `A0`.
   - Send `STATUS` and `DEBUG` over serial; confirm `state=THC_ACTIVE` when all gates clear.
   - Verify Z direction: voltage **below** setpoint → Z moves **up**; voltage **above** setpoint → Z moves **down**.
   - Verify anti-dive: jump voltage high → Z lifts by `ANTI_DIVE_LIFT_MM`.
   - If any direction is wrong, change `-D Z_DIR_INVERT=1` in `platformio.ini` and re-flash.

Do **not** test on real material until bench polarity has been confirmed. An inverted anti-dive actively drives the torch into the workpiece.

---

## Deployment / flashing

1. Build the desired environment.
2. Connect the Uno R4 Minima via USB.
3. Run `pio upload -e uno_r4_minima`.
4. Open `pio device monitor` to interact with the serial CLI.

### Serial commands

Commands are case-insensitive. Unknown commands echo back a hint.

| Command | Description |
|---------|-------------|
| `DEBUG` | Toggle periodic `key=value` status lines and edge-triggered `EV:` events |
| `STATUS` | Print an immediate `key=value` status snapshot |
| `RESET_EEPROM` | Restore all tunable parameters to firmware defaults |
| `HELP` | List commands |

---

## Safety considerations

This firmware controls a **plasma cutting machine** and a **high-current arc**. Treat every change as safety-critical.

- **Z polarity is the most dangerous configuration error.** A wrong sign causes the anti-dive lift to become a dive into the cut piece. Always verify on the bench first.
- **Do not change `STEPPER_MAX_SPEED`, `STEPPER_ACCELERATION`, or `MAX_CUT_SPEED`** without re-tuning PID from scratch. The defaults are coupled to the gains and are intentionally aggressive.
- **The WDT timeout is 5.5 s** (near the library maximum). A boot delay of 3 s precedes watchdog arming to allow re-flashing.
- **EEPROM writes are deferred 1 s.** This reduces flash wear but means a power cycle within ~1 s of a parameter change may lose that change.
- **Anti-dive uses an aggressive motion envelope** (`ANTI_DIVE_LIFT_SPEED=5000`, `ANTI_DIVE_LIFT_ACCEL=20000`). The lift height is configurable via `ANTI_DIVE_LIFT_MM`; do not reduce speed/accel without careful testing.
- All digital inputs related to plasma/THC gating use active levels documented in `Config.h`:
  - `PLASMA_PIN` LOW = arc OK
  - `ENABLE_PIN` LOW = THC enabled (pull-up input)
  - `THC_OFF_PIN` HIGH = THC allowed / signal active

---

## Notes for AI agents

- `Config.h` is the single source of truth for pins, defaults, and limits. When asked to make a value tunable, prefer adding it there or exposing it as a `platformio.ini` build flag.
- `main.cpp` is thin. Business logic belongs in the appropriate module class.
- If modifying the THC gating chain, update both `THCController::updateTHCState()` and `SerialCommand::currentStateLabel()` / `thcInactiveReason()` so the serial diagnostics stay accurate.
- If adding a new serial command, follow the existing case-insensitive pattern and provide feedback for unknown commands.
- When touching EEPROM, always validate loaded values against the ranges in `Config.h` and use `EEPROMManager::scheduleSave*()` + `update()` for deferred writes.
- The `.vscode/` files are auto-generated by PlatformIO. Do not hand-edit `c_cpp_properties.json`; update `platformio.ini` instead.
