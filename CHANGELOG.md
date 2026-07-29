# Changelog

All notable changes to this project are documented in this file.

## [2.6.0] - 2026-07-28

### Added
- **Live Z-height readout on the main LCD screen.** The 4 status icons (enable / direction-arrow / THC / plasma) on screen 0 have been replaced by a live Z-axis delta value, so the torch height is visible during a cut without scrolling through a menu. New layout:
  ```
  Act→ 121.2V   590
  Tgt→ 122.0V Z+1.2
  ```
  The `Z+1.2` field shows the Z-axis position in millimetres **relative to the cut-start height**: it is zeroed at the exact moment the THC takes ownership of the Z axis (the `thcActive` false→true edge, i.e. when all gates clear and the THC begins controlling height), so each cut starts at `Z 0.0` and the readout tracks drift during that cut. It reflects both normal PID corrections and anti-dive lifts. When the THC is idle the field shows `Z----` (no reference available). Absolute height would require a Z home switch, which the current hardware does not have — this relative readout is what the stepper position can actually tell us.

### Changed
- **THC state is now encoded in the label arrows** (`Act:` / `Tgt:`) instead of the old icon column:
  - `Act:` and `Tgt:` — THC idle
  - `Tgt→` with `Act:` — THC engaged (owns Z) but holding steady (PID output inside the deadzone)
  - `Tgt→` with `Act→` — THC engaged **and** actively correcting
  This is finer-grained than the previous single direction icon and frees the column for the Z readout. The arrow is a custom CGRAM glyph (`CHAR_ARROW_RIGHT`); the HD44780 ROM arrow at `0x7E` was deliberately avoided because it renders as a left arrow on the A00 character ROM that most I2C backpacks ship with.

### Fixed
- **Build failure introduced by the new Z readout.** `THCController::getZDeltaMm()` was declared `const` but calls `AccelStepper::currentPosition()`, which is not a `const` method, so every translation unit including `THCController.h` failed with `error: passing 'const AccelStepper' as 'this' argument discards qualifiers`. The getter is now non-`const`, consistent with `getMotorPosition()` right above it. No behavioral change.

### Internal
- **Dead-code cleanup.** A second full audit (the first was at v2.3.0) removed 10 confirmed-dead items with no behavioral change: `SPEED_UNIT` and `DIST_PER_STEP_Z` (Config.h), two orphan function prototypes (main.cpp), the unused `tempVoltageCorrectionFactor` member (THCController), the unused `lastAntiDiveDisplayActive` cache field (DisplayManager), the write-only `lastRotationDelta` member (EncoderManager), and the write-only `totalStepX`/`totalStepY` counters (SpeedMonitor). The 7 old custom-character slots that were only consumed by the deleted status-icon code were also removed.

### Notes
- **No motor, PID, anti-dive, or safety-path change.** The Z readout is display-only and reads from the position the AccelStepper already tracks. Z polarity follows the existing `Z_DIR_INVERT` semantics — **bench-verify the sign of the Z readout before cutting**: a positive delta should mean the torch has risen above the cut-start height, a negative delta that it has dropped.
- Serial telemetry is unaffected. The absolute stepper position is still reported as `pos=` in `STATUS` / `DEBUG` logs; the LCD `Z` field is a separate, relative, display-only value.
- Inspired by field feedback from Russ S.

## [2.5.3] - 2026-07-27

### Fixed
- **Stale `DEFAULT_VOLTAGEDIVIDER` value in `Config.h`, `AGENTS.md`, and `README.md`.** `platformio.ini` is the project's source of truth and overrides `Config.h` through the `#ifndef` pattern, so the build flag (`50.0`) has always won at compile time and the fallback value (`83.27`) in `Config.h` was dead code — but the three files disagreed, and the documentation (83.27) did not match what actually compiled (50.0). The fallback and both docs are now aligned to `50.0`. **No behavioral change** for anyone building from `platformio.ini`: the compiled value is unchanged. Anyone building `Config.h` outside PlatformIO (or reading the docs) now sees the correct value.

### Notes
- The `README.md` description of the flag was also clarified: `DEFAULT_VOLTAGEDIVIDER` is a hardware-calibration value (`V_plasma / V_at_ADC`) determined by the physical voltage-divider resistors feeding `A0`, not a universal default. Each builder must measure it for their own analog front-end.
- Thanks to Russ S. for spotting the inconsistency.

## [2.5.2] - 2026-07-26

### Fixed
- **Kp, Ki and Kd did not persist across power cycles (Uno R4 Minima).** The EEPROM address layout was inherited from an AVR-era assumption where `double` is 4 bytes. On the RA4M1 (ARM Cortex-M4, hard FPU) a `double` is 8 bytes (binary64), so the 4-byte-spaced Kp/Ki/Kd slots overlapped each other in flash, and the "EEPROM initialized" magic flag sat on top of the upper bytes of Kd. Every write of one gain partially clobbered the next. Symptoms reported in the field and now explained: Kp and Ki reading back as `0.0` after a power cycle (a corrupted read that lands on exactly `0.0` passes the `>= 0` validation, so it is never replaced with the default), Kd surviving inconsistently, and persistence requiring several set / power-cycle iterations to "stick". Identical on two different boards — consistent with a firmware bug, not a write-timing issue.

  The `double` slots (Kp/Ki/Kd) are now spaced **8 bytes apart** (`EEPROM_KP_ADDR=16`, `EEPROM_KI_ADDR=24`, `EEPROM_KD_ADDR=32`). The `float` slots (setpoint, correction factor, cut speed, threshold ratio) were already 4 bytes apart and remain correct.

### Changed
- **Forced EEPROM re-initialization on first boot with v2.5.2.** The "initialized" flag has been moved to a previously-virgin address (`EEPROM_INITIALIZED_FLAG` 28 → 40) **and** its magic value bumped (`0xAA` → `0xAB`). Both changes guarantee that every board previously flashed with v2.5.1 or earlier will detect an uninitialized EEPROM on its first v2.5.2 boot and cleanly overwrite the corrupted Kp/Ki/Kd slots with defaults. **Action required after upgrading:** re-enter your tuned PID gains (and cut speed) via the LCD or serial, then power-cycle to confirm they persist.

### Notes
- This is a data-layout fix only. The deferred-write batching, validation ranges, and save/load API are unchanged. No other behavior is affected.
- Thanks to Russ S. for the field report that surfaced this — his PIC18 EEPROM-timing anecdote was the clue that pointed at a write-path bug rather than a controller fault.

## [2.5.1] - 2026-07-11

### Fixed
- **Z-ownership relays transferred during the pierce phase.** `SWITCH1`/`SWITCH2` (which hand the Z axis from the CNC controller to the THC) used to switch on `plasmaPinLow` (arc transfer), so the relays closed as soon as the arc struck — before the THC's own safety gate (`cutMotionGateReady` + stabilization) had opened. During the entire pierce and the descent to cut height, neither the CNC nor the THC could properly manage Z height. The relays now follow `thcActive`, so Z ownership is only transferred once the full gating chain has cleared. The relay drive was also moved out of `updatePlasmaState()` into a new `updateRelays()` called after `updateTHCState()`, so the relay state reflects the current tick's decision instead of the previous one.
- **Anti-dive re-trigger storm during long void crossings.** While an anti-dive lift was active, the slow voltage reference kept integrating the high arc voltage, so the reference rose toward the fast voltage. When the 1000 ms lift timeout expired, the fast voltage was still well above the elevated reference, causing immediate re-trigger and repeated Z lifts. The slow filter is now frozen during anti-dive, and on release it is re-seeded to the current arc voltage with the convergence gate reset, giving a 500 ms cooldown before anti-dive can re-arm.
- **False first anti-dive trigger at cut start.** The slow filter is re-seeded when the plasma stabilizes, but the first second or two of cutting still contains pierce/kerf transients and the reference has not fully converged to the steady cut voltage. Anti-dive is now disabled for 2 s after THC becomes active, while normal PID height correction remains active.

### Added
- **Anti-dive lift telemetry.** A cumulative lift counter (`cum_lift`, in steps) is now reset at the start of each cut and incremented on every anti-dive trigger. It appears in the `EV: anti-dive TRIGGERED`, `EV: anti-dive RELEASED`, and periodic `ad=ACTIVE` status lines. If `cum_lift` grows well beyond `ANTI_DIVE_LIFT_STEPS` during a single cut, the anti-dive is re-triggering — the counter exists to gather field evidence before deciding on a behavioral fix.
- **Anti-dive RELEASED now reports a reason** (`return` vs `timeout`) so timeout-forced releases (the scenario most likely to expose a runaway) are distinguishable from normal voltage-return releases in the logs.
- Anti-dive TRIGGERED/RELEASED events now include the Z motor position (`pos=`) for tracking cumulative Z drift.
- New constant `ANTI_DIVE_IGNORE_AFTER_START_MS` (default 2000 ms) in `src/Config.h`.

## [2.5.0] - 2026-06-26

### Fixed
- **Premature anti-dive triggers at cut start with high-voltage plasma cutters** (e.g. Hypertherm 45A Sync). The fixed 5 V drop threshold and 30 ms confirmation were too sensitive for the higher arc voltages and energetic pierce transients of 45 A-class machines. Anti-dive now uses a relative threshold and waits for the slow filter to converge before arming.

### Changed
- **Anti-dive activation threshold is now relative to the voltage setpoint**: `max(5 V, 5 % of setpoint)`. At the user's 138 V setpoint the effective threshold is 6.9 V instead of 5 V.
- **Anti-dive confirmation time raised from 30 ms to 100 ms** to ignore short pierce/kerf spikes.

### Added
- **Slow-filter convergence gate.** After the plasma stabilizes, anti-dive is blocked until 50 new slow-filter samples (500 ms) have been integrated. This prevents the reference from deciding lifts while it is still settling from pierce voltage to cut voltage.
- **New serial diagnostic state `WAIT_SLOW_FILTER`** in the THC activation chain, reported by `STATUS` and `DEBUG` logs.

## [2.4.0] - 2026-06-21

### Changed
- **Dual-rate voltage sampling path.** `THCController` now runs two independent sampling chains instead of one shared reading:
  - **PID input:** sampled and filtered at the existing 1 kHz rate, using a 15-sample rolling average with a trimmed mean (rejecting the 2 highest / lowest outliers) and an EMA (`INPUT_ALPHA=0.95`). This gives the PID a much cleaner voltage estimate without the lag that a single slow filter would impose.
  - **Anti-dive reference:** sampled at 100 Hz (`SLOW_SAMPLE_INTERVAL_MS=10`) through its own 10-sample rolling average, then into the existing 200-sample slow EMA. Keeping the slow path at a lower rate prevents ADC noise from being integrated at full loop speed while still tracking real arc drift.
- **Slow-filter re-seeding on plasma stabilization.** The slow reference is now re-seeded when the plasma input stabilizes, eliminating the ~2 s convergence period after pierce that previously left anti-dive with a stale or polluted reference.

### Added
- **30 ms temporal confirmation before anti-dive triggers** (`ANTI_DIVE_CONFIRM_MS=30`). A voltage drop must persist for three consecutive slow samples before the emergency Z lift fires, cutting false lifts from single noisy ADC readings.

### Fixed
- **LCD speed display buffer overflow and clamping.** The 4-digit speed fields used a 5-byte `char` buffer, which was one byte short for the terminating null. Top-row speed is now clamped to `9999` and negative values are pinned to `0`; the bottom-row cut-speed field gets the same clamping. Buffers widened to 8 bytes to leave headroom.

### Notes
- Documentation refreshed: `CLAUDE.md` has been merged into `AGENTS.md`, which is now the single agent-facing guide. `README.md` was updated to cover the v2.2.2 → v2.3.2 changes, the `Z_DIR_INVERT` flag, and the explicit anti-dive lift behavior.

## [2.3.2] - 2026-05-26

### Added
- **`Z_DIR_INVERT` build flag for Z direction polarity.** If your driver wiring makes the torch move down when the firmware commands up, set `-D Z_DIR_INVERT=1` in `platformio.ini` instead of rewiring. The flag is passed to `AccelStepper::setPinsInverted()` and flips the DIR pin output so the firmware's "positive steps = up" contract stays correct for both PID corrections and anti-dive lift.

## [2.3.1] - 2026-05-26

### Fixed
- **Anti-dive now performs an explicit Z lift.** The previous implementation relied on an accidental side-effect: the stepper target stayed at `0` and `runMotor()` happened to pull the torch toward its startup position. That was implicit, unconfigurable, and fragile. The new behavior commands `moveTo(currentPosition + ANTI_DIVE_LIFT_STEPS)` using an aggressive envelope (`ANTI_DIVE_LIFT_SPEED=5000`, `ANTI_DIVE_LIFT_ACCEL=20000`) and restores normal PID motion when the condition clears.

### Changed
- **Anti-dive lift height is now a build flag:** `-D ANTI_DIVE_LIFT_MM=3.0` (default 3 mm). Speed and acceleration remain fixed in `Config.h` because they are safety-critical.
- **`runMotor()` gates `stepper.run()` while anti-dive is active**, removing the latent conflict where both `run()` and the PID's `runSpeed()` were trying to drive the same motor.

## [2.3.0] - 2026-05-26

### Changed
- **Dead-code cleanup across `src/`.** Removed ~90 lines of unused constants, methods, and includes surfaced by a full source audit (`ANTI_DIVE_DURATION_MIN/MAX`, `ENABLE_Z_PIN`, `readRawVoltage()`, `resetSlowFilter()`, unused `DisplayManager` helpers, etc.). No functional changes.
- Renamed `performAntiDiveLift()` → `holdDuringAntiDive()` to reflect what the function actually did before the v2.3.1 lift implementation.

## [2.2.4] - 2026-05-26

### Added
- **`STEPPER_MAX_SPEED`, `STEPPER_ACCELERATION`, and `MAX_CUT_SPEED` are now exposed as build flags**, matching how `STEPS_PER_MM_*` and `DEFAULT_VOLTAGEDIVIDER` were already handled.

### Notes
- These three values are **tightly coupled to the PID gains** and define the THC's correction authority. The overrides in `platformio.ini` are commented out by default and wrapped in a hard warning; changing them without re-tuning PID can saturate the loop and cause torch dive or chatter.

## [2.2.3] - 2026-05-20

### Fixed
- **Encoder could not tune `Ki`.** `KI_MAX` was `1.0` while the default `Ki` was `7.5`, so any attempt to adjust the integral term through the menu was silently rejected. `KI_MAX` is now `50`, consistent with `KP_MAX=1500` and `KD_MAX=100`.
- **Removed unused `ENABLE_Z_PIN` define.** It duplicated `ENABLE_PIN` on pin 10 and was never referenced; removing it eliminates the apparent pin collision.

## [2.2.2] - 2026-05-10

### Changed
- **Serial commands are now case-insensitive.** Typing `help`, `status`, or `debug` is accepted instead of requiring uppercase input.
- **Unknown commands now echo back a hint** instead of being silently dropped, making it obvious when the serial link is alive.
- Empty lines are ignored so a bare `Enter` can be used to probe the link without noise.

## [2.2.1] - 2026-04-24

### Fixed
- **DFU re-flash reliability from VSCode / PlatformIO.** When the running firmware crashes in a tight loop, the watchdog keeps resetting the board and the host never gets a stable window to complete the 1200-baud touch + `dfu-util` transfer, so uploads fail intermittently. `setup()` now holds for 3 seconds **before** arming the watchdog, giving the host a guaranteed window to catch the board. The watchdog timeout is also bumped to 5500 ms (near the library max of 5592 ms), which more than covers any realistic loop stall without meaningfully weakening the plasma-EMI protection.

### Notes
- Cost: every cold boot now takes 3 s of unprotected time before the WDT engages. Acceptable for a shop-floor controller but worth knowing if you rely on sub-second boot to ready state. Adjust `wdtBootDelayStart` in `src/main.cpp` if you need a different trade-off.
- The claim sometimes made that `NVIC_SystemReset()` "doesn't reset the WDT registers" on the RA4M1 does not apply to the software-started WDT that this library uses — it applies to the IWDT, which SmartTHC does not use. The real value of the boot window is giving the host time to catch a crashing firmware, not working around a silicon quirk.

## [2.2.0] - 2026-04-24

### Fixed
- **Watchdog reboot loop on boot.** `WDT.begin()` on the Uno R4 Minima expects a timeout in **milliseconds**, not the Renesas FSP `WDT_TIMEOUT_*` enum values. The previous code passed `WDT_TIMEOUT_128` (enum value `0`), which the library interpreted as a ~1 ms watchdog, resetting the MCU before the first loop iteration. Symptom: blank LCD with backlight pulsing in lockstep with the output switch relay clicks. Fixed by passing a numeric 2000 ms timeout.
- **Slow voltage filter pollution during plasma OFF.** The 200-sample slow filter (used as the anti-dive reference) was advancing unconditionally, integrating ADC noise while the arc was off. The gate was then tightened to `arcDetected`, but `ARC_THRESHOLD` (10 V) is too low to distinguish the real arc from the inductive tail of the torch: after `PLASMA_PIN` goes HIGH, `fastVoltage` stays above 10 V for several hundred ms, and garbage readings (observed: 70–207 V during extinction) kept leaking into the buffer. Field logs showed the slow reference frozen at 31.8 V instead of the last real arc value (~122 V), breaking anti-dive on the next cut. Slow filter is now gated by `plasmaPinLow && arcDetected` and re-seeded on every genuine off→on transition.

### Changed
- **Serial logging overhaul.** Output is now grep/awk friendly and far denser in information:
  - One compact `key=value` status line per `LOG_INTERVAL`, e.g.
    `t=162412 st=THC_ACTIVE v=121.2/120.9 tgt=122.0 out=+0 spd=590 z=+1 ad=off`
    — now includes setpoint, PID output, Z step position, and a state label mirroring the THC activation chain (`PLASMA_OFF`, `WAIT_STAB`, `THC_SIG_OFF`, `ENABLE_OFF`, `ARC_LOST`, `WAIT_RESTAB`, `WAIT_MOTION`, `WAIT_SPEED`, `ANTI_DIVE`, `THC_ACTIVE`, `ARMED`).
  - Edge-triggered `EV:` lines on every real state transition: plasma on/off, arc detected/lost, plasma stabilized, THC_SIG asserted/released, motion gate ready/lost, THC re-stab delay armed/done, THC active (with reason when inactive), and anti-dive triggered/released (with voltage drop and duration).
  - `STATUS` command response is also `key=value` formatted for consistency.
- **`platformio.ini` serial monitor defaults** — `monitor_speed=115200`, `monitor_filters=send_on_enter`, `monitor_eol=LF`, `monitor_echo=yes`. `pio device monitor` now opens at the right baud and lets you type commands with local echo.

### Added
- `THCController::isThcOnReStabilized()` and `THCController::isCutMotionGateReady()` getters, so the serial log can name the exact gate blocking THC activation instead of reporting `Unknown condition`.

## [2.1.0] - 2026-04-10

### Added
- Motion-gated THC activation flow to prevent premature height control during pierce phase.
- New timing and motion parameters in `src/Config.h`:
  - `CUT_MOTION_CONFIRM_DELAY`
  - `THC_AFTER_CUT_START_DELAY`
  - `CUT_SPEED_HYSTERESIS_RATIO`
- New `SpeedMonitor` motion-state API:
  - `isCutMotionDetected()`
  - `hasCutMotionStableSince(...)`
  - `getCutMotionStartTime()`
- `THCController::setSpeedMonitor(...)` to inject movement state into THC gating logic.

### Changed
- THC activation now requires confirmed XY cutting motion and a fixed post-motion delay.
- Speed state detection now uses hysteresis to reduce threshold chatter.
- README and internal project guidance updated to describe the v2.1 activation model.

### Notes
- Existing plasma stabilization and THC_OFF re-stabilization protections remain active and are now combined with motion gating for improved pierce robustness.

## [2.0.0] - Previous release

- Major modular refactor and stability improvements.

## [1.0.0] - Initial release

- Initial public release of SmartTHC.
