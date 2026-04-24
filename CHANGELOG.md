# Changelog

All notable changes to this project are documented in this file.

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
