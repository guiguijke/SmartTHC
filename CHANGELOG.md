# Changelog

All notable changes to this project are documented in this file.

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
