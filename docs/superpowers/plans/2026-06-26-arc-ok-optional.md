# ARC_OK Optional Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the physical `ARC_OK` pin optional via a PlatformIO build flag; when disabled, derive `plasmaPinLow` from the plasma voltage exceeding a fixed 50 V threshold.

**Architecture:** A compile-time switch (`#if ARC_OK_PIN`) gates the physical pin read in `THCController`. When the flag is 0, the same `plasmaPinLow` boolean is computed from `fastVoltage > ARC_OK_FALLBACK_VOLTAGE`, leaving the rest of the THC gating chain untouched.

**Tech Stack:** PlatformIO, Arduino (Renesas RA), C++17, AccelStepper, ArduPID.

---

## File Structure

| File | Responsibility |
|------|----------------|
| `src/Config.h` | Defines `ARC_OK_PIN` (default 1) and `ARC_OK_FALLBACK_VOLTAGE` (default 50.0 V). |
| `platformio.ini` | Exposes `-D ARC_OK_PIN=1` in `[common].build_flags` so users can override it per environment. |
| `src/THCController.cpp` | Gates `pinMode(PLASMA_PIN, INPUT)` and the `digitalRead(PLASMA_PIN)` call on `ARC_OK_PIN`. |

---

## Task 1: Add constants to `src/Config.h`

**Files:**
- Modify: `src/Config.h`

- [ ] **Step 1: Open `src/Config.h` and locate the PINS section**

  Around line 16-20 you should see:
  ```cpp
  // Plasma & THC I/O
  #define PLASMA_PIN      12  // Input: plasma arc OK signal
  #define ENABLE_PIN      10  // Input: THC enable (pull-up)
  #define THC_OFF_PIN     11  // Input: THC off signal
  #define PLASMA_VOLTAGE  A0  // ADC: plasma voltage
  ```

- [ ] **Step 2: Insert the new defines immediately after `PLASMA_VOLTAGE`**

  ```cpp
  #define PLASMA_VOLTAGE  A0  // ADC: plasma voltage

  // ARC_OK handling: 1 = read PLASMA_PIN, 0 = derive arc OK from plasma voltage
  #ifndef ARC_OK_PIN
  #define ARC_OK_PIN 1
  #endif

  #ifndef ARC_OK_FALLBACK_VOLTAGE
  #define ARC_OK_FALLBACK_VOLTAGE 50.0f
  #endif
  ```

- [ ] **Step 3: Verify no duplicate definitions exist**

  Search the file for `ARC_OK_PIN` and `ARC_OK_FALLBACK_VOLTAGE`. There should be exactly one definition of each.

---

## Task 2: Add the build flag to `platformio.ini`

**Files:**
- Modify: `platformio.ini`

- [ ] **Step 1: Open `platformio.ini` and locate `[common].build_flags`**

  Around line 23-29:
  ```ini
  build_flags =
      -D STEPS_PER_MM_X=200.0  ; Steps per mm for X axis (customizable per CNC)
      -D STEPS_PER_MM_Y=200.0  ; Steps per mm for Y axis (customizable per CNC)
      -D STEPS_PER_MM_Z=50.0  ; Steps per mm for Z axis (used for anti-dive lift calc)
      -D ANTI_DIVE_LIFT_MM=3.0  ; Emergency Z retract height (mm) when anti-dive triggers
      -D Z_DIR_INVERT=0  ; 1 = flip Z direction (set this if torch dives instead of lifts on bench test)
      -D DEFAULT_VOLTAGEDIVIDER=50.0  ; Voltage divider factor (moved from code)
  ```

- [ ] **Step 2: Append the new flag after `DEFAULT_VOLTAGEDIVIDER`**

  ```ini
      -D DEFAULT_VOLTAGEDIVIDER=50.0  ; Voltage divider factor (moved from code)
      -D ARC_OK_PIN=1  ; 1 = use physical ARC_OK pin, 0 = derive arc OK from plasma voltage (>50 V)
  ```

- [ ] **Step 3: Save and close the file**

---

## Task 3: Gate `pinMode(PLASMA_PIN)` in `src/THCController.cpp`

**Files:**
- Modify: `src/THCController.cpp`

- [ ] **Step 1: Open `src/THCController.cpp` and locate `begin()`**

  Around line 85-92:
  ```cpp
  void THCController::begin() {
      // Configure pins
      pinMode(PLASMA_PIN, INPUT);
      pinMode(ENABLE_PIN, INPUT_PULLUP);
      pinMode(THC_OFF_PIN, INPUT);
      pinMode(SWITCH1, OUTPUT);
      pinMode(SWITCH2, OUTPUT);
      pinMode(PLASMA_VOLTAGE, INPUT);
  ```

- [ ] **Step 2: Wrap `PLASMA_PIN` configuration in `#if ARC_OK_PIN`**

  ```cpp
  void THCController::begin() {
      // Configure pins
  #if ARC_OK_PIN
      pinMode(PLASMA_PIN, INPUT);
  #endif
      pinMode(ENABLE_PIN, INPUT_PULLUP);
      pinMode(THC_OFF_PIN, INPUT);
      pinMode(SWITCH1, OUTPUT);
      pinMode(SWITCH2, OUTPUT);
      pinMode(PLASMA_VOLTAGE, INPUT);
  ```

---

## Task 4: Gate `digitalRead(PLASMA_PIN)` in `src/THCController.cpp`

**Files:**
- Modify: `src/THCController.cpp`

- [ ] **Step 1: Locate `updatePlasmaState()`**

  Around line 329-332:
  ```cpp
  void THCController::updatePlasmaState(unsigned long currentTime) {
      // Read inputs
      plasmaPinLow = (digitalRead(PLASMA_PIN) == LOW);
      enablePinLow = (digitalRead(ENABLE_PIN) == LOW);
      thcOff = (digitalRead(THC_OFF_PIN) == HIGH);
  ```

- [ ] **Step 2: Replace the single pin read with a compile-time switch**

  ```cpp
  void THCController::updatePlasmaState(unsigned long currentTime) {
      // Read inputs
  #if ARC_OK_PIN
      plasmaPinLow = (digitalRead(PLASMA_PIN) == LOW);
  #else
      plasmaPinLow = (fastVoltage > ARC_OK_FALLBACK_VOLTAGE);
  #endif
      enablePinLow = (digitalRead(ENABLE_PIN) == LOW);
      thcOff = (digitalRead(THC_OFF_PIN) == HIGH);
  ```

- [ ] **Step 3: Verify the rest of `updatePlasmaState()` is unchanged**

  The switch logic, stabilization logic, and `lastPlasmaStabilized` handling should remain exactly as they were.

---

## Task 5: Build the default environment

- [ ] **Step 1: Run the build**

  ```bash
  pio build -e uno_r4_minima
  ```

- [ ] **Step 2: Expected result**

  Build succeeds with no new warnings. Output ends with:
  ```
  SUCCESS
  ```

---

## Task 6: Build the imperial environment

- [ ] **Step 1: Run the build**

  ```bash
  pio build -e uno_r4_minima_imperial
  ```

- [ ] **Step 2: Expected result**

  Build succeeds with no new warnings.

---

## Task 7: Verify build with `ARC_OK_PIN=0`

- [ ] **Step 1: Add a temporary test environment to `platformio.ini`**

  Append at the end of the file:
  ```ini
  [env:uno_r4_minima_no_arc_ok]
  platform = ${common.platform}
  board = ${common.board}
  framework = ${common.framework}
  lib_deps = ${common.lib_deps}
  build_flags =
      ${common.build_flags}
      -D USE_IMPERIAL=0
      -D ARC_OK_PIN=0
  ```

- [ ] **Step 2: Run the build**

  ```bash
  pio build -e uno_r4_minima_no_arc_ok
  ```

- [ ] **Step 3: Expected result**

  Build succeeds. This proves the physical pin code path is fully compiled out.

- [ ] **Step 4: Remove the temporary environment (optional)**

  If the user does not want a dedicated environment committed, delete the `[env:uno_r4_minima_no_arc_ok]` block from `platformio.ini`. If keeping it, rename it to something user-friendly such as `[env:uno_r4_minima_arc_ok_by_voltage]`.

---

## Task 8: Commit the changes

- [ ] **Step 1: Stage the modified files**

  ```bash
  git add src/Config.h src/THCController.cpp platformio.ini
  ```

- [ ] **Step 2: Commit**

  ```bash
  git commit -m "feat: make ARC_OK pin optional with voltage fallback

  Add ARC_OK_PIN build flag (default 1). When set to 0, the physical
  PLASMA_PIN is not read; plasmaPinLow is derived from fastVoltage
  exceeding ARC_OK_FALLBACK_VOLTAGE (50 V). This lets users run
  without a dedicated ARC_OK signal while keeping all existing gating
  and display logic unchanged."
  ```

- [ ] **Step 3: Verify commit**

  ```bash
  git log --oneline -1
  ```

  Expected output contains the commit subject.

---

## Self-Review

### Spec coverage

| Spec section | Plan task |
|--------------|-----------|
| Add `ARC_OK_PIN` and `ARC_OK_FALLBACK_VOLTAGE` to `Config.h` | Task 1 |
| Add `-D ARC_OK_PIN=1` to `platformio.ini` | Task 2 |
| Gate `pinMode(PLASMA_PIN)` on `ARC_OK_PIN` | Task 3 |
| Gate `digitalRead(PLASMA_PIN)` and use voltage fallback | Task 4 |
| Build verification default + imperial | Tasks 5, 6 |
| Build verification with `ARC_OK_PIN=0` | Task 7 |

### Placeholder scan

No TBD, TODO, or vague steps. Each code change is shown verbatim. Each command is exact.

### Type consistency

- `ARC_OK_PIN` is used as an integer preprocessor token (`#if ARC_OK_PIN`), matching how `USE_IMPERIAL` and `Z_DIR_INVERT` are used elsewhere in the project.
- `ARC_OK_FALLBACK_VOLTAGE` is a `float` and compared against `fastVoltage` (also `float`).
- `plasmaPinLow` remains a `bool` in both branches.

---

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-06-26-arc-ok-optional.md`. Two execution options:**

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

**Which approach?**
