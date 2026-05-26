/**
 * SmartTHC - Configuration
 *
 * Centralizes all constants, pins and default values
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// PINS - Arduino Uno R4 Minima
// ============================================================================

// Plasma & THC I/O
#define PLASMA_PIN      12  // Input: plasma arc OK signal
#define ENABLE_PIN      10  // Input: THC enable (pull-up)
#define THC_OFF_PIN     11  // Input: THC off signal
#define PLASMA_VOLTAGE  A0  // ADC: plasma voltage

// Z motor control
#define DIR_PIN         8   // Direction
#define STEP_PIN        6   // Step
#define SWITCH1         9   // Output switch 1
#define SWITCH2         13  // Output switch 2

// Rotary encoder KY-040
#define ENCODER_PIN_A   4   // CLK
#define ENCODER_PIN_B   5   // DT
#define BUTTON_PIN      7   // SW (button)

// X/Y speed interrupts
#define STEP_X_PIN      2   // X interrupt
#define STEP_Y_PIN      3   // Y interrupt

// ============================================================================
// MECHANICAL PARAMETERS
// ============================================================================

#ifndef STEPS_PER_MM_X
#define STEPS_PER_MM_X  200.0
#endif

#ifndef STEPS_PER_MM_Y
#define STEPS_PER_MM_Y  200.0
#endif

#ifndef STEPS_PER_MM_Z
#define STEPS_PER_MM_Z  50.0
#endif

#ifndef ANTI_DIVE_LIFT_MM
#define ANTI_DIVE_LIFT_MM  3.0
#endif

// Z direction polarity. 0 = positive PID output / positive lift offset move
// the torch UP (default wiring). 1 = same logical semantics but the DIR pin
// output is inverted at the driver — set this if your Z stepper turns the
// wrong way on bench test. Affects PID and anti-dive lift simultaneously.
#ifndef Z_DIR_INVERT
#define Z_DIR_INVERT 0
#endif

#ifndef DEFAULT_VOLTAGEDIVIDER
#define DEFAULT_VOLTAGEDIVIDER  83.27
#endif

#define MM_PER_INCH     25.4

// ============================================================================
// UNITS (IMPERIAL vs METRIC)
// ============================================================================

#if USE_IMPERIAL
    const float DEFAULT_CUT_SPEED = 51.0f;      // ~1300 mm/min in IPM
    #ifndef MAX_CUT_SPEED
    #define MAX_CUT_SPEED 400.0f
    #endif
    const float CUT_SPEED_ADJUST_STEP = 5.0f;
    const float DIST_PER_STEP_X = (1.0f / STEPS_PER_MM_X) / MM_PER_INCH;
    const float DIST_PER_STEP_Y = (1.0f / STEPS_PER_MM_Y) / MM_PER_INCH;
    const char* const SPEED_UNIT = "IPM";
    const char* const SPEED_UNIT_LONG = "Inches/min";
#else
    const float DEFAULT_CUT_SPEED = 1300.0f;
    #ifndef MAX_CUT_SPEED
    #define MAX_CUT_SPEED 10000.0f
    #endif
    const float CUT_SPEED_ADJUST_STEP = 100.0f;
    const float DIST_PER_STEP_X = 1.0f / STEPS_PER_MM_X;
    const float DIST_PER_STEP_Y = 1.0f / STEPS_PER_MM_Y;
    const char* const SPEED_UNIT = "mm/min";
    const char* const SPEED_UNIT_LONG = "mm/min";
#endif

// Z axis always in mm
const float DIST_PER_STEP_Z = 1.0f / STEPS_PER_MM_Z;
const long ANTI_DIVE_LIFT_STEPS = (long)(ANTI_DIVE_LIFT_MM * STEPS_PER_MM_Z);

// ============================================================================
// PID DEFAULT VALUES
// ============================================================================

const float DEFAULT_SETPOINT = 110.0f;
const float DEFAULT_CORRECTION_FACTOR = 1.0f;
const float DEFAULT_THRESHOLD_RATIO = 0.8f;
const float DEFAULT_KP = 30.0f;
const float DEFAULT_KI = 7.5f;
const float DEFAULT_KD = 2.0f;

// ============================================================================
// EEPROM ADDRESSES
// ============================================================================

#define EEPROM_SETPOINT_ADDR            0
#define EEPROM_CORRECTION_FACTOR_ADDR   4
#define EEPROM_CUT_SPEED_ADDR           8
#define EEPROM_THRESHOLD_RATIO_ADDR     12
#define EEPROM_KP_ADDR                  16
#define EEPROM_KI_ADDR                  20
#define EEPROM_KD_ADDR                  24
#define EEPROM_INITIALIZED_FLAG         28

const byte EEPROM_INIT_FLAG_VALUE = 0xAA;

// ============================================================================
// TIMINGS & INTERVALS
// ============================================================================

const unsigned long DISPLAY_INTERVAL = 250;         // ms - LCD refresh
const unsigned long SPEED_INTERVAL = 50;            // ms - speed calculation
const unsigned long LOG_INTERVAL = 150;             // ms - serial logging
const unsigned long LOOP_LOG_INTERVAL = 10000;      // ms - loop stats
const unsigned long EEPROM_WRITE_INTERVAL = 1000;   // ms - EEPROM save
const unsigned long STABILIZATION_DELAY = 750;      // ms - plasma stabilization delay
const unsigned long THC_ON_RESTAB_DELAY = 300;      // ms - re-stabilization delay after THC_OFF -> THC_ON
const unsigned long STEP_TIMEOUT = 500;             // ms - step timeout
const unsigned long DEBOUNCE_US = 3;                // us - interrupt debounce

const unsigned long CUT_MOTION_CONFIRM_DELAY = 200;
const unsigned long THC_AFTER_CUT_START_DELAY = 500;
const float CUT_SPEED_HYSTERESIS_RATIO = 0.1f;

// PID timing
const unsigned long PID_INTERVAL_US = 1000;         // us - 1kHz

// ============================================================================
// ANTI-DIVE PARAMETERS
// ============================================================================

const unsigned long MAX_ANTI_DIVE_DURATION = 1000;  // ms - max total duration
const float DROP_THRESHOLD = 5.0f;                  // V - activation threshold
const float RETURN_THRESHOLD = 3.0f;                // V - deactivation threshold

// Emergency Z retract envelope used during anti-dive. Deliberately faster
// than the PID envelope (STEPPER_MAX_SPEED / STEPPER_ACCELERATION) so the
// lift completes in tens of milliseconds — the whole point of anti-dive
// is to outrun the dive itself. The lift HEIGHT is exposed as a build
// flag (ANTI_DIVE_LIFT_MM); speed/accel stay fixed because they are
// safety-critical and shouldn't drift with user tuning.
const float ANTI_DIVE_LIFT_SPEED = 5000.0f;         // steps/s - emergency retract speed
const float ANTI_DIVE_LIFT_ACCEL = 20000.0f;        // steps/s² - aggressive ramp for snappy retract

// Position history for anti-dive
const int POSITION_HISTORY_INTERVAL = 100;          // ms
const int POSITION_HISTORY_SIZE = 20;               // ~2 seconds

// ============================================================================
// ANTI-DIVE DISPLAY PARAMETERS
// ============================================================================

const unsigned long ANTI_DIVE_DISPLAY_DURATION = 5000;  // ms - message duration
const unsigned long ANTI_DIVE_BLINK_INTERVAL = 250;     // ms - blink interval

// ============================================================================
// THRESHOLDS & LIMITS
// ============================================================================

const float ARC_THRESHOLD = 10.0f;                  // V - arc detection
const float SPEED_CORRECTION = 1.0f;
const unsigned long MIN_STEPS = 20;

// PID limits
const double PID_OUTPUT_MIN = -2500.0;
const double PID_OUTPUT_MAX = 2500.0;
const double PID_WINDUP_MIN = -100.0;
const double PID_WINDUP_MAX = 100.0;

// Parameter limits
const float SETPOINT_MIN = 50.0f;
const float SETPOINT_MAX = 200.0f;
const float CORRECTION_FACTOR_MIN = 0.5f;
const float CORRECTION_FACTOR_MAX = 2.0f;
const float THRESHOLD_RATIO_MIN = 0.0f;
const float THRESHOLD_RATIO_MAX = 1.0f;
const double KP_MAX = 1500.0;
const double KI_MAX = 50.0;
const double KD_MAX = 100.0;

// ============================================================================
// MOTOR PARAMETERS
//
// !!! WARNING !!! READ BEFORE OVERRIDING STEPPER_MAX_SPEED / STEPPER_ACCELERATION
//
// These are NOT generic stepper-tuning numbers. They define the THC's correction
// authority — how fast the Z axis is allowed to chase a plasma voltage transient
// (kerf crossings, edge effects, plate warp). They are intentionally HIGHER than
// the values you would use for a basic Z move (e.g. FluidNC homing/jogging),
// because the THC must out-run normal motion to keep the torch on target.
//
// PID gains (KP / KI / KD) are TIGHTLY COUPLED to these values. If you change
// STEPPER_MAX_SPEED or STEPPER_ACCELERATION, you MUST re-tune the PID from
// scratch. Lowering them caps PID output → the loop saturates → response
// degrades, oscillation, or torch dive. Raising them without retuning Kp can
// produce overshoot and chatter.
//
// Defaults (5000 / 5000) target: NEMA17 1.8°/step, ~400 pulses/rev (1/8 µstep),
// 8 mm-pitch leadscrew → 50 steps/mm Z. If your Z drive matches that envelope,
// LEAVE THESE ALONE. If you override via build flags, plan a full PID re-tune
// session immediately afterwards — there is no shortcut.
//
// MAX_CUT_SPEED is unit-aware: if you override it, set a value consistent with
// USE_IMPERIAL (IPM vs mm/min). Default is 10000 mm/min or 400 IPM.
// ============================================================================

#ifndef STEPPER_MAX_SPEED
#define STEPPER_MAX_SPEED 5000.0f
#endif

#ifndef STEPPER_ACCELERATION
#define STEPPER_ACCELERATION 5000.0f
#endif

const float STEPPER_DEADZONE = 1.0f;                // V - PID dead zone

// ============================================================================
// FILTERING
// ============================================================================

const int SPEED_FILTER_SIZE = 1;
#define OVERSAMPLE_TARGET 10
const float INPUT_ALPHA = 0.7f;                     // Fast low-pass
const float ALPHA_SLOW = 0.8f;                      // Slow low-pass
const int N_SLOW = 200;                             // Slow average

// ============================================================================
// LCD
// ============================================================================

#define LCD_I2C_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// Custom characters
#define CHAR_ENABLE       0
#define CHAR_PLASMA       1
#define CHAR_THC_ACTIVE   2
#define CHAR_ARROW_UP     3
#define CHAR_ARROW_DOWN   4
#define CHAR_STABLE       5
#define CHAR_ENABLE_ALL   6

// ============================================================================
// NUMBER OF SCREENS
// ============================================================================

const int NB_SCREENS = 8;

#endif // CONFIG_H
