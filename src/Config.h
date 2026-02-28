/**
 * SmartTHC - Configuration
 * 
 * Centralise toutes les constantes, pins et valeurs par défaut
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// PINS - Arduino Uno R4 Minima
// ============================================================================

// Entrées/Sorties Plasma & THC
#define PLASMA_PIN      12  // Input: plasma arc OK signal
#define ENABLE_PIN      10  // Input: THC enable (pull-up)
#define THC_OFF_PIN     11  // Input: THC off signal
#define PLASMA_VOLTAGE  A0  // ADC: plasma voltage

// Contrôle moteur Z
#define ENABLE_Z_PIN    10  // Enable stepper Z
#define DIR_PIN         8   // Direction
#define STEP_PIN        6   // Step
#define SWITCH1         9   // Sortie switch 1
#define SWITCH2         13  // Sortie switch 2

// Encodeur rotatif KY-040
#define ENCODER_PIN_A   4   // CLK
#define ENCODER_PIN_B   5   // DT
#define BUTTON_PIN      7   // SW (bouton)

// Interruptions vitesse X/Y
#define STEP_X_PIN      2   // Interruption X
#define STEP_Y_PIN      3   // Interruption Y

// ============================================================================
// PARAMÈTRES MÉCANIQUES
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

#ifndef ANTI_DIVE_BONUS_MM
#define ANTI_DIVE_BONUS_MM  1.5
#endif

#ifndef DEFAULT_VOLTAGEDIVIDER
#define DEFAULT_VOLTAGEDIVIDER  83.27
#endif

#define MM_PER_INCH     25.4

// ============================================================================
// UNITÉS (IMPÉRIAL vs MÉTRIQUE)
// ============================================================================

#if USE_IMPERIAL
    const float DEFAULT_CUT_SPEED = 51.0f;      // ~1300 mm/min en IPM
    const float MAX_CUT_SPEED = 400.0f;
    const float CUT_SPEED_ADJUST_STEP = 5.0f;
    const float DIST_PER_STEP_X = (1.0f / STEPS_PER_MM_X) / MM_PER_INCH;
    const float DIST_PER_STEP_Y = (1.0f / STEPS_PER_MM_Y) / MM_PER_INCH;
    const char* const SPEED_UNIT = "IPM";
    const char* const SPEED_UNIT_LONG = "Inches/min";
#else
    const float DEFAULT_CUT_SPEED = 1300.0f;
    const float MAX_CUT_SPEED = 10000.0f;
    const float CUT_SPEED_ADJUST_STEP = 100.0f;
    const float DIST_PER_STEP_X = 1.0f / STEPS_PER_MM_X;
    const float DIST_PER_STEP_Y = 1.0f / STEPS_PER_MM_Y;
    const char* const SPEED_UNIT = "mm/min";
    const char* const SPEED_UNIT_LONG = "mm/min";
#endif

// Z axis toujours en mm
const float DIST_PER_STEP_Z = 1.0f / STEPS_PER_MM_Z;
const long ANTI_DIVE_BONUS_STEPS = (long)(ANTI_DIVE_BONUS_MM * STEPS_PER_MM_Z);

// ============================================================================
// VALEURS PAR DÉFAUT PID
// ============================================================================

const float DEFAULT_SETPOINT = 110.0f;
const float DEFAULT_CORRECTION_FACTOR = 1.0f;
const float DEFAULT_THRESHOLD_RATIO = 0.8f;
const float DEFAULT_KP = 30.0f;
const float DEFAULT_KI = 7.5f;
const float DEFAULT_KD = 2.0f;

// ============================================================================
// ADRESSES EEPROM
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
// TIMINGS & INTERVALLES
// ============================================================================

const unsigned long DISPLAY_INTERVAL = 250;         // ms - rafraîchissement LCD
const unsigned long SPEED_INTERVAL = 50;            // ms - calcul vitesse
const unsigned long LOG_INTERVAL = 150;             // ms - logs série
const unsigned long LOOP_LOG_INTERVAL = 10000;      // ms - stats boucle
const unsigned long EEPROM_WRITE_INTERVAL = 1000;   // ms - sauvegarde EEPROM
const unsigned long STABILIZATION_DELAY = 500;      // ms - délai stabilisation plasma
const unsigned long STEP_TIMEOUT = 500;             // ms - timeout pas
const unsigned long DEBOUNCE_US = 3;                // µs - anti-rebond interruptions

// PID timing
const unsigned long PID_INTERVAL_US = 1000;         // µs - 1kHz

// ============================================================================
// PARAMÈTRES ANTI-DIVE
// ============================================================================

const unsigned long ANTI_DIVE_DURATION_MIN = 50;    // ms - durée min
const unsigned long ANTI_DIVE_DURATION_MAX = 300;   // ms - durée max
const unsigned long MAX_ANTI_DIVE_DURATION = 1000;  // ms - durée max totale
const float DROP_THRESHOLD = 5.0f;                  // V - seuil activation
const float RETURN_THRESHOLD = 3.0f;                // V - seuil désactivation
const float ANTI_DIVE_LIFT_SPEED = 1000.0f;         // steps/s
const float ANTI_DIVE_LIFT_ACCEL = 5000.0f;         // steps/s²

// Historique position pour anti-dive
const int POSITION_HISTORY_INTERVAL = 100;          // ms
const int POSITION_HISTORY_SIZE = 20;               // ~2 secondes

// ============================================================================
// PARAMÈTRES AFFICHAGE ANTI-DIVE
// ============================================================================

const unsigned long ANTI_DIVE_DISPLAY_DURATION = 5000;  // ms - durée message
const unsigned long ANTI_DIVE_BLINK_INTERVAL = 250;     // ms - clignotement

// ============================================================================
// SEUILS & LIMITES
// ============================================================================

const float ARC_THRESHOLD = 10.0f;                  // V - détection arc
const float SPEED_CORRECTION = 1.0f;
const unsigned long MIN_STEPS = 20;

// Limites PID
const double PID_OUTPUT_MIN = -2500.0;
const double PID_OUTPUT_MAX = 2500.0;
const double PID_WINDUP_MIN = -100.0;
const double PID_WINDUP_MAX = 100.0;

// Limites paramètres
const float SETPOINT_MIN = 50.0f;
const float SETPOINT_MAX = 200.0f;
const float CORRECTION_FACTOR_MIN = 0.5f;
const float CORRECTION_FACTOR_MAX = 2.0f;
const float THRESHOLD_RATIO_MIN = 0.0f;
const float THRESHOLD_RATIO_MAX = 1.0f;
const double KP_MAX = 1500.0;
const double KI_MAX = 1.0;
const double KD_MAX = 100.0;

// ============================================================================
// PARAMÈTRES MOTEUR
// ============================================================================

const float STEPPER_MAX_SPEED = 5000.0f;
const float STEPPER_ACCELERATION = 5000.0f;
const float STEPPER_DEADZONE = 1.0f;                // V - zone morte PID

// ============================================================================
// FILTRAGE
// ============================================================================

const int SPEED_FILTER_SIZE = 1;
#define OVERSAMPLE_TARGET 10
const float INPUT_ALPHA = 0.7f;                     // Low-pass rapide
const float ALPHA_SLOW = 0.8f;                      // Low-pass lent
const int N_SLOW = 200;                             // Moyenne lente

// ============================================================================
// LCD
// ============================================================================

#define LCD_I2C_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// Caractères personnalisés
#define CHAR_ENABLE       0
#define CHAR_PLASMA       1
#define CHAR_THC_ACTIVE   2
#define CHAR_ARROW_UP     3
#define CHAR_ARROW_DOWN   4
#define CHAR_STABLE       5
#define CHAR_ENABLE_ALL   6

// ============================================================================
// NOMBRE D'ÉCRANS
// ============================================================================

const int NB_SCREENS = 8;

#endif // CONFIG_H
