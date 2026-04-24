/**
 * SmartTHC - Main
 *
 * Modular THC system orchestration
 * Arduino Uno R4 Minima
 */

#include <Arduino.h>
#include <WDT.h>
#include "Config.h"
#include "EncoderManager.h"
#include "SpeedMonitor.h"
#include "EEPROMManager.h"
#include "THCController.h"
#include "DisplayManager.h"
#include "SerialCommand.h"

// ============================================================================
// MODULE INSTANCES
// ============================================================================

EncoderManager encoder;
SpeedMonitor speedMonitor;
EEPROMManager eeprom;
THCController thc;
DisplayManager display;
SerialCommand serialCmd;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

int currentScreen = 0;
int prevScreen = -1;
float tempCorrectionFactor = DEFAULT_CORRECTION_FACTOR;

// Timing
unsigned long lastDisplayTime = 0;
unsigned long lastPidTime = 0;

// Loop stats
unsigned long loopStartTime;
unsigned long loopExecutionTimeSum = 0;
unsigned int loopCount = 0;

// ============================================================================
// PROTOTYPES
// ============================================================================

void handleScreenTransitions();
void handleEncoderInput();
void handleParameterAdjustment(int delta);
void notifyAntiDiveIfNeeded();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Serial init
    serialCmd.begin(115200);

    Serial.println("SmartTHC - Initializing...");

    // EEPROM init
    eeprom.begin();

    // Load parameters
    THCParameters params;
    eeprom.loadParameters(params);

    // Apply parameters
    thc.setSetpoint(params.setpoint);
    thc.setCorrectionFactor(params.correctionFactor);
    tempCorrectionFactor = params.correctionFactor;

    speedMonitor.setCutSpeed(params.cutSpeed);
    speedMonitor.setThresholdRatio(params.thresholdRatio);

    thc.setKp(params.kp);
    thc.setKi(params.ki);
    thc.setKd(params.kd);

    // Module init
    encoder.begin();
    speedMonitor.begin();
    thc.begin();
    thc.setSpeedMonitor(&speedMonitor);
    display.begin();

    // Watchdog Timer - auto reboot if MCU hangs (plasma EMI)
    // IWDT clocks at ~15 kHz: 8192 cycles ≈ 550 ms, safely above the 250 ms
    // LCD refresh and any full-screen redraw burst over I2C.
    WDT.begin(WDT_TIMEOUT_8192);

    Serial.println("SmartTHC - Ready!");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    WDT.refresh();
    loopStartTime = micros();
    unsigned long currentTime = millis();

    // ------------------------------------------------------------------------
    // 1. Encoder update (rotation + button)
    // ------------------------------------------------------------------------
    encoder.update();

    // ------------------------------------------------------------------------
    // 2. Button handling (screen change)
    // ------------------------------------------------------------------------
    if (encoder.isButtonClicked()) {
        prevScreen = currentScreen;
        currentScreen = (currentScreen + 1) % NB_SCREENS;

        // Enter/exit temporary correction screen
        if (currentScreen == 2 && prevScreen != 2) {
            tempCorrectionFactor = thc.getCorrectionFactor();
            Serial.println("Entering temporary voltage_correction_factor adjustment mode");
        }
        if (prevScreen == 2 && currentScreen != 2) {
            if (abs(tempCorrectionFactor - thc.getCorrectionFactor()) > 0.001f) {
                thc.setCorrectionFactor(tempCorrectionFactor);
                eeprom.scheduleSaveCorrectionFactor(tempCorrectionFactor);
                Serial.println("Exiting adjustment mode: voltage_correction_factor saved");
            }
        }
    }

    // ------------------------------------------------------------------------
    // 3. Rotary encoder handling (parameter adjustment)
    // ------------------------------------------------------------------------
    int encoderDelta = encoder.getDelta();
    if (encoderDelta != 0) {
        handleParameterAdjustment(encoderDelta);
    }

    // ------------------------------------------------------------------------
    // 4. Speed monitor update
    // ------------------------------------------------------------------------
    speedMonitor.update(currentTime);

    // Record position for anti-dive
    speedMonitor.recordPosition(currentTime, thc.getMotorPosition());

    // ------------------------------------------------------------------------
    // 5. THC controller update (PID, anti-dive, motor)
    // ------------------------------------------------------------------------
    // Run at 1kHz
    unsigned long currentMicros = micros();
    if (currentMicros - lastPidTime >= PID_INTERVAL_US) {
        thc.update(currentTime);
        lastPidTime = currentMicros;

        // Notify display if anti-dive just activated
        notifyAntiDiveIfNeeded();
    }

    // Run motor as often as possible
    thc.runMotor();

    // ------------------------------------------------------------------------
    // 6. LCD display update
    // ------------------------------------------------------------------------
    if (currentTime - lastDisplayTime >= DISPLAY_INTERVAL) {
        display.update(currentTime, currentScreen, &thc, &speedMonitor,
                       tempCorrectionFactor, encoderDelta);
        lastDisplayTime = currentTime;
    }

    // ------------------------------------------------------------------------
    // 7. Serial command processing and logging
    // ------------------------------------------------------------------------
    serialCmd.update(&eeprom, &thc, &speedMonitor);
    serialCmd.logStatus(currentTime, &thc, &speedMonitor);

    // ------------------------------------------------------------------------
    // 8. Deferred EEPROM saves
    // ------------------------------------------------------------------------
    eeprom.update();

    // ------------------------------------------------------------------------
    // 9. Execution stats
    // ------------------------------------------------------------------------
    unsigned long loopEndTime = micros();
    unsigned long loopExecutionTime = loopEndTime - loopStartTime;
    loopExecutionTimeSum += loopExecutionTime;
    loopCount++;

    // Log stats every 10 seconds
    if (currentTime % LOOP_LOG_INTERVAL < DISPLAY_INTERVAL && loopCount > 0) {
        unsigned long avgLoopTime = loopExecutionTimeSum / loopCount;
        float loopFrequency = 1000000.0f / avgLoopTime;
        serialCmd.logLoopStats(currentTime, avgLoopTime, loopFrequency);
        loopExecutionTimeSum = 0;
        loopCount = 0;
    }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void handleParameterAdjustment(int delta) {
    switch (currentScreen) {
        case 1: { // Setpoint
            float newSetpoint = thc.getSetpoint() + delta;
            if (newSetpoint < SETPOINT_MIN) newSetpoint = SETPOINT_MIN;
            if (newSetpoint > SETPOINT_MAX) newSetpoint = SETPOINT_MAX;
            thc.setSetpoint(newSetpoint);
            eeprom.scheduleSaveSetpoint(newSetpoint);
            break;
        }
        case 2: { // Correction factor (temporary)
            tempCorrectionFactor += delta * 0.01f;
            if (tempCorrectionFactor < CORRECTION_FACTOR_MIN)
                tempCorrectionFactor = CORRECTION_FACTOR_MIN;
            if (tempCorrectionFactor > CORRECTION_FACTOR_MAX)
                tempCorrectionFactor = CORRECTION_FACTOR_MAX;
            break;
        }
        case 3: { // Cut speed
            float newSpeed = speedMonitor.getCutSpeed() + delta * CUT_SPEED_ADJUST_STEP;
            speedMonitor.setCutSpeed(newSpeed);
            eeprom.scheduleSaveCutSpeed(speedMonitor.getCutSpeed());
            break;
        }
        case 4: { // Threshold ratio
            float newRatio = speedMonitor.getThresholdRatio() + delta * 0.1f;
            speedMonitor.setThresholdRatio(newRatio);
            eeprom.scheduleSaveThresholdRatio(speedMonitor.getThresholdRatio());
            break;
        }
        case 5: { // Kp
            double newKp = thc.getKp() + delta * 1.0;
            thc.setKp(newKp);
            eeprom.scheduleSaveKp(thc.getKp());
            break;
        }
        case 6: { // Ki
            double newKi = thc.getKi() + delta * 0.5;
            thc.setKi(newKi);
            eeprom.scheduleSaveKi(thc.getKi());
            break;
        }
        case 7: { // Kd
            double newKd = thc.getKd() + delta * 0.5;
            thc.setKd(newKd);
            eeprom.scheduleSaveKd(thc.getKd());
            break;
        }
        default:
            break;
    }
}

void notifyAntiDiveIfNeeded() {
    // Check if anti-dive just activated
    AntiDiveEvent event = thc.getAntiDiveEvent();
    if (event.justActivated) {
        display.notifyAntiDiveActivated();
        thc.clearAntiDiveJustActivated();
    }
}
