/**
 * SmartTHC - EEPROM Manager
 *
 * Parameter save/load implementation
 */

#include "EEPROMManager.h"
#include <limits.h>

EEPROMManager::EEPROMManager()
    : pendingSetpoint(DEFAULT_SETPOINT)
    , pendingCorrectionFactor(DEFAULT_CORRECTION_FACTOR)
    , pendingCutSpeed(DEFAULT_CUT_SPEED)
    , pendingThresholdRatio(DEFAULT_THRESHOLD_RATIO)
    , pendingKp(DEFAULT_KP)
    , pendingKi(DEFAULT_KI)
    , pendingKd(DEFAULT_KD)
    , saveSetpointFlag(false)
    , saveCorrectionFactorFlag(false)
    , saveCutSpeedFlag(false)
    , saveThresholdRatioFlag(false)
    , saveKpFlag(false)
    , saveKiFlag(false)
    , saveKdFlag(false)
    , lastEEPROMWrite(0)
{
}

void EEPROMManager::begin() {
    if (!isEEPROMInitialized()) {
        resetToDefaults();
    }
}

void EEPROMManager::loadParameters(THCParameters& params) {
    params.setpoint = readFloat(EEPROM_SETPOINT_ADDR);
    params.correctionFactor = readFloat(EEPROM_CORRECTION_FACTOR_ADDR);
    params.cutSpeed = readFloat(EEPROM_CUT_SPEED_ADDR);
    params.thresholdRatio = readFloat(EEPROM_THRESHOLD_RATIO_ADDR);
    params.kp = readDouble(EEPROM_KP_ADDR);
    params.ki = readDouble(EEPROM_KI_ADDR);
    params.kd = readDouble(EEPROM_KD_ADDR);

    // Validate and correct if needed
    if (!validateSetpoint(params.setpoint)) params.setpoint = DEFAULT_SETPOINT;
    if (!validateCorrectionFactor(params.correctionFactor)) params.correctionFactor = DEFAULT_CORRECTION_FACTOR;
    if (!validateCutSpeed(params.cutSpeed)) params.cutSpeed = DEFAULT_CUT_SPEED;
    if (!validateThresholdRatio(params.thresholdRatio)) params.thresholdRatio = DEFAULT_THRESHOLD_RATIO;
    if (!validateKp(params.kp)) params.kp = DEFAULT_KP;
    if (!validateKi(params.ki)) params.ki = DEFAULT_KI;
    if (!validateKd(params.kd)) params.kd = DEFAULT_KD;
}

void EEPROMManager::saveParameters(const THCParameters& params) {
    writeFloat(EEPROM_SETPOINT_ADDR, params.setpoint);
    writeFloat(EEPROM_CORRECTION_FACTOR_ADDR, params.correctionFactor);
    writeFloat(EEPROM_CUT_SPEED_ADDR, params.cutSpeed);
    writeFloat(EEPROM_THRESHOLD_RATIO_ADDR, params.thresholdRatio);
    writeDouble(EEPROM_KP_ADDR, params.kp);
    writeDouble(EEPROM_KI_ADDR, params.ki);
    writeDouble(EEPROM_KD_ADDR, params.kd);
    markEEPROMInitialized();
    lastEEPROMWrite = millis();
}

void EEPROMManager::scheduleSaveSetpoint(float value) {
    if (validateSetpoint(value) && abs(value - pendingSetpoint) > 0.01f) {
        pendingSetpoint = value;
        saveSetpointFlag = true;
    }
}

void EEPROMManager::scheduleSaveCorrectionFactor(float value) {
    if (validateCorrectionFactor(value) && abs(value - pendingCorrectionFactor) > 0.001f) {
        pendingCorrectionFactor = value;
        saveCorrectionFactorFlag = true;
    }
}

void EEPROMManager::scheduleSaveCutSpeed(float value) {
    if (validateCutSpeed(value) && abs(value - pendingCutSpeed) > 0.01f) {
        pendingCutSpeed = value;
        saveCutSpeedFlag = true;
    }
}

void EEPROMManager::scheduleSaveThresholdRatio(float value) {
    if (validateThresholdRatio(value) && abs(value - pendingThresholdRatio) > 0.001f) {
        pendingThresholdRatio = value;
        saveThresholdRatioFlag = true;
    }
}

void EEPROMManager::scheduleSaveKp(double value) {
    if (validateKp(value) && abs(value - pendingKp) > 0.01) {
        pendingKp = value;
        saveKpFlag = true;
    }
}

void EEPROMManager::scheduleSaveKi(double value) {
    if (validateKi(value) && abs(value - pendingKi) > 0.0001) {
        pendingKi = value;
        saveKiFlag = true;
    }
}

void EEPROMManager::scheduleSaveKd(double value) {
    if (validateKd(value) && abs(value - pendingKd) > 0.0001) {
        pendingKd = value;
        saveKdFlag = true;
    }
}

void EEPROMManager::update() {
    unsigned long currentTime = millis();

    // Check minimum delay between EEPROM writes
    if (currentTime - lastEEPROMWrite < EEPROM_WRITE_INTERVAL) {
        return;
    }

    bool wrote = false;

    if (saveSetpointFlag) {
        writeFloat(EEPROM_SETPOINT_ADDR, pendingSetpoint);
        saveSetpointFlag = false;
        wrote = true;
    }

    if (saveCorrectionFactorFlag) {
        writeFloat(EEPROM_CORRECTION_FACTOR_ADDR, pendingCorrectionFactor);
        saveCorrectionFactorFlag = false;
        wrote = true;
    }

    if (saveCutSpeedFlag) {
        writeFloat(EEPROM_CUT_SPEED_ADDR, pendingCutSpeed);
        saveCutSpeedFlag = false;
        wrote = true;
    }

    if (saveThresholdRatioFlag) {
        writeFloat(EEPROM_THRESHOLD_RATIO_ADDR, pendingThresholdRatio);
        saveThresholdRatioFlag = false;
        wrote = true;
    }

    if (saveKpFlag) {
        writeDouble(EEPROM_KP_ADDR, pendingKp);
        saveKpFlag = false;
        wrote = true;
    }

    if (saveKiFlag) {
        writeDouble(EEPROM_KI_ADDR, pendingKi);
        saveKiFlag = false;
        wrote = true;
    }

    if (saveKdFlag) {
        writeDouble(EEPROM_KD_ADDR, pendingKd);
        saveKdFlag = false;
        wrote = true;
    }

    if (wrote) {
        lastEEPROMWrite = currentTime;
    }
}

void EEPROMManager::resetToDefaults() {
    THCParameters defaults;
    saveParameters(defaults);
    markEEPROMInitialized();
}

// Validation
bool EEPROMManager::validateSetpoint(float value) {
    return !isnan(value) && value >= SETPOINT_MIN && value <= SETPOINT_MAX;
}

bool EEPROMManager::validateCorrectionFactor(float value) {
    return !isnan(value) && value >= CORRECTION_FACTOR_MIN && value <= CORRECTION_FACTOR_MAX;
}

bool EEPROMManager::validateCutSpeed(float value) {
    return !isnan(value) && value >= 0 && value <= MAX_CUT_SPEED;
}

bool EEPROMManager::validateThresholdRatio(float value) {
    return !isnan(value) && value >= THRESHOLD_RATIO_MIN && value <= THRESHOLD_RATIO_MAX;
}

bool EEPROMManager::validateKp(double value) {
    return !isnan(value) && value >= 0.0 && value <= KP_MAX;
}

bool EEPROMManager::validateKi(double value) {
    return !isnan(value) && value >= 0.0 && value <= KI_MAX;
}

bool EEPROMManager::validateKd(double value) {
    return !isnan(value) && value >= 0.0 && value <= KD_MAX;
}

// Private methods
void EEPROMManager::writeFloat(int address, float value) {
    EEPROM.put(address, value);
}

float EEPROMManager::readFloat(int address) {
    float value;
    EEPROM.get(address, value);
    return value;
}

void EEPROMManager::writeDouble(int address, double value) {
    EEPROM.put(address, value);
}

double EEPROMManager::readDouble(int address) {
    double value;
    EEPROM.get(address, value);
    return value;
}

bool EEPROMManager::isEEPROMInitialized() {
    byte flag;
    EEPROM.get(EEPROM_INITIALIZED_FLAG, flag);
    return flag == EEPROM_INIT_FLAG_VALUE;
}

void EEPROMManager::markEEPROMInitialized() {
    EEPROM.put(EEPROM_INITIALIZED_FLAG, EEPROM_INIT_FLAG_VALUE);
}
