/**
 * SmartTHC - Gestionnaire de commandes série
 * 
 * Implémentation des commandes série et logging
 */

#include "SerialCommand.h"
#include "EEPROMManager.h"
#include "THCController.h"
#include "SpeedMonitor.h"
#include <ArduPID.h>

SerialCommand::SerialCommand()
    : lastLogTime(0)
    , lastLoopLogTime(0)
{
}

void SerialCommand::begin(long baudRate) {
    Serial.begin(baudRate);
}

void SerialCommand::update(EEPROMManager* eeprom, THCController* thc, SpeedMonitor* speed) {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        processCommand(command, eeprom, thc, speed);
    }
}

void SerialCommand::processCommand(String& command, EEPROMManager* eeprom, 
                                   THCController* thc, SpeedMonitor* speed) {
    if (command == "RESET_EEPROM") {
        eeprom->resetToDefaults();
        
        // Recharger les paramètres
        THCParameters params;
        eeprom->loadParameters(params);
        
        thc->setSetpoint(params.setpoint);
        thc->setCorrectionFactor(params.correctionFactor);
        speed->setCutSpeed(params.cutSpeed);
        speed->setThresholdRatio(params.thresholdRatio);
        thc->setKp(params.kp);
        thc->setKi(params.ki);
        thc->setKd(params.kd);
        
        Serial.println("EEPROM reset via serial command");
    }
    else if (command == "STATUS") {
        printStatus(thc, speed);
    }
    else if (command == "HELP") {
        Serial.println("Commands: RESET_EEPROM, STATUS, HELP");
    }
}

void SerialCommand::logStatus(unsigned long currentTime, THCController* thc, SpeedMonitor* speed) {
    if (currentTime - lastLogTime < LOG_INTERVAL) {
        return;
    }
    
    bool plasmaPinLow = thc->isPlasmaActive();
    bool thcOff = thc->isTHCOff();
    bool enablePinLow = thc->isEnableLow();
    bool arcDetected = thc->isArcDetected();
    bool thcActive = thc->isTHCActive();
    bool antiDiveActive = thc->isAntiDiveActive();
    
    if (plasmaPinLow) {
        Serial.print("TGT / Torch speed: ");
        Serial.print(speed->getCutSpeed());
        Serial.print(" / ");
        Serial.print(speed->getFilteredSpeed());
        Serial.print(" ");
        Serial.print(SPEED_UNIT);
        Serial.print(" | Threshold speed: ");
        Serial.print(speed->getThresholdSpeed());
        Serial.print(" ");
        Serial.println(SPEED_UNIT);
        
        Serial.print("PLASMA_PIN: LOW | ENABLE_PIN: ");
        Serial.print(enablePinLow ? "LOW" : "HIGH");
        Serial.print(" | Stabilization: ");
        Serial.print(thc->isPlasmaStabilized() ? "OK" : "Waiting");
        Serial.print(" | Arc detected: ");
        Serial.print(arcDetected ? "Yes" : "No");
        Serial.print(" | THC active: ");
        Serial.print(thcActive ? "Yes" : "No");
        Serial.print(" | THC_SIG: ");
        Serial.print(thcOff ? "ACTIVE" : "INACTIVE");
        Serial.print(" | Speed OK: ");
        Serial.println(speed->isSpeedOK() ? "Yes" : "No");
        
        Serial.print("V | Fast: ");
        Serial.print(thc->getFastVoltage());
        Serial.print("V | Slow: ");
        Serial.print(thc->getSlowVoltage());
        Serial.print("V | Anti-dive: ");
        Serial.println(antiDiveActive ? "Active" : "Inactive");
        
        // Log si THC inactif
        if (!thcActive) {
            Serial.print("Reason THC inactive: ");
            if (!thcOff) Serial.println("THC_OFF_PIN LOW");
            else if (!enablePinLow) Serial.println("ENABLE_PIN HIGH");
            else if (!plasmaPinLow) Serial.println("PLASMA_PIN HIGH");
            else if (!thc->isPlasmaStabilized()) Serial.println("Stabilization not reached");
            else if (!arcDetected) Serial.println("Arc not detected");
            else if (!speed->isSpeedOK()) Serial.println("Torch speed < threshold");
            else if (antiDiveActive) Serial.println("Anti-dive active");
            else Serial.println("Unknown condition");
        }
    } else {
        Serial.print("Cut Speed setup: ");
        Serial.print(speed->getCutSpeed());
        Serial.print(" ");
        Serial.print(SPEED_UNIT);
        Serial.print(" | Fast voltage: ");
        Serial.print(thc->getFastVoltage());
        Serial.print("V | PLASMA_PIN: HIGH | THC_SIG: ");
        Serial.println(thcOff ? "ACTIVE" : "INACTIVE");
    }
    
    lastLogTime = currentTime;
}

void SerialCommand::logLoopStats(unsigned long currentTime, unsigned long avgTime, float frequency) {
    if (currentTime - lastLoopLogTime >= LOOP_LOG_INTERVAL) {
        Serial.print("Average execution time: ");
        Serial.print(avgTime);
        Serial.print(" us | Frequency: ");
        Serial.print(frequency, 1);
        Serial.println(" Hz");
        lastLoopLogTime = currentTime;
    }
}

void SerialCommand::printStatus(THCController* thc, SpeedMonitor* speed) {
    Serial.println("=== SmartTHC Status ===");
    Serial.print("Setpoint: ");
    Serial.println(thc->getSetpoint());
    Serial.print("Fast Voltage: ");
    Serial.println(thc->getFastVoltage());
    Serial.print("THC Active: ");
    Serial.println(thc->isTHCActive() ? "Yes" : "No");
    Serial.print("Speed: ");
    Serial.println(speed->getFilteredSpeed());
}
