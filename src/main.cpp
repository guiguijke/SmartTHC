/**
 * SmartTHC - Main
 * 
 * Orchestration du système THC modulaire
 * Arduino Uno R4 Minima
 */

#include <Arduino.h>
#include "Config.h"
#include "EncoderManager.h"
#include "SpeedMonitor.h"
#include "EEPROMManager.h"
#include "THCController.h"
#include "DisplayManager.h"
#include "SerialCommand.h"

// ============================================================================
// INSTANCES DES MODULES
// ============================================================================

EncoderManager encoder;
SpeedMonitor speedMonitor;
EEPROMManager eeprom;
THCController thc;
DisplayManager display;
SerialCommand serialCmd;

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

int currentScreen = 0;
int prevScreen = -1;
float tempCorrectionFactor = DEFAULT_CORRECTION_FACTOR;

// Timing
unsigned long lastDisplayTime = 0;
unsigned long lastPidTime = 0;

// Stats boucle
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
    // Initialisation série
    serialCmd.begin(115200);
    
    Serial.println("SmartTHC - Initializing...");
    
    // Initialisation EEPROM
    eeprom.begin();
    
    // Charger les paramètres
    THCParameters params;
    eeprom.loadParameters(params);
    
    // Appliquer les paramètres
    thc.setSetpoint(params.setpoint);
    thc.setCorrectionFactor(params.correctionFactor);
    tempCorrectionFactor = params.correctionFactor;
    
    speedMonitor.setCutSpeed(params.cutSpeed);
    speedMonitor.setThresholdRatio(params.thresholdRatio);
    
    thc.setKp(params.kp);
    thc.setKi(params.ki);
    thc.setKd(params.kd);
    
    // Initialisation des modules
    encoder.begin();
    speedMonitor.begin();
    thc.begin();
    display.begin();
    
    Serial.println("SmartTHC - Ready!");
}

// ============================================================================
// LOOP PRINCIPALE
// ============================================================================

void loop() {
    loopStartTime = micros();
    unsigned long currentTime = millis();
    
    // ------------------------------------------------------------------------
    // 1. Mise à jour de l'encodeur (lecture rotation + bouton)
    // ------------------------------------------------------------------------
    encoder.update();
    
    // ------------------------------------------------------------------------
    // 2. Gestion du bouton (changement d'écran)
    // ------------------------------------------------------------------------
    if (encoder.isButtonClicked()) {
        prevScreen = currentScreen;
        currentScreen = (currentScreen + 1) % NB_SCREENS;
        
        // Entrée/sortie de l'écran de correction temporaire
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
    // 3. Gestion de l'encodeur rotatif (ajustement des paramètres)
    // ------------------------------------------------------------------------
    int encoderDelta = encoder.getDelta();
    if (encoderDelta != 0) {
        handleParameterAdjustment(encoderDelta);
    }
    
    // ------------------------------------------------------------------------
    // 4. Mise à jour du moniteur de vitesse
    // ------------------------------------------------------------------------
    speedMonitor.update(currentTime);
    
    // Enregistrer la position pour l'anti-dive
    speedMonitor.recordPosition(currentTime, thc.getMotorPosition());
    
    // ------------------------------------------------------------------------
    // 5. Mise à jour du contrôleur THC (PID, anti-dive, moteur)
    // ------------------------------------------------------------------------
    // Exécuter à 1kHz
    unsigned long currentMicros = micros();
    if (currentMicros - lastPidTime >= PID_INTERVAL_US) {
        thc.update(currentTime);
        lastPidTime = currentMicros;
        
        // Notifier l'affichage si anti-dive vient de s'activer
        notifyAntiDiveIfNeeded();
    }
    
    // Faire tourner le moteur aussi souvent que possible
    thc.runMotor();
    
    // ------------------------------------------------------------------------
    // 6. Mise à jour de l'affichage LCD
    // ------------------------------------------------------------------------
    if (currentTime - lastDisplayTime >= DISPLAY_INTERVAL) {
        display.update(currentTime, currentScreen, &thc, &speedMonitor, 
                       tempCorrectionFactor, encoderDelta);
        lastDisplayTime = currentTime;
    }
    
    // ------------------------------------------------------------------------
    // 7. Traitement des commandes série et logging
    // ------------------------------------------------------------------------
    serialCmd.update(&eeprom, &thc, &speedMonitor);
    serialCmd.logStatus(currentTime, &thc, &speedMonitor);
    
    // ------------------------------------------------------------------------
    // 8. Sauvegardes EEPROM différées
    // ------------------------------------------------------------------------
    eeprom.update();
    
    // ------------------------------------------------------------------------
    // 9. Stats d'exécution
    // ------------------------------------------------------------------------
    unsigned long loopEndTime = micros();
    unsigned long loopExecutionTime = loopEndTime - loopStartTime;
    loopExecutionTimeSum += loopExecutionTime;
    loopCount++;
    
    // Log des stats toutes les 10 secondes
    if (currentTime % LOOP_LOG_INTERVAL < DISPLAY_INTERVAL && loopCount > 0) {
        unsigned long avgLoopTime = loopExecutionTimeSum / loopCount;
        float loopFrequency = 1000000.0f / avgLoopTime;
        serialCmd.logLoopStats(currentTime, avgLoopTime, loopFrequency);
        loopExecutionTimeSum = 0;
        loopCount = 0;
    }
}

// ============================================================================
// FONCTIONS AUXILIAIRES
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
        case 2: { // Correction factor (temporaire)
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
    // Vérifier si l'anti-dive vient de s'activer
    AntiDiveEvent event = thc.getAntiDiveEvent();
    if (event.justActivated) {
        display.notifyAntiDiveActivated();
        thc.clearAntiDiveJustActivated();
    }
}
