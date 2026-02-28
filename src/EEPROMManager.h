/**
 * SmartTHC - Gestionnaire EEPROM
 * 
 * Gère la sauvegarde et le chargement des paramètres en EEPROM
 */

#ifndef EEPROM_MANAGER_H
#define EEPROM_MANAGER_H

#include <Arduino.h>
#include <EEPROM.h>
#include "Config.h"

// Structure pour les paramètres
struct THCParameters {
    float setpoint;
    float correctionFactor;
    float cutSpeed;
    float thresholdRatio;
    double kp;
    double ki;
    double kd;
    
    // Valeurs valides par défaut
    THCParameters() 
        : setpoint(DEFAULT_SETPOINT)
        , correctionFactor(DEFAULT_CORRECTION_FACTOR)
        , cutSpeed(DEFAULT_CUT_SPEED)
        , thresholdRatio(DEFAULT_THRESHOLD_RATIO)
        , kp(DEFAULT_KP)
        , ki(DEFAULT_KI)
        , kd(DEFAULT_KD)
    {}
};

class EEPROMManager {
public:
    EEPROMManager();
    
    // Initialisation - charge les paramètres ou initialise avec défaut
    void begin();
    
    // Chargement/sauvegarde complets
    void loadParameters(THCParameters& params);
    void saveParameters(const THCParameters& params);
    
    // Sauvegardes individuelles (avec délai)
    void scheduleSaveSetpoint(float value);
    void scheduleSaveCorrectionFactor(float value);
    void scheduleSaveCutSpeed(float value);
    void scheduleSaveThresholdRatio(float value);
    void scheduleSaveKp(double value);
    void scheduleSaveKi(double value);
    void scheduleSaveKd(double value);
    
    // Traitement des sauvegardes différées (à appeler dans loop)
    void update();
    
    // Réinitialisation
    void resetToDefaults();
    
    // Validation
    static bool validateSetpoint(float value);
    static bool validateCorrectionFactor(float value);
    static bool validateCutSpeed(float value);
    static bool validateThresholdRatio(float value);
    static bool validateKp(double value);
    static bool validateKi(double value);
    static bool validateKd(double value);

private:
    // Valeurs en attente de sauvegarde
    float pendingSetpoint;
    float pendingCorrectionFactor;
    float pendingCutSpeed;
    float pendingThresholdRatio;
    double pendingKp;
    double pendingKi;
    double pendingKd;
    
    // Flags de sauvegarde différée
    bool saveSetpointFlag;
    bool saveCorrectionFactorFlag;
    bool saveCutSpeedFlag;
    bool saveThresholdRatioFlag;
    bool saveKpFlag;
    bool saveKiFlag;
    bool saveKdFlag;
    
    unsigned long lastEEPROMWrite;
    
    void writeFloat(int address, float value);
    float readFloat(int address);
    void writeDouble(int address, double value);
    double readDouble(int address);
    
    bool isEEPROMInitialized();
    void markEEPROMInitialized();
};

#endif // EEPROM_MANAGER_H
