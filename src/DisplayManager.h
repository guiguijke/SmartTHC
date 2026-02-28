/**
 * SmartTHC - Gestionnaire d'affichage
 * 
 * Gère l'affichage LCD 16x2 avec rafraîchissement optimisé
 * et message clignotant Anti-Dive
 */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "Config.h"

// Forward declarations
class THCController;
class SpeedMonitor;

// État de l'affichage Anti-Dive
enum AntiDiveDisplayState {
    AD_DISPLAY_IDLE,        // Affichage normal
    AD_DISPLAY_ACTIVE,      // Message anti-dive affiché
    AD_DISPLAY_BLINK_ON,    // Clignotant ON
    AD_DISPLAY_BLINK_OFF    // Clignotant OFF
};

class DisplayManager {
public:
    DisplayManager();
    
    // Initialisation
    void begin();
    
    // Mise à jour de l'affichage
    void update(unsigned long currentTime, int currentScreen, 
                THCController* thc, SpeedMonitor* speedMonitor,
                float tempCorrectionFactor, int encoderDelta);
    
    // Notification d'activation anti-dive
    void notifyAntiDiveActivated();
    
    // Forcer le rafraîchissement complet
    void forceRefresh();

private:
    LiquidCrystal_I2C lcd;
    
    // État de l'affichage
    int lastScreen;
    AntiDiveDisplayState adState;
    unsigned long adStartTime;
    unsigned long lastBlinkTime;
    bool blinkVisible;
    
    // Variables pour éviter les réécritures inutiles
    float lastActualVoltage;
    float lastSetpoint;
    int lastSpeed;
    bool lastThcActive;
    double lastOutput;
    bool lastEnableLow;
    bool lastPlasmaLow;
    bool lastThcOff;
    float lastCutSpeed;
    float lastTempCorrectionFactor;
    float lastUncorrectedFast;
    float lastAdjustedVoltage;
    bool lastAntiDiveDisplayActive;
    
    // Méthodes d'affichage par écran
    void drawScreen0(THCController* thc, SpeedMonitor* speedMonitor);
    void drawScreen1(THCController* thc);
    void drawScreen2(THCController* thc, float tempCorrectionFactor);
    void drawScreen3(SpeedMonitor* speedMonitor);
    void drawScreen4(SpeedMonitor* speedMonitor);
    void drawScreen5(THCController* thc);
    void drawScreen6(THCController* thc);
    void drawScreen7(THCController* thc);
    
    // Méthodes utilitaires
    void clearLine(int line);
    void printRightAligned(int col, int row, int value, int width);
    void printRightAligned(int col, int row, float value, int width, int decimals);
    void drawStatusIcons(THCController* thc);
    void resetCachedValues();
    
    // Gestion du message Anti-Dive
    void updateAntiDiveDisplay(unsigned long currentTime);
    bool shouldShowAntiDiveMessage(unsigned long currentTime);
    void drawAntiDiveMessage();
    void clearAntiDiveMessage();
    
    // Création des caractères personnalisés
    void createCustomCharacters();
};

#endif // DISPLAY_MANAGER_H
