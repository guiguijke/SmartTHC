/**
 * SmartTHC - Moniteur de vitesse
 * 
 * Gère le calcul de la vitesse de la torche via les interruptions X/Y
 */

#ifndef SPEED_MONITOR_H
#define SPEED_MONITOR_H

#include <Arduino.h>
#include "Config.h"

// Structure pour stocker l'historique de position
struct PosHistory {
    unsigned long time;
    long position;
};

class SpeedMonitor {
public:
    SpeedMonitor();
    
    // Initialisation
    void begin();
    
    // Mise à jour périodique (appelée dans loop)
    void update(unsigned long currentTime);
    
    // Accesseurs
    float getFilteredSpeed() const { return filteredSpeed; }
    float getThresholdSpeed() const { return thresholdSpeed; }
    
    // Configuration
    void setCutSpeed(float speed);
    void setThresholdRatio(float ratio);
    float getCutSpeed() const { return cutSpeed; }
    float getThresholdRatio() const { return thresholdRatio; }
    
    // État vitesse
    bool isSpeedOK() const { return speedState; }
    
    // Accès à l'historique de position (pour anti-dive)
    long getPositionAtTime(unsigned long targetTime, long defaultPos) const;
    void recordPosition(unsigned long currentTime, long position);
    
    // Handlers d'interruption (static pour attachInterrupt)
    static void onStepX();
    static void onStepY();

private:
    // Compteurs de pas (volatile car modifiés par ISR)
    static volatile uint16_t stepCountX;
    static volatile uint16_t stepCountY;
    static volatile unsigned long lastStepXTime;
    static volatile unsigned long lastStepYTime;
    
    // Calcul vitesse
    float cutSpeed;
    float thresholdRatio;
    float thresholdSpeed;
    float filteredSpeed;
    bool speedState;
    
    // Filtre
    float speedReadings[SPEED_FILTER_SIZE];
    int speedReadingIndex;
    float sumSpeedReadings;
    
    // Timing
    unsigned long lastSpeedTime;
    unsigned long lastStepTime;
    unsigned long totalStepX;
    unsigned long totalStepY;
    
    // Historique position
    PosHistory positionHistory[POSITION_HISTORY_SIZE];
    int positionHistoryIndex;
    unsigned long lastPositionRecordTime;
    
    void calculateSpeed(unsigned long currentTime);
    void updatePositionHistory(unsigned long currentTime, long position);
};

#endif // SPEED_MONITOR_H
