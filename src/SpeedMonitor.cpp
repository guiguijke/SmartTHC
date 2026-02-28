/**
 * SmartTHC - Moniteur de vitesse
 * 
 * Implémentation du calcul de vitesse torche
 */

#include "SpeedMonitor.h"

// Définitions des membres statiques
volatile uint16_t SpeedMonitor::stepCountX = 0;
volatile uint16_t SpeedMonitor::stepCountY = 0;
volatile unsigned long SpeedMonitor::lastStepXTime = 0;
volatile unsigned long SpeedMonitor::lastStepYTime = 0;

SpeedMonitor::SpeedMonitor()
    : cutSpeed(DEFAULT_CUT_SPEED)
    , thresholdRatio(DEFAULT_THRESHOLD_RATIO)
    , thresholdSpeed(DEFAULT_CUT_SPEED * DEFAULT_THRESHOLD_RATIO)
    , filteredSpeed(0.0f)
    , speedState(false)
    , speedReadingIndex(0)
    , sumSpeedReadings(0.0f)
    , lastSpeedTime(0)
    , lastStepTime(0)
    , totalStepX(0)
    , totalStepY(0)
    , positionHistoryIndex(0)
    , lastPositionRecordTime(0)
{
    // Initialiser le tableau de filtrage
    for (int i = 0; i < SPEED_FILTER_SIZE; i++) {
        speedReadings[i] = 0.0f;
    }
    
    // Initialiser l'historique de position
    for (int i = 0; i < POSITION_HISTORY_SIZE; i++) {
        positionHistory[i].time = 0;
        positionHistory[i].position = 0;
    }
}

void SpeedMonitor::begin() {
    pinMode(STEP_X_PIN, INPUT);
    pinMode(STEP_Y_PIN, INPUT);
    
    // Attacher les interruptions
    attachInterrupt(digitalPinToInterrupt(STEP_X_PIN), onStepX, FALLING);
    attachInterrupt(digitalPinToInterrupt(STEP_Y_PIN), onStepY, FALLING);
}

void SpeedMonitor::update(unsigned long currentTime) {
    // Enregistrer l'historique de position si nécessaire
    // (la position actuelle doit être passée depuis l'extérieur)
    
    // Calculer la vitesse à intervalles réguliers
    if (currentTime - lastSpeedTime >= SPEED_INTERVAL) {
        calculateSpeed(currentTime);
    }
}

void SpeedMonitor::setCutSpeed(float speed) {
    if (speed < 0) speed = 0;
    if (speed > MAX_CUT_SPEED) speed = MAX_CUT_SPEED;
    cutSpeed = speed;
    thresholdSpeed = cutSpeed * thresholdRatio;
}

void SpeedMonitor::setThresholdRatio(float ratio) {
    if (ratio < THRESHOLD_RATIO_MIN) ratio = THRESHOLD_RATIO_MIN;
    if (ratio > THRESHOLD_RATIO_MAX) ratio = THRESHOLD_RATIO_MAX;
    thresholdRatio = ratio;
    thresholdSpeed = cutSpeed * thresholdRatio;
}

void SpeedMonitor::calculateSpeed(unsigned long currentTime) {
    // Récupérer et réinitialiser les compteurs (section critique)
    noInterrupts();
    uint16_t tempX = stepCountX;
    uint16_t tempY = stepCountY;
    stepCountX = 0;
    stepCountY = 0;
    interrupts();
    
    totalStepX += tempX;
    totalStepY += tempY;
    
    // Calculer la vitesse
    float torchSpeed = 0.0f;
    unsigned long deltaTime = currentTime - lastSpeedTime;
    
    if ((tempX + tempY) >= MIN_STEPS && deltaTime > 0) {
        float distanceX = tempX * DIST_PER_STEP_X;
        float distanceY = tempY * DIST_PER_STEP_Y;
        float totalDistance = sqrt(distanceX * distanceX + distanceY * distanceY);
        torchSpeed = (totalDistance / (deltaTime / 1000.0f)) * 60.0f * SPEED_CORRECTION;
    }
    
    // Appliquer le filtre
    sumSpeedReadings -= speedReadings[speedReadingIndex];
    speedReadings[speedReadingIndex] = torchSpeed;
    sumSpeedReadings += torchSpeed;
    speedReadingIndex = (speedReadingIndex + 1) % SPEED_FILTER_SIZE;
    filteredSpeed = sumSpeedReadings / SPEED_FILTER_SIZE;
    
    if (filteredSpeed < 0) filteredSpeed = 0.0f;
    
    // Reset si timeout
    if ((tempX + tempY) == 0 && (currentTime - lastStepTime > STEP_TIMEOUT)) {
        totalStepX = 0;
        totalStepY = 0;
        for (int i = 0; i < SPEED_FILTER_SIZE; i++) {
            speedReadings[i] = 0.0f;
        }
        sumSpeedReadings = 0.0f;
        filteredSpeed = 0.0f;
    }
    
    if ((tempX + tempY) > 0) {
        lastStepTime = currentTime;
    }
    
    // Mettre à jour l'état de vitesse
    if (!speedState && filteredSpeed >= thresholdSpeed) {
        speedState = true;
    } else if (speedState && filteredSpeed < thresholdSpeed) {
        speedState = false;
    }
    
    lastSpeedTime = currentTime;
}

long SpeedMonitor::getPositionAtTime(unsigned long targetTime, long defaultPos) const {
    long closestPos = defaultPos;
    unsigned long closestDiff = 0xFFFFFFFFUL;
    
    for (int i = 0; i < POSITION_HISTORY_SIZE; i++) {
        unsigned long hTime = positionHistory[i].time;
        if (hTime == 0) continue;
        
        unsigned long diff = (unsigned long)abs((long)targetTime - (long)hTime);
        if (diff < closestDiff) {
            closestDiff = diff;
            closestPos = positionHistory[i].position;
        }
    }
    
    return closestPos;
}

void SpeedMonitor::recordPosition(unsigned long currentTime, long position) {
    if (currentTime - lastPositionRecordTime >= POSITION_HISTORY_INTERVAL) {
        positionHistory[positionHistoryIndex].time = currentTime;
        positionHistory[positionHistoryIndex].position = position;
        positionHistoryIndex = (positionHistoryIndex + 1) % POSITION_HISTORY_SIZE;
        lastPositionRecordTime = currentTime;
    }
}

void SpeedMonitor::updatePositionHistory(unsigned long currentTime, long position) {
    recordPosition(currentTime, position);
}

// Handlers d'interruption
void SpeedMonitor::onStepX() {
    unsigned long now = micros();
    if (now - lastStepXTime > DEBOUNCE_US) {
        stepCountX++;
    }
    lastStepXTime = now;
}

void SpeedMonitor::onStepY() {
    unsigned long now = micros();
    if (now - lastStepYTime > DEBOUNCE_US) {
        stepCountY++;
    }
    lastStepYTime = now;
}
