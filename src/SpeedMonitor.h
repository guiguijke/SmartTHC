/**
 * SmartTHC - Speed Monitor
 *
 * Manages torch speed calculation via X/Y step interrupts
 */

#ifndef SPEED_MONITOR_H
#define SPEED_MONITOR_H

#include <Arduino.h>
#include "Config.h"

// Position history structure
struct PosHistory {
    unsigned long time;
    long position;
};

class SpeedMonitor {
public:
    SpeedMonitor();

    // Initialization
    void begin();

    // Periodic update (called in loop)
    void update(unsigned long currentTime);

    // Getters
    float getFilteredSpeed() const { return filteredSpeed; }
    float getThresholdSpeed() const { return thresholdSpeed; }

    // Configuration
    void setCutSpeed(float speed);
    void setThresholdRatio(float ratio);
    float getCutSpeed() const { return cutSpeed; }
    float getThresholdRatio() const { return thresholdRatio; }

    // Speed state
    bool isSpeedOK() const { return speedState; }
    bool isCutMotionDetected() const { return cutMotionDetected; }
    bool hasCutMotionStableSince(unsigned long timestamp) const;
    unsigned long getCutMotionStartTime() const { return cutMotionStartTime; }

    // Position history access (for anti-dive)
    long getPositionAtTime(unsigned long targetTime, long defaultPos) const;
    void recordPosition(unsigned long currentTime, long position);

    // Interrupt handlers (static for attachInterrupt)
    static void onStepX();
    static void onStepY();

private:
    // Step counters (volatile as modified by ISR)
    static volatile uint16_t stepCountX;
    static volatile uint16_t stepCountY;
    static volatile unsigned long lastStepXTime;
    static volatile unsigned long lastStepYTime;

    // Speed calculation
    float cutSpeed;
    float thresholdRatio;
    float thresholdSpeed;
    float filteredSpeed;
    bool speedState;
    bool cutMotionDetected;
    bool motionPending;
    unsigned long motionPendingStartTime;
    unsigned long cutMotionStartTime;

    // Filter
    float speedReadings[SPEED_FILTER_SIZE];
    int speedReadingIndex;
    float sumSpeedReadings;

    // Timing
    unsigned long lastSpeedTime;
    unsigned long lastStepTime;
    unsigned long totalStepX;
    unsigned long totalStepY;

    // Position history
    PosHistory positionHistory[POSITION_HISTORY_SIZE];
    int positionHistoryIndex;
    unsigned long lastPositionRecordTime;

    void calculateSpeed(unsigned long currentTime);
    void updatePositionHistory(unsigned long currentTime, long position);
};

#endif // SPEED_MONITOR_H
