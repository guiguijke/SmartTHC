/**
 * SmartTHC - Display Manager
 *
 * Manages the 16x2 LCD with optimized refresh and blinking Anti-Dive message
 */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "Config.h"

// Forward declarations
class THCController;
class SpeedMonitor;

// Anti-Dive display state
enum AntiDiveDisplayState {
    AD_DISPLAY_IDLE,        // Normal display
    AD_DISPLAY_ACTIVE,      // Anti-dive message shown
    AD_DISPLAY_BLINK_ON,    // Blinking ON
    AD_DISPLAY_BLINK_OFF    // Blinking OFF
};

class DisplayManager {
public:
    DisplayManager();

    // Initialization
    void begin();

    // Display update
    void update(unsigned long currentTime, int currentScreen,
                THCController* thc, SpeedMonitor* speedMonitor,
                float tempCorrectionFactor, int encoderDelta);

    // Anti-dive activation notification
    void notifyAntiDiveActivated();

    // Force full refresh
    void forceRefresh();

private:
    LiquidCrystal_I2C lcd;

    // Display state
    int lastScreen;
    AntiDiveDisplayState adState;
    unsigned long adStartTime;
    unsigned long lastBlinkTime;
    bool blinkVisible;

    // Cached values to avoid unnecessary rewrites
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

    // Screen drawing methods
    void drawScreen0(THCController* thc, SpeedMonitor* speedMonitor);
    void drawScreen1(THCController* thc);
    void drawScreen2(THCController* thc, float tempCorrectionFactor);
    void drawScreen3(SpeedMonitor* speedMonitor);
    void drawScreen4(SpeedMonitor* speedMonitor);
    void drawScreen5(THCController* thc);
    void drawScreen6(THCController* thc);
    void drawScreen7(THCController* thc);

    // Utility methods
    void clearLine(int line);
    void printRightAligned(int col, int row, int value, int width);
    void printRightAligned(int col, int row, float value, int width, int decimals);
    void drawStatusIcons(THCController* thc);
    void resetCachedValues();

    // Anti-Dive message handling
    void updateAntiDiveDisplay(unsigned long currentTime);
    bool shouldShowAntiDiveMessage(unsigned long currentTime);
    void drawAntiDiveMessage();
    void clearAntiDiveMessage();

    // Custom character creation
    void createCustomCharacters();
};

#endif // DISPLAY_MANAGER_H
