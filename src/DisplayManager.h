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
                float tempCorrectionFactor);

    // Anti-dive activation notification
    void notifyAntiDiveActivated();

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
    bool lastTgtArrow;          // true when → was last drawn after "Tgt" (on/off switch engaged)
    bool lastActArrow;          // true when → was last drawn after "Act" (THC regulation active)
    float lastZDelta;           // last drawn Z delta (mm); sentinel -999.0 = force redraw
    bool forceRedraw;           // one-shot: draw the static frame once after a screen change
    float lastTempCorrectionFactor;
    float lastUncorrectedFast;
    float lastAdjustedVoltage;

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
    void resetCachedValues();

    // Anti-Dive message handling
    void updateAntiDiveDisplay(unsigned long currentTime);
    void drawAntiDiveMessage();

    // Custom character creation
    void createCustomCharacters();
};

#endif // DISPLAY_MANAGER_H
