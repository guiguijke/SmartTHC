/**
 * SmartTHC - Display Manager
 *
 * LCD display implementation with blinking Anti-Dive message
 */

#include "DisplayManager.h"
#include "THCController.h"
#include "SpeedMonitor.h"

// Custom characters
static byte enableChar[8] = {0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b00000};
static byte plasmaChar[8] = {0b11111, 0b11111, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
static byte thcActifChar[8] = {0b11111, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111, 0b00000};
static byte arrowUp[8] = {0b00100, 0b01110, 0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00000};
static byte arrowDown[8] = {0b00100, 0b00100, 0b00100, 0b00100, 0b11111, 0b01110, 0b00100, 0b00000};
static byte stableChar[8] = {0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000};
static byte enableall[8] = {0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b00000};

DisplayManager::DisplayManager()
    : lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS)
    , lastScreen(-1)
    , adState(AD_DISPLAY_IDLE)
    , adStartTime(0)
    , lastBlinkTime(0)
    , blinkVisible(true)
    , lastActualVoltage(-1.0f)
    , lastSetpoint(-1.0f)
    , lastSpeed(-1)
    , lastThcActive(false)
    , lastOutput(0.0)
    , lastEnableLow(false)
    , lastPlasmaLow(false)
    , lastThcOff(false)
    , lastCutSpeed(-1.0f)
    , lastTempCorrectionFactor(-1.0f)
    , lastUncorrectedFast(-1.0f)
    , lastAdjustedVoltage(-1.0f)
    , lastAntiDiveDisplayActive(false)
{
}

void DisplayManager::begin() {
    lcd.init();
    lcd.begin(LCD_COLUMNS, LCD_ROWS);
    lcd.backlight();

    createCustomCharacters();
}

void DisplayManager::createCustomCharacters() {
    lcd.createChar(CHAR_ENABLE, enableChar);
    lcd.createChar(CHAR_PLASMA, plasmaChar);
    lcd.createChar(CHAR_THC_ACTIVE, thcActifChar);
    lcd.createChar(CHAR_ARROW_UP, arrowUp);
    lcd.createChar(CHAR_ARROW_DOWN, arrowDown);
    lcd.createChar(CHAR_STABLE, stableChar);
    lcd.createChar(CHAR_ENABLE_ALL, enableall);
}

void DisplayManager::update(unsigned long currentTime, int currentScreen,
                            THCController* thc, SpeedMonitor* speedMonitor,
                            float tempCorrectionFactor, int encoderDelta) {
    // Update Anti-Dive message state
    updateAntiDiveDisplay(currentTime);

    // Screen change
    if (currentScreen != lastScreen) {
        lcd.clear();
        lastScreen = currentScreen;
        resetCachedValues();
    }

    // Draw appropriate screen
    switch (currentScreen) {
        case 0:
            drawScreen0(thc, speedMonitor);
            break;
        case 1:
            drawScreen1(thc);
            break;
        case 2:
            drawScreen2(thc, tempCorrectionFactor);
            break;
        case 3:
            drawScreen3(speedMonitor);
            break;
        case 4:
            drawScreen4(speedMonitor);
            break;
        case 5:
            drawScreen5(thc);
            break;
        case 6:
            drawScreen6(thc);
            break;
        case 7:
            drawScreen7(thc);
            break;
    }
}

void DisplayManager::notifyAntiDiveActivated() {
    adState = AD_DISPLAY_ACTIVE;
    adStartTime = millis();
    blinkVisible = true;
    lastBlinkTime = adStartTime;
}

void DisplayManager::updateAntiDiveDisplay(unsigned long currentTime) {
    switch (adState) {
        case AD_DISPLAY_IDLE:
            // Nothing to do
            break;

        case AD_DISPLAY_ACTIVE:
            // Check if display time has elapsed
            if (currentTime - adStartTime >= ANTI_DIVE_DISPLAY_DURATION) {
                adState = AD_DISPLAY_IDLE;
                // Force speed zone refresh
                lastSpeed = -1;
            } else {
                // Switch to blink mode
                adState = AD_DISPLAY_BLINK_ON;
                lastBlinkTime = currentTime;
            }
            break;

        case AD_DISPLAY_BLINK_ON:
            if (currentTime - lastBlinkTime >= ANTI_DIVE_BLINK_INTERVAL) {
                adState = AD_DISPLAY_BLINK_OFF;
                lastBlinkTime = currentTime;
                blinkVisible = false;
                // Force refresh
                lastSpeed = -1;
            }
            break;

        case AD_DISPLAY_BLINK_OFF:
            if (currentTime - lastBlinkTime >= ANTI_DIVE_BLINK_INTERVAL) {
                // Check if total time has elapsed
                if (currentTime - adStartTime >= ANTI_DIVE_DISPLAY_DURATION) {
                    adState = AD_DISPLAY_IDLE;
                    blinkVisible = true;
                } else {
                    adState = AD_DISPLAY_BLINK_ON;
                    blinkVisible = true;
                }
                lastBlinkTime = currentTime;
                // Force refresh
                lastSpeed = -1;
            }
            break;
    }
}

bool DisplayManager::shouldShowAntiDiveMessage(unsigned long currentTime) {
    return (adState == AD_DISPLAY_ACTIVE ||
            adState == AD_DISPLAY_BLINK_ON ||
            (adState == AD_DISPLAY_BLINK_OFF && !blinkVisible));
}

void DisplayManager::drawAntiDiveMessage() {
    lcd.setCursor(12, 0);
    if (blinkVisible) {
        lcd.print("ADIV");
    } else {
        lcd.print("    ");
    }
}

void DisplayManager::clearAntiDiveMessage() {
    // Nothing to do, handled in drawScreen0
}

void DisplayManager::drawScreen0(THCController* thc, SpeedMonitor* speedMonitor) {
    // Line 0: "Act: XXX.X V SPD"
    float actualVoltage = thc->getFastVoltage();
    if (actualVoltage != lastActualVoltage) {
        lcd.setCursor(0, 0);
        lcd.print("Act:");
        lcd.setCursor(4, 0);
        char vBuf[8];
        dtostrf(actualVoltage, 5, 1, vBuf);  // Fixed width 5 chars: "  9.4" or "123.4"
        lcd.print(vBuf);
        lcd.print("V ");  // V + space to clear any leftover
        lastActualVoltage = actualVoltage;
    }

    // Speed zone or Anti-Dive message
    if (adState != AD_DISPLAY_IDLE) {
        drawAntiDiveMessage();
    } else {
        // Normal speed display
        int displayedSpeed = (int)(speedMonitor->getFilteredSpeed());
        if (displayedSpeed > 9999) displayedSpeed = 9999;

        if (speedMonitor->getFilteredSpeed() < 0.1f) {
            if (lastSpeed != 0) {
                lcd.setCursor(12, 0);
                lcd.print("   0");
                lastSpeed = 0;
            }
        } else if (displayedSpeed != lastSpeed) {
            lcd.setCursor(12, 0);
            char buffer[5];
            snprintf(buffer, sizeof(buffer), "%4d", displayedSpeed);
            lcd.print(buffer);
            lastSpeed = displayedSpeed;
        }
    }

    // Line 1: "Tgt: XXX.X V [status]"
    float setpoint = thc->getSetpoint();
    if (setpoint != lastSetpoint) {
        lcd.setCursor(0, 1);
        lcd.print("Tgt:");
        lcd.setCursor(4, 1);
        char sBuf[8];
        dtostrf(setpoint, 5, 1, sBuf);  // Fixed width 5 chars
        lcd.print(sBuf);
        lcd.print("V");
        lastSetpoint = setpoint;
    }

    // Status icons
    drawStatusIcons(thc);
}

void DisplayManager::drawStatusIcons(THCController* thc) {
    bool enableLow = thc->isEnableLow();
    bool plasmaLow = thc->isPlasmaActive();
    bool thcActive = thc->isTHCActive();
    bool thcOff = thc->isTHCOff();
    double output = thc->getPidOutput();

    if (enableLow != lastEnableLow || plasmaLow != lastPlasmaLow ||
        thcActive != lastThcActive || thcOff != lastThcOff ||
        output != lastOutput) {

        lcd.setCursor(11, 1);
        // Enable
        lcd.write(enableLow ? CHAR_ENABLE : (byte)'O');

        lcd.setCursor(12, 1);
        // THC direction
        if (thcActive) {
            if (output > 10) {
                lcd.write(CHAR_ARROW_UP);
            } else if (output < -10) {
                lcd.write(CHAR_ARROW_DOWN);
            } else {
                lcd.write(CHAR_STABLE);
            }
        } else {
            lcd.print(" ");
        }

        lcd.setCursor(13, 1);
        // THC off/active
        if (thcOff && !thcActive) {
            lcd.print("o");
        } else if (thcOff && thcActive) {
            lcd.write(CHAR_THC_ACTIVE);
        } else {
            lcd.print("-");
        }

        lcd.setCursor(14, 1);
        // Plasma
        if (plasmaLow) {
            lcd.write(CHAR_PLASMA);
        } else {
            lcd.print(" ");
        }

        lastEnableLow = enableLow;
        lastPlasmaLow = plasmaLow;
        lastThcActive = thcActive;
        lastThcOff = thcOff;
        lastOutput = output;
    }
}

void DisplayManager::drawScreen1(THCController* thc) {
    lcd.setCursor(0, 0);
    lcd.print("Set V:      V");
    lcd.setCursor(7, 0);
    lcd.print(thc->getSetpoint(), 1);
}

void DisplayManager::drawScreen2(THCController* thc, float tempCorrectionFactor) {
    // Line 0: "V Corr: XX.XX"
    if (tempCorrectionFactor != lastTempCorrectionFactor) {
        lcd.setCursor(0, 0);
        lcd.print("V Corr:     ");
        lcd.setCursor(8, 0);
        lcd.print(tempCorrectionFactor, 2);
        lastTempCorrectionFactor = tempCorrectionFactor;
    }

    // Line 1: "Base:XXX.X Adj:XXX.X"
    float uncorrected = thc->getUncorrectedFast();
    float adjusted = uncorrected * tempCorrectionFactor;

    if (uncorrected != lastUncorrectedFast || adjusted != lastAdjustedVoltage) {
        lcd.setCursor(0, 1);
        lcd.print("Base:    Adj:   ");
        lcd.setCursor(5, 1);
        lcd.print(uncorrected, 1);
        lcd.setCursor(13, 1);
        lcd.print(adjusted, 1);
        lastUncorrectedFast = uncorrected;
        lastAdjustedVoltage = adjusted;
    }
}

void DisplayManager::drawScreen3(SpeedMonitor* speedMonitor) {
    lcd.setCursor(0, 0);
    lcd.print("CutSpd:     ");
    lcd.setCursor(8, 0);
    char buffer[5];
    snprintf(buffer, sizeof(buffer), "%4.0f", speedMonitor->getCutSpeed());
    lcd.print(buffer);

    lcd.setCursor(0, 1);
    lcd.print("Unit: ");
    lcd.print(SPEED_UNIT_LONG);
}

void DisplayManager::drawScreen4(SpeedMonitor* speedMonitor) {
    lcd.setCursor(0, 0);
    lcd.print("Spd Ths %:  ");
    lcd.setCursor(11, 0);
    lcd.print(speedMonitor->getThresholdRatio(), 1);
}

void DisplayManager::drawScreen5(THCController* thc) {
    lcd.setCursor(0, 0);
    lcd.print("PID Kp:      ");
    lcd.setCursor(8, 0);
    lcd.print(thc->getKp(), 3);
}

void DisplayManager::drawScreen6(THCController* thc) {
    lcd.setCursor(0, 0);
    lcd.print("PID Ki:      ");
    lcd.setCursor(8, 0);
    lcd.print(thc->getKi(), 4);
}

void DisplayManager::drawScreen7(THCController* thc) {
    lcd.setCursor(0, 0);
    lcd.print("PID Kd:      ");
    lcd.setCursor(8, 0);
    lcd.print(thc->getKd(), 3);
}

void DisplayManager::clearLine(int line) {
    lcd.setCursor(0, line);
    for (int i = 0; i < LCD_COLUMNS; i++) {
        lcd.print(" ");
    }
}

void DisplayManager::printRightAligned(int col, int row, int value, int width) {
    lcd.setCursor(col, row);
    char buffer[6];
    snprintf(buffer, sizeof(buffer), "%*d", width, value);
    lcd.print(buffer);
}

void DisplayManager::printRightAligned(int col, int row, float value, int width, int decimals) {
    lcd.setCursor(col, row);
    char format[10];
    snprintf(format, sizeof(format), "%%%d.%df", width, decimals);
    char buffer[10];
    snprintf(buffer, sizeof(buffer), format, value);
    lcd.print(buffer);
}

void DisplayManager::forceRefresh() {
    lastScreen = -1;
    resetCachedValues();
}

void DisplayManager::resetCachedValues() {
    lastActualVoltage = -1.0f;
    lastSetpoint = -1.0f;
    lastSpeed = -1;
    lastThcActive = false;
    lastOutput = 0.0;
    lastEnableLow = false;
    lastPlasmaLow = false;
    lastThcOff = false;
    lastCutSpeed = -1.0f;
    lastTempCorrectionFactor = -1.0f;
    lastUncorrectedFast = -1.0f;
    lastAdjustedVoltage = -1.0f;
}
