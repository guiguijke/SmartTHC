/**
 * SmartTHC - Display Manager
 *
 * LCD display implementation with blinking Anti-Dive message
 */

#include "DisplayManager.h"
#include "THCController.h"
#include "SpeedMonitor.h"

// Custom characters
// → used after "Act"/"Tgt" to show THC engagement / active correction.
static byte arrowRight[8] = {
    0b00000,
    0b00100,
    0b00110,
    0b11111,
    0b11111,
    0b00110,
    0b00100,
    0b00000
};

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
    , lastTgtArrow(false)
    , lastActArrow(false)
    , lastZDelta(-999.0f)
    , lastTempCorrectionFactor(-1.0f)
    , lastUncorrectedFast(-1.0f)
    , lastAdjustedVoltage(-1.0f)
    , forceRedraw(true)  // draw the static frame on the very first update
{
}

void DisplayManager::begin() {
    lcd.init();
    lcd.begin(LCD_COLUMNS, LCD_ROWS);
    lcd.backlight();

    createCustomCharacters();
}

void DisplayManager::createCustomCharacters() {
    lcd.createChar(CHAR_ARROW_RIGHT, arrowRight);
}

void DisplayManager::update(unsigned long currentTime, int currentScreen,
                            THCController* thc, SpeedMonitor* speedMonitor,
                            float tempCorrectionFactor) {
    // Update Anti-Dive message state
    updateAntiDiveDisplay(currentTime);

    // Screen change
    if (currentScreen != lastScreen) {
        lcd.clear();
        lastScreen = currentScreen;
        resetCachedValues();
        forceRedraw = true;  // ensure static labels get drawn even when their cached state already matches the idle defaults
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

    forceRedraw = false;  // static frame is now on screen; revert to change-gated refresh
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

void DisplayManager::drawAntiDiveMessage() {
    lcd.setCursor(12, 0);
    if (blinkVisible) {
        lcd.print("ADIV");
    } else {
        lcd.print("    ");
    }
}

void DisplayManager::drawScreen0(THCController* thc, SpeedMonitor* speedMonitor) {
    // THC status drives the two label arrows:
    //   Tgt→ = THC engaged (owns Z, all gates cleared)
    //   Act→ = THC actively correcting (PID output driving the stepper)
    // When idle both stay ':'. This encodes three distinct states in the
    // labels themselves, freeing the old icon column for the Z-delta readout.
    bool thcActive = thc->isTHCActive();
    bool correcting = thcActive && (fabs(thc->getPidOutput()) > 10.0);

    // --- Line 0: "Act[→|:] XXX.X V SPD" ---
    // Redraw the "Act" label separator when its state changes. We use the
    // custom CHAR_ARROW_RIGHT glyph rather than the HD44780 ROM arrow because
    // the ROM glyph at 0x7E differs between A00 and A02 character ROMs (left
    // arrow on A00), and I2C backpacks commonly ship with A00. The custom
    // glyph is ROM-independent.
    if (forceRedraw || correcting != lastActArrow) {
        lcd.setCursor(0, 0);
        lcd.print("Act");
        lcd.write(correcting ? CHAR_ARROW_RIGHT : (byte)':');
        lastActArrow = correcting;
    }

    float actualVoltage = thc->getFastVoltage();
    if (forceRedraw || actualVoltage != lastActualVoltage) {
        lcd.setCursor(4, 0);
        char vBuf[8];
        dtostrf(actualVoltage, 5, 1, vBuf);  // Fixed width 5 chars: "  9.4" or "123.4"
        lcd.print(vBuf);
        lcd.print("V ");
        lastActualVoltage = actualVoltage;
    }

    // Speed zone or Anti-Dive message
    if (adState != AD_DISPLAY_IDLE) {
        drawAntiDiveMessage();
    } else {
        // Normal speed display — top-right 4-digit field, clamped to 9999
        int displayedSpeed = (int)(speedMonitor->getFilteredSpeed());
        if (displayedSpeed < 0) displayedSpeed = 0;
        if (displayedSpeed > 9999) displayedSpeed = 9999;

        if (forceRedraw || displayedSpeed != lastSpeed) {
            lcd.setCursor(12, 0);
            char buffer[8];
            snprintf(buffer, sizeof(buffer), "%4d", displayedSpeed);
            lcd.print(buffer);
            lastSpeed = displayedSpeed;
        }
    }

    // --- Line 1: "Tgt[→|:] XXX.X V Z[+1.2|----]" ---
    if (forceRedraw || thcActive != lastTgtArrow) {
        lcd.setCursor(0, 1);
        lcd.print("Tgt");
        lcd.write(thcActive ? CHAR_ARROW_RIGHT : (byte)':');
        lastTgtArrow = thcActive;
    }

    float setpoint = thc->getSetpoint();
    if (forceRedraw || setpoint != lastSetpoint) {
        lcd.setCursor(4, 1);
        char sBuf[8];
        dtostrf(setpoint, 5, 1, sBuf);  // Fixed width 5 chars
        lcd.print(sBuf);
        lcd.print("V");
        lastSetpoint = setpoint;
    }

    // Z delta from cut-start height. 6 chars: " Z+1.2" / " Z-0.3" / " Z----"
    // when THC is idle (no reference yet). Clamped to +/-9.9 so the field
    // never overflows the 16-column line.
    float zDelta = thcActive ? thc->getZDeltaMm() : 0.0f;
    bool drawZ = thcActive;
    bool zChanged = (zDelta != lastZDelta);
    bool zStateChanged = (drawZ != (lastZDelta > -900.0f));

    if (forceRedraw || zStateChanged || (drawZ && zChanged)) {
        lcd.setCursor(10, 1);
        if (drawZ) {
            if (zDelta > 9.9f) zDelta = 9.9f;
            if (zDelta < -9.9f) zDelta = -9.9f;
            char zBuf[8];
            // " Z" + sign + digit.digit  =>  e.g. " Z+1.2"  (6 chars total)
            snprintf(zBuf, sizeof(zBuf), " Z%+.1f", (double)zDelta);
            lcd.print(zBuf);
            lastZDelta = zDelta;
        } else {
            lcd.print(" Z----");
            lastZDelta = -999.0f;  // sentinel = "idle, no reference"
        }
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
    float cutSpeed = speedMonitor->getCutSpeed();
    if (cutSpeed < 0.0f) cutSpeed = 0.0f;
    if (cutSpeed > 9999.0f) cutSpeed = 9999.0f;
    char buffer[8];
    snprintf(buffer, sizeof(buffer), "%4.0f", cutSpeed);
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

void DisplayManager::resetCachedValues() {
    lastActualVoltage = -1.0f;
    lastSetpoint = -1.0f;
    lastSpeed = -1;
    lastTgtArrow = false;
    lastActArrow = false;
    lastZDelta = -999.0f;
    lastTempCorrectionFactor = -1.0f;
    lastUncorrectedFast = -1.0f;
    lastAdjustedVoltage = -1.0f;
}
