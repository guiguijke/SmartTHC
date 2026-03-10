/**
 * SmartTHC - KY-040 Encoder Manager
 *
 * Implementation with robust debouncing to prevent spurious clicks
 */

#include "EncoderManager.h"

EncoderManager::EncoderManager()
    : encoderPos(0)
    , lastEncoderPos(0)
    , prevCLK(HIGH)
    , prevDT(HIGH)
    , buttonState(BTN_IDLE)
    , lastButtonReading(HIGH)
    , buttonStateTime(0)
    , lastClickTime(0)
    , clickPending(false)
    , rotationActive(false)
    , lastRotationTime(0)
    , lastRotationDelta(0)
    , debounceTime(30)      // 30ms to confirm press/release
    , lockoutTime(250)      // 250ms lockout after click
    , rotationTimeout(150)  // 150ms without rotation = end of rotation
{
}

void EncoderManager::begin() {
    // Pin configuration
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Initial reading
    prevCLK = digitalRead(ENCODER_PIN_A);
    prevDT = digitalRead(ENCODER_PIN_B);
    lastButtonReading = readButtonPin();
}

void EncoderManager::update() {
    unsigned long currentTime = millis();

    // Read rotary encoder
    readEncoder();

    // Rotation activity detection
    int currentDelta = encoderPos - lastEncoderPos;
    if (currentDelta != 0) {
        rotationActive = true;
        lastRotationTime = currentTime;
        lastRotationDelta = currentDelta;
    } else if (rotationActive && (currentTime - lastRotationTime > rotationTimeout)) {
        // End of rotation after timeout
        rotationActive = false;
    }

    // Update button state machine
    updateButtonState(currentTime);
}

int EncoderManager::getDelta() {
    int delta = encoderPos - lastEncoderPos;
    lastEncoderPos = encoderPos;
    return delta;
}

bool EncoderManager::isButtonClicked() {
    if (clickPending) {
        clickPending = false;
        return true;
    }
    return false;
}

void EncoderManager::readEncoder() {
    int currentCLK = digitalRead(ENCODER_PIN_A);
    int currentDT = digitalRead(ENCODER_PIN_B);

    // Edge detection on CLK
    if (currentCLK != prevCLK) {
        // Standard KY-040 reading algorithm
        if (currentCLK == HIGH) {
            if (currentDT == LOW && prevDT == HIGH) {
                encoderPos++;
            } else if (currentDT == HIGH && prevDT == LOW) {
                encoderPos--;
            }
        }
        prevCLK = currentCLK;
        prevDT = currentDT;
    }
}

bool EncoderManager::readButtonPin() {
    // Button is active LOW (pull-up)
    return digitalRead(BUTTON_PIN) == LOW;
}

void EncoderManager::updateButtonState(unsigned long currentTime) {
    bool buttonReading = readButtonPin();

    switch (buttonState) {
        case BTN_IDLE:
            // Waiting for press
            if (buttonReading && !lastButtonReading) {
                // Falling edge detected
                buttonState = BTN_PRESSED;
                buttonStateTime = currentTime;
            }
            break;

        case BTN_PRESSED:
            // Press confirmation (debounce)
            if (!buttonReading) {
                // Returned HIGH too soon = bounce
                buttonState = BTN_IDLE;
            } else if (currentTime - buttonStateTime >= debounceTime) {
                // Press confirmed after debounce
                buttonState = BTN_HELD;
            }
            break;

        case BTN_HELD:
            // Waiting for release
            if (!buttonReading) {
                // Rising edge detected
                buttonState = BTN_RELEASED;
                buttonStateTime = currentTime;
            }
            break;

        case BTN_RELEASED:
            // Release confirmation (debounce)
            if (buttonReading) {
                // Returned LOW too soon = bounce
                buttonState = BTN_HELD;
            } else if (currentTime - buttonStateTime >= debounceTime) {
                // Release confirmed = CLICK VALIDATED

                // Check lockout since last click
                if (currentTime - lastClickTime >= lockoutTime) {
                    // Ignore click if currently rotating (KY-040 safety)
                    if (!rotationActive) {
                        clickPending = true;
                        lastClickTime = currentTime;
                    }
                }
                buttonState = BTN_IDLE;
            }
            break;
    }

    lastButtonReading = buttonReading;
}
