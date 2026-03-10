/**
 * SmartTHC - KY-040 Encoder Manager
 *
 * Manages the rotary encoder and button with robust debouncing
 * Fix for spurious click during rotation
 */

#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include <Arduino.h>
#include "Config.h"

// Button state machine states
enum ButtonState {
    BTN_IDLE,           // Nothing happening
    BTN_PRESSED,        // Press detected, awaiting confirmation
    BTN_HELD,           // Press confirmed
    BTN_RELEASED        // Release detected, click validated
};

class EncoderManager {
public:
    EncoderManager();

    // Initialization
    void begin();

    // Update - call in loop()
    void update();

    // Encoder position reading
    int getPosition() const { return encoderPos; }
    void resetPosition() { encoderPos = 0; lastEncoderPos = 0; }

    // Delta since last call
    int getDelta();

    // Click detection (clean falling edge)
    bool isButtonClicked();

    // Raw states (for debug)
    bool isButtonPressed() const { return buttonState == BTN_HELD; }
    bool isRotating() const { return rotationActive; }

    // Configuration
    void setDebounceTime(unsigned long ms) { debounceTime = ms; }
    void setLockoutTime(unsigned long ms) { lockoutTime = ms; }
    void setRotationTimeout(unsigned long ms) { rotationTimeout = ms; }

private:
    // Encoder state
    volatile int encoderPos;
    int lastEncoderPos;
    int prevCLK;
    int prevDT;

    // Button state (state machine)
    ButtonState buttonState;
    bool lastButtonReading;
    unsigned long buttonStateTime;
    unsigned long lastClickTime;
    bool clickPending;

    // Rotation detection
    bool rotationActive;
    unsigned long lastRotationTime;
    int lastRotationDelta;

    // Debounce parameters
    unsigned long debounceTime;     // Press/release confirmation time
    unsigned long lockoutTime;      // Lockout after click
    unsigned long rotationTimeout;  // Time before end of rotation

    // Internal methods
    void readEncoder();
    void updateButtonState(unsigned long currentTime);
    bool readButtonPin();
};

#endif // ENCODER_MANAGER_H
