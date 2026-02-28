/**
 * SmartTHC - Gestionnaire d'encodeur KY-040
 * 
 * Implémentation avec anti-rebond robuste pour éviter les clics intempestifs
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
    , debounceTime(30)      // 30ms pour confirmer appui/relâchement
    , lockoutTime(250)      // 250ms de verrouillage après clic
    , rotationTimeout(150)  // 150ms sans rotation = fin de rotation
{
}

void EncoderManager::begin() {
    // Configuration des pins
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Lecture initiale
    prevCLK = digitalRead(ENCODER_PIN_A);
    prevDT = digitalRead(ENCODER_PIN_B);
    lastButtonReading = readButtonPin();
}

void EncoderManager::update() {
    unsigned long currentTime = millis();
    
    // Lecture de l'encodeur rotatif
    readEncoder();
    
    // Détection d'activité de rotation
    int currentDelta = encoderPos - lastEncoderPos;
    if (currentDelta != 0) {
        rotationActive = true;
        lastRotationTime = currentTime;
        lastRotationDelta = currentDelta;
    } else if (rotationActive && (currentTime - lastRotationTime > rotationTimeout)) {
        // Fin de rotation après timeout
        rotationActive = false;
    }
    
    // Mise à jour de la machine à états du bouton
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
    
    // Détection de front sur CLK
    if (currentCLK != prevCLK) {
        // Algo de lecture KY-040 standard
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
    // Le bouton est actif LOW (pull-up)
    return digitalRead(BUTTON_PIN) == LOW;
}

void EncoderManager::updateButtonState(unsigned long currentTime) {
    bool buttonReading = readButtonPin();
    
    switch (buttonState) {
        case BTN_IDLE:
            // Attente d'un appui
            if (buttonReading && !lastButtonReading) {
                // Front descendant détecté
                buttonState = BTN_PRESSED;
                buttonStateTime = currentTime;
            }
            break;
            
        case BTN_PRESSED:
            // Confirmation de l'appui (anti-rebond)
            if (!buttonReading) {
                // Retour à HIGH trop tôt = rebond
                buttonState = BTN_IDLE;
            } else if (currentTime - buttonStateTime >= debounceTime) {
                // Appui confirmé après debounce
                buttonState = BTN_HELD;
            }
            break;
            
        case BTN_HELD:
            // Attente du relâchement
            if (!buttonReading) {
                // Front montant détecté
                buttonState = BTN_RELEASED;
                buttonStateTime = currentTime;
            }
            break;
            
        case BTN_RELEASED:
            // Confirmation du relâchement (anti-rebond)
            if (buttonReading) {
                // Retour à LOW trop tôt = rebond
                buttonState = BTN_HELD;
            } else if (currentTime - buttonStateTime >= debounceTime) {
                // Relâchement confirmé = CLIC VALIDÉ
                
                // Vérifier le verrouillage après dernier clic
                if (currentTime - lastClickTime >= lockoutTime) {
                    // Ignorer le clic si on est en pleine rotation (sécurité KY-040)
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
