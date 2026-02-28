/**
 * SmartTHC - Gestionnaire d'encodeur KY-040
 * 
 * Gère l'encodeur rotatif et son bouton avec anti-rebond robuste
 * Fix pour le problème de clic intempestif lors de la rotation
 */

#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include <Arduino.h>
#include "Config.h"

// États de la machine à états du bouton
enum ButtonState {
    BTN_IDLE,           // Rien ne se passe
    BTN_PRESSED,        // Appui détecté, attente confirmation
    BTN_HELD,           // Appui confirmé
    BTN_RELEASED        // Relâchement détecté, clic validé
};

class EncoderManager {
public:
    EncoderManager();
    
    // Initialisation
    void begin();
    
    // Mise à jour - à appeler dans loop()
    void update();
    
    // Lecture de la position de l'encodeur
    int getPosition() const { return encoderPos; }
    void resetPosition() { encoderPos = 0; lastEncoderPos = 0; }
    
    // Lecture du delta depuis le dernier appel
    int getDelta();
    
    // Détection d'un clic (front descendant propre)
    bool isButtonClicked();
    
    // États bruts (pour debug)
    bool isButtonPressed() const { return buttonState == BTN_HELD; }
    bool isRotating() const { return rotationActive; }
    
    // Configuration
    void setDebounceTime(unsigned long ms) { debounceTime = ms; }
    void setLockoutTime(unsigned long ms) { lockoutTime = ms; }
    void setRotationTimeout(unsigned long ms) { rotationTimeout = ms; }

private:
    // État de l'encodeur
    volatile int encoderPos;
    int lastEncoderPos;
    int prevCLK;
    int prevDT;
    
    // État du bouton (machine à états)
    ButtonState buttonState;
    bool lastButtonReading;
    unsigned long buttonStateTime;
    unsigned long lastClickTime;
    bool clickPending;
    
    // Détection rotation
    bool rotationActive;
    unsigned long lastRotationTime;
    int lastRotationDelta;
    
    // Paramètres anti-rebond
    unsigned long debounceTime;     // Temps de confirmation appui/relâchement
    unsigned long lockoutTime;      // Verrouillage après clic
    unsigned long rotationTimeout;  // Temps avant fin de rotation
    
    // Méthodes internes
    void readEncoder();
    void updateButtonState(unsigned long currentTime);
    bool readButtonPin();
};

#endif // ENCODER_MANAGER_H
