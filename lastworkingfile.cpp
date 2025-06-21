#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <PID_v1.h>

#TEST

// Define pins (adjusted for UNO R4 Minima)
#define PLASMA_PIN A0    // Analog pin for plasma signal
#define ENABLE_PIN 10
#define PIN_2 9
#define PIN_3 8
#define DIR_PIN 7
#define STEP_PIN 6
#define STEP_X_PIN 2     // Interrupt pin (INT0)
#define STEP_Y_PIN 3     // Interrupt pin (INT1)
#define ENCODER_PIN_A 4  // Changed to avoid interrupt conflicts
#define ENCODER_PIN_B 5
#define BUTTON_PIN 11

// Initialize objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Tension filtering variables
float tension_reelle_filtre = 0.0;
const float ALPHA_EWMA = 0.3;
bool ewma_initialized = false;

// Speed filtering variables
const int taille_filtre_vitesse = 2;
float lectures_vitesse[taille_filtre_vitesse];
int index_lecture_vitesse = 0;
float somme_lectures_vitesse = 0.0;
float vitesse_torche_filtre = 0.0;
float vitesse_test = 0; // For testing
float plasma_test_V = 0; // For testing

// Step counting variables
volatile unsigned long stepCountX = 0;
volatile unsigned long stepCountY = 0;
unsigned long totalStepX = 0;
unsigned long totalStepY = 0;
const unsigned long SPEED_INTERVAL = 50;
unsigned long lastSpeedTime = 0;
const float DIST_PER_STEP_X = 0.0025;
const float DIST_PER_STEP_Y = 0.0025;
const unsigned long MIN_STEPS = 20;
unsigned long lastStepTime = 0;
const unsigned long STEP_TIMEOUT = 500;
volatile unsigned long lastStepXTime = 0;
volatile unsigned long lastStepYTime = 0;
const unsigned long DEBOUNCE_US = 3;
volatile unsigned long pulseTimeX = 0;
volatile unsigned long pulseTimeY = 0;
volatile unsigned long bounceCountX = 0;
volatile unsigned long bounceCountY = 0;
const float SPEED_CORRECTION = 1.0;

// PID variables
double Setpoint = 100.0; // Initial target voltage 100V
double Input = 0;
double Output = 0;
double Kp = 4.0;
double Ki = 0.15;
double Kd = 0.2;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Speed thresholds and correction factor
float cut_speed = 2500.0;
float threshold_ratio = 0.7;
float threshold_speed = cut_speed * threshold_ratio;
float tension_correction_factor = 1.0;

// Display and logging variables
unsigned long dernier_affichage = 0;
const unsigned long intervalle_affichage = 250;
const float seuil_arc = 10.0;
const unsigned long delai_stabilisation = 1000;
unsigned long temps_plasma_active = 0;
bool plasma_stabilise = false;
const unsigned long LOG_INTERVAL = 100;
unsigned long lastLogTime = 0;
bool thc_actif = false;

// Encoder and screen navigation
volatile int encoderPos = 0;
int currentScreen = 0;
const int NB_SCREENS = 8;
int prevCLK;
int prevDT;

// Custom LCD characters
byte enableChar[8] = {0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b00000};
byte plasmaChar[8] = {0b11111, 0b11111, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
byte thcActifChar[8] = {0b11111, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111, 0b00000};
byte arrowUp[8] = {0b00100, 0b01110, 0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00000};
byte arrowDown[8] = {0b00100, 0b00100, 0b00100, 0b00100, 0b11111, 0b01110, 0b00100, 0b00000};
byte stableChar[8] = {0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000};
byte enableall[8] = {0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b00000};

// Variables globales pour la mesure du temps
unsigned long loopStartTime;
unsigned long loopEndTime;
unsigned long lastLoopLogTime = 0;
const unsigned long LOOP_LOG_INTERVAL = 10000;
unsigned long loopExecutionTimeSum = 0;
unsigned int loopCount = 0;

// Function declarations
void calculateSpeed();
void readAndFilterTension();
void managePlasmaAndTHC();
void updateLCD();
void doEncoder();
void countStepX();
void countStepY();

void setup() {
  pinMode(PLASMA_PIN, INPUT);
  pinMode(ENABLE_PIN, INPUT_PULLUP);
  pinMode(PIN_2, OUTPUT);
  pinMode(PIN_3, OUTPUT);
  pinMode(STEP_X_PIN, INPUT);
  pinMode(STEP_Y_PIN, INPUT);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(PIN_2, HIGH);
  digitalWrite(PIN_3, HIGH);

  // Set ADC resolution to 14 bits
  analogReadResolution(14);

  Serial.begin(115200);

  // Initialisation des états précédents pour l'encodeur
  prevCLK = digitalRead(ENCODER_PIN_A);
  prevDT = digitalRead(ENCODER_PIN_B);

  // Interrupt setup (UNO R4 Minima supports interrupts on pins 2 and 3)
  attachInterrupt(digitalPinToInterrupt(STEP_X_PIN), countStepX, FALLING);
  attachInterrupt(digitalPinToInterrupt(STEP_Y_PIN), countStepY, FALLING);
  // Encoder interrupt on pin 4 (not a hardware interrupt, handled in loop)
  
  lcd.init();
  lcd.begin(16, 2);
  lcd.backlight();

  lcd.createChar(0, enableChar);
  lcd.createChar(1, plasmaChar);
  lcd.createChar(2, thcActifChar);
  lcd.createChar(3, arrowUp);
  lcd.createChar(4, arrowDown);
  lcd.createChar(5, stableChar);
  lcd.createChar(6, enableall);

  for (int i = 0; i < taille_filtre_vitesse; i++) {
    lectures_vitesse[i] = 0.0;
  }

  // Configure AccelStepper
  stepper.setMaxSpeed(16000); // Steps per second
  stepper.setAcceleration(8000); // Steps per second^2

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-1000, 1000);
}

void loop() {
  loopStartTime = micros();
  unsigned long currentTime = millis();

  // Gestion du bouton pour changer d'écran
  static bool lastButtonState = HIGH;
  static unsigned long lastButtonTime = 0;
  bool buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH && currentTime - lastButtonTime > 50) {
    currentScreen = (currentScreen + 1) % NB_SCREENS;
    lastButtonTime = currentTime;
  }
  lastButtonState = buttonState;

  // Gestion de l'encodeur (polling instead of interrupt)
  int currentCLK = digitalRead(ENCODER_PIN_A);
  if (currentCLK != prevCLK) {
    doEncoder();
  }
  prevCLK = currentCLK;

  // Gestion des changements de l'encodeur
  static int lastEncoderPos = 0;
  
  int currentEncoderPos = encoderPos;
  
  int delta = currentEncoderPos - lastEncoderPos;
  lastEncoderPos = currentEncoderPos;

  // Gestion des écrans et ajustements des paramètres
  switch (currentScreen) {
    case 1: // Tension cible
      Setpoint += delta;
      if (Setpoint < 50) Setpoint = 50;
      if (Setpoint > 200) Setpoint = 200;
      break;
    case 2: // Facteur de correction de tension
      tension_correction_factor += delta * 0.01;
      break;
    case 3: // Vitesse de coupe
      cut_speed += delta * 100;
      if (cut_speed < 0) cut_speed = 0;
      threshold_speed = cut_speed * threshold_ratio;
      break;
    case 4: // Seuil de vitesse
      threshold_ratio += delta * 0.1;
      threshold_speed = cut_speed * threshold_ratio;
      break;
    case 5: // PID Kp
      Kp += delta * 0.1;
      if (Kp < 0) Kp = 0;
      myPID.SetTunings(Kp, Ki, Kd);
      break;
    case 6: // PID Ki
      Ki += delta * 0.01;
      if (Ki < 0) Ki = 0;
      myPID.SetTunings(Kp, Ki, Kd);
      break;
    case 7: // PID Kd
      Kd += delta * 0.01;
      if (Kd < 0) Kd = 0;
      myPID.SetTunings(Kp, Ki, Kd);
      break;
  }

  if (currentTime - lastSpeedTime >= SPEED_INTERVAL) {
    calculateSpeed();
  }

  if (currentTime - lastLogTime >= LOG_INTERVAL) {
    bool plasma_pin_low = (analogRead(PLASMA_PIN) > 100); // Adjust threshold as needed
    
    if (plasma_pin_low) {
      Serial.print("Vitesse torche: ");
      Serial.print(vitesse_torche_filtre);
      Serial.println(" mm/min");
      Serial.print("PLASMA_PIN: LOW | ENABLE_PIN: ");
      Serial.print(digitalRead(ENABLE_PIN) == LOW ? "LOW" : "HIGH");
      Serial.print(" | Stabilisation: ");
      Serial.print(plasma_stabilise ? "OK" : "En attente");
      Serial.print(" | Arc détecté: ");
      Serial.print(tension_reelle_filtre > seuil_arc ? "Oui" : "Non");
      Serial.print(" | THC actif: ");
      Serial.println(thc_actif ? "Oui" : "Non");
      Serial.print("Tension mesurée: ");
      Serial.print(tension_reelle_filtre);
      Serial.print(" V | Tension cible: ");
      Serial.print(Setpoint);
      Serial.print(" V | PID Output: ");
      Serial.println(Output);
    } else {
      Serial.print("Tension mesurée: ");
      Serial.print(tension_reelle_filtre);
      Serial.print(" V | PLASMA_PIN: HIGH");
      Serial.println();
    }
    
    lastLogTime = currentTime;
  }
  managePlasmaAndTHC();
  readAndFilterTension();

  if (currentTime - dernier_affichage >= intervalle_affichage) {
    updateLCD();
    dernier_affichage = currentTime;
  }

  // Run stepper
  stepper.run();

  loopEndTime = micros();
  unsigned long loopExecutionTime = loopEndTime - loopStartTime;
  loopExecutionTimeSum += loopExecutionTime;
  loopCount++;

  if (currentTime - lastLoopLogTime >= LOOP_LOG_INTERVAL) {
    if (loopCount > 0) {
      unsigned long averageLoopTime = loopExecutionTimeSum / loopCount;
      float loopFrequency = 1000000.0 / averageLoopTime;
      Serial.print("Temps d'exécution moyen: ");
      Serial.print(averageLoopTime);
      Serial.print(" us | Fréquence: ");
      Serial.print(loopFrequency, 1);
      Serial.println(" Hz");
    }
    loopExecutionTimeSum = 0;
    loopCount = 0;
    lastLoopLogTime = currentTime;
  }
}

void doEncoder() {
  int currentCLK = digitalRead(ENCODER_PIN_A);
  int currentDT = digitalRead(ENCODER_PIN_B);

  if (currentCLK == HIGH && currentDT == LOW && prevCLK == LOW && prevDT == HIGH) {
    encoderPos++;
  }
  if (currentCLK == HIGH && currentDT == HIGH && prevCLK == LOW && prevDT == LOW) {
    encoderPos--;
  }
  prevCLK = currentCLK;
  prevDT = currentDT;
}

void calculateSpeed() {
  unsigned long currentTime = millis();
  noInterrupts();
  unsigned long tempX = stepCountX;
  unsigned long tempY = stepCountY;
  stepCountX = 0;
  stepCountY = 0;
  interrupts();

  totalStepX += tempX;
  totalStepY += tempY;

  float vitesse_torche = 0.0;
  unsigned long deltaTime = currentTime - lastSpeedTime;
  if (tempX + tempY >= MIN_STEPS && deltaTime > 0) {
    float distanceX = tempX * DIST_PER_STEP_X;
    float distanceY = tempY * DIST_PER_STEP_Y;
    float totalDistance = sqrt(distanceX * distanceX + distanceY * distanceY);
    vitesse_torche = (totalDistance / (deltaTime / 1000.0)) * 60.0 * SPEED_CORRECTION;
  }

  somme_lectures_vitesse -= lectures_vitesse[index_lecture_vitesse];
  lectures_vitesse[index_lecture_vitesse] = vitesse_torche;
  somme_lectures_vitesse += vitesse_torche;
  index_lecture_vitesse = (index_lecture_vitesse + 1) % taille_filtre_vitesse;
  vitesse_torche_filtre = somme_lectures_vitesse / taille_filtre_vitesse;
  if (vitesse_torche_filtre < 0) vitesse_torche_filtre = 0.0;

  if (tempX + tempY == 0 && currentTime - lastStepTime > STEP_TIMEOUT) {
    totalStepX = 0;
    totalStepY = 0;
    for (int i = 0; i < taille_filtre_vitesse; i++) {
      lectures_vitesse[i] = 0.0;
    }
    somme_lectures_vitesse = 0.0;
    vitesse_torche_filtre = 0.0 + vitesse_test;
  }
  if (tempX + tempY > 0) lastStepTime = currentTime;

  lastSpeedTime = currentTime;
}

void readAndFilterTension() {
  int adc0 = analogRead(PLASMA_PIN); // 0-16383 (14-bit)
  // Convert ADC to voltage (adjust scaling based on your circuit)
  float tension_reelle = (adc0 / 16383.0) * 5.0 * 16.042 * tension_correction_factor + plasma_test_V; // Example scaling

  if (!ewma_initialized) {
    tension_reelle_filtre = tension_reelle;
    ewma_initialized = true;
  } else {
    tension_reelle_filtre = ALPHA_EWMA * tension_reelle + (1.0 - ALPHA_EWMA) * tension_reelle_filtre;
  }

  Input = tension_reelle_filtre;
}

void managePlasmaAndTHC() {
  bool plasma_pin_low = (analogRead(PLASMA_PIN) > 100); // Adjust threshold
  bool enable_pin_low = (digitalRead(ENABLE_PIN) == LOW);
  bool arc_detecte = (tension_reelle_filtre > seuil_arc);

  if (plasma_pin_low) {
    digitalWrite(PIN_2, LOW);
    digitalWrite(PIN_3, LOW);
  } else {
    digitalWrite(PIN_2, HIGH);
    digitalWrite(PIN_3, HIGH);
  }

  if (plasma_pin_low && !plasma_stabilise) {
    if (temps_plasma_active == 0) temps_plasma_active = millis();
    if (millis() - temps_plasma_active >= delai_stabilisation) plasma_stabilise = true;
  } else if (!plasma_pin_low) {
    temps_plasma_active = 0;
    plasma_stabilise = false;
  }

  static bool thc_etat = false;
  if (!thc_etat && vitesse_torche_filtre >= threshold_speed) {
    thc_etat = true;
  } else if (thc_etat && vitesse_torche_filtre < threshold_speed) {
    thc_etat = false;
  }
  thc_actif = enable_pin_low && plasma_pin_low && plasma_stabilise && arc_detecte && thc_etat;

  myPID.Compute();
  if (thc_actif) {
    double error = Setpoint - Input;
    if (abs(error) > 1.0) {
      stepper.setSpeed(Output); // Set speed based on PID output
      stepper.move(Output > 0 ? 1000 : -1000); // Move in direction
    } else {
      stepper.setSpeed(0);
      stepper.stop();
    }
  } else {
    stepper.setSpeed(0);
    stepper.stop();
  }
}

void updateLCD() {
  // Same as original, no changes needed
  static int lastScreen = -1;
  static float lastTensionReelle = -1;
  static float lastSetpoint = -1;
  static int lastVitesse = -1;
  static bool lastThcActif = false;
  static double lastOutput = 0;
  static bool lastEnableLow = false;
  static bool lastPlasmaLow = false;

  if (currentScreen != lastScreen) {
    lcd.clear();
    switch (currentScreen) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("TRC:     V");
        lcd.setCursor(0, 1);
        lcd.print("TGT:     V");
        break;
      case 1:
        lcd.setCursor(0, 0);
        lcd.print("Set V:      V");
        break;
      case 2:
        lcd.setCursor(0, 0);
        lcd.print("V factor:   ");
        break;
      case 3:
        lcd.setCursor(0, 0);
        lcd.print("CutSpd:     ");
        break;
      case 4:
        lcd.setCursor(0, 0);
        lcd.print("CutSpdThs %:");
        break;
      case 5:
        lcd.setCursor(0, 0);
        lcd.print("PID Kp:     ");
        break;
      case 6:
        lcd.setCursor(0, 0);
        lcd.print("PID Ki:     ");
        break;
      case 7:
        lcd.setCursor(0, 0);
        lcd.print("PID Kd:     ");
        break;
    }
    lastScreen = currentScreen;
    lastTensionReelle = -1;
    lastSetpoint = -1;
    lastVitesse = -1;
    lastThcActif = false;
    lastOutput = 0;
    lastEnableLow = false;
    lastPlasmaLow = false;
  }

  if (currentScreen == 0) {
    if (tension_reelle_filtre != lastTensionReelle) {
      lcd.setCursor(4, 0);
      lcd.print(tension_reelle_filtre, 1);
      lastTensionReelle = tension_reelle_filtre;
    }

    if (Setpoint != lastSetpoint) {
      lcd.setCursor(4, 1);
      lcd.print(Setpoint, 1);
      lastSetpoint = Setpoint;
    }

    int vitesse_affichee = (int)(vitesse_torche_filtre);
    if (vitesse_affichee > 9999) vitesse_affichee = 9999;
    if (vitesse_torche_filtre < 0.1) {
      if (lastVitesse != 0) {
        lcd.setCursor(12, 0);
        lcd.print("   0");
        lastVitesse = 0;
      }
    } else if (vitesse_affichee != lastVitesse) {
      lcd.setCursor(12, 0);
      lcd.print(vitesse_affichee);
      lastVitesse = vitesse_affichee;
    }

    bool enable_low = digitalRead(ENABLE_PIN) == LOW;
    bool plasma_low = analogRead(PLASMA_PIN) > 100;
    if (thc_actif != lastThcActif || enable_low != lastEnableLow || plasma_low != lastPlasmaLow) {
      lcd.setCursor(11, 1);
      if (thc_actif) {
        lcd.write(2);
      } else if (enable_low && plasma_low) {
        lcd.write(6);
      } else if (enable_low) {
        lcd.write(0);
      } else if (plasma_low) {
        lcd.write(1);
      } else {
        lcd.print(" ");
      }
      lastThcActif = thc_actif;
      lastEnableLow = enable_low;
      lastPlasmaLow = plasma_low;
    }

    if (thc_actif) {
      lcd.setCursor(13, 1);
      if (Output > 10) {
        lcd.write(3);
      } else if (Output < -10) {
        lcd.write(4);
      } else {
        lcd.write(5);
      }
      int intensity = map(abs(Output), 0, 1000, 0, 2);
      lcd.setCursor(14, 1);
      for (int i = 0; i < intensity; i++) {
        lcd.print("|");
      }
      for (int i = intensity; i < 2; i++) {
        lcd.print(" ");
      }
    } else if (lastOutput != 0) {
      lcd.setCursor(13, 1);
      lcd.print("   ");
    }
    lastOutput = Output;
  } else {
    switch (currentScreen) {
      case 1:
        lcd.setCursor(7, 0);
        lcd.print(Setpoint, 1);
        break;
      case 2:
        lcd.setCursor(10, 0);
        lcd.print(tension_correction_factor, 2);
        break;
      case 3:
        lcd.setCursor(8, 0);
        lcd.print(cut_speed, 0);
        break;
      case 4:
        lcd.setCursor(12, 0);
        lcd.print(threshold_ratio, 1);
        break;
      case 5:
        lcd.setCursor(8, 0);
        lcd.print(Kp, 1);
        break;
      case 6:
        lcd.setCursor(8, 0);
        lcd.print(Ki, 2);
        break;
      case 7:
        lcd.setCursor(8, 0);
        lcd.print(Kd, 2);
        break;
    }
  }
}

void countStepX() {
  unsigned long currentTime = micros();
  if (currentTime - lastStepXTime >= DEBOUNCE_US) {
    stepCountX++;
    pulseTimeX = currentTime;
    lastStepXTime = currentTime;
  } else bounceCountX++;
}

void countStepY() {
  unsigned long currentTime = micros();
  if (currentTime - lastStepYTime >= DEBOUNCE_US) {
    stepCountY++;
    pulseTimeY = currentTime;
    lastStepYTime = currentTime;
  } else bounceCountY++;
}

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <ArduPID.h>

// Define pins for Arduino Uno R4 Minima
#define PLASMA_PIN 12  // Input pin for plasma arc OK signal
#define ENABLE_PIN 10  // Changé pour éviter conflit avec SWITCH2
#define SWITCH1 9      // Anciennement PIN_2
#define SWITCH2 13     // Anciennement PIN_3
#define DIR_PIN 8
#define STEP_PIN 6
#define STEP_X_PIN 2   // Interrupt pin
#define STEP_Y_PIN 3   // Interrupt pin
#define ENCODER_PIN_A 4 // Encoder CLK
#define ENCODER_PIN_B 5 // Encoder DT
#define BUTTON_PIN 7    // Encoder SW
#define PLASMA_VOLTAGE A0 // ADC input for plasma voltage


// Initialize objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);


// Speed filtering variables
const int taille_filtre_vitesse = 2;
float lectures_vitesse[taille_filtre_vitesse];
int index_lecture_vitesse = 0;
float somme_lectures_vitesse = 0.0;
float vitesse_torche_filtre = 0.0;
float vitesse_test = 0; // Pour tests
float plasma_test_V = 0; // Pour tests

// Step counting variables
volatile unsigned long stepCountX = 0;
volatile unsigned long stepCountY = 0;
unsigned long totalStepX = 0;
unsigned long totalStepY = 0;
const unsigned long SPEED_INTERVAL = 50;
unsigned long lastSpeedTime = 0;
const float DIST_PER_STEP_X = 0.0025;
const float DIST_PER_STEP_Y = 0.0025;
const unsigned long MIN_STEPS = 20;
unsigned long lastStepTime = 0;
const unsigned long STEP_TIMEOUT = 500;
volatile unsigned long lastStepXTime = 0;
volatile unsigned long lastStepYTime = 0;
const unsigned long DEBOUNCE_US = 3;
volatile unsigned long pulseTimeX = 0;
volatile unsigned long pulseTimeY = 0;
volatile unsigned long bounceCountX = 0;
volatile unsigned long bounceCountY = 0;
const float SPEED_CORRECTION = 1.0;

// PID variables
double Setpoint = 100.0;  // Initial target voltage 100V
double Input = 0;
double Output = 0;
double Kp = 0.0;
double Ki = 0.0;
double Kd = 0.0;
ArduPID myPID;

// Speed thresholds and correction factor
float cut_speed = 2500.0;
float threshold_ratio = 0.7;
float threshold_speed = cut_speed * threshold_ratio;

float tension_correction_factor = 1.0;

// Display and logging variables
unsigned long dernier_affichage = 0;
const unsigned long intervalle_affichage = 250;
const float seuil_arc = 10.0;
const unsigned long delai_stabilisation = 500;
unsigned long temps_plasma_active = 0;
bool plasma_stabilise = false;
const unsigned long LOG_INTERVAL = 150;
unsigned long lastLogTime = 0;
bool thc_actif = false;

// Encoder and screen navigation
volatile int encoderPos = 0;
int currentScreen = 0;
const int NB_SCREENS = 8;
int prevCLK; // Previous state of CLK
int prevDT;  // Previous state of DT

// Custom LCD characters
byte enableChar[8] = {0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b00000};
byte plasmaChar[8] = {0b11111, 0b11111, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
byte thcActifChar[8] = {0b11111, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111, 0b00000};
byte arrowUp[8] = {0b00100, 0b01110, 0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00000};
byte arrowDown[8] = {0b00100, 0b00100, 0b00100, 0b00100, 0b11111, 0b01110, 0b00100, 0b00000};
byte stableChar[8] = {0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000};
byte enableall[8] = {0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b00000};

// Variables globales pour la mesure du temps
unsigned long loopStartTime;
unsigned long loopEndTime;
unsigned long lastLoopLogTime = 0;
const unsigned long LOOP_LOG_INTERVAL = 10000;  // 10 secondes
unsigned long loopExecutionTimeSum = 0;
unsigned int loopCount = 0;

// Variables globales pour la méthode anti-dive
float tension_fast = 0.0;              // Tension filtrée rapidement
float tension_slow = 0.0;              // Tension filtrée lentement
bool anti_dive_active = false;         // État de l'anti-dive
unsigned long anti_dive_start_time = 0;// Temps de début de désactivation
const unsigned long ANTI_DIVE_DURATION_MIN = 50; // Durée min (ms) à haute vitesse
const unsigned long ANTI_DIVE_DURATION_MAX = 300; // Durée max (ms) à basse vitesse
bool ewma_initialized = false;         // Initialisation des filtres
float ALPHA =0.0;
// Tension filtering variables
float tension_reelle_filtre = 0.0;

// Variables globales pour les moyennes glissantes
const int N_FAST_MIN = 5;    // Min échantillons pour tension rapide
const int N_FAST_MAX = 20;   // Max échantillons pour tension rapide
const int N_SLOW_MIN = 10;   // Min échantillons pour tension lente
const int N_SLOW_MAX = 50;   // Max échantillons pour tension lente
float tension_samples_fast[50]; // Tableau pour tension rapide (taille max)
float tension_samples_slow[50]; // Tableau pour tension lente (taille max)
int index_fast = 0;
int index_slow = 0;
float sum_fast = 0.0;
float sum_slow = 0.0;
int N_fast = N_FAST_MIN;
int N_slow = N_SLOW_MIN;
bool arrays_initialized = false; // Indicateur d'initialisation des tableaux

// Function declarations
void calculateSpeed();
void readAndFilterTension();
void managePlasmaAndTHC();
void updateLCD();
void doEncoder();
void countStepX();
void countStepY();

void setup() {
  pinMode(PLASMA_PIN, INPUT);
  pinMode(ENABLE_PIN, INPUT_PULLUP);
  pinMode(SWITCH1, OUTPUT);
  pinMode(SWITCH2, OUTPUT);
  pinMode(STEP_X_PIN, INPUT);
  pinMode(STEP_Y_PIN, INPUT);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(SWITCH1, HIGH);
  digitalWrite(SWITCH2, HIGH);

  Serial.begin(115200);

  // Initialisation des états précédents pour l'encodeur
  prevCLK = digitalRead(ENCODER_PIN_A);
  prevDT = digitalRead(ENCODER_PIN_B);

  attachInterrupt(digitalPinToInterrupt(STEP_X_PIN), countStepX, FALLING);
  attachInterrupt(digitalPinToInterrupt(STEP_Y_PIN), countStepY, FALLING);

  lcd.init();
  lcd.begin(16, 2);
  lcd.backlight();

  lcd.createChar(0, enableChar);
  lcd.createChar(1, plasmaChar);
  lcd.createChar(2, thcActifChar);
  lcd.createChar(3, arrowUp);
  lcd.createChar(4, arrowDown);
  lcd.createChar(5, stableChar);
  lcd.createChar(6, enableall);

  for (int i = 0; i < taille_filtre_vitesse; i++) {
    lectures_vitesse[i] = 0.0;
  }

  stepper.setMaxSpeed(16000);
  
  // Configuration pour 100 ms
  stepper.setAcceleration(60000);
  myPID.begin(&Input, &Output, &Setpoint, 350.0, 0.1, 20.0);  // Initialisation ArduPID
  myPID.setOutputLimits(-10000, 10000);  // Limites de sortie
  myPID.setDeadBand(-0.75, 0.75);
  Ki = 0.1;
  Kd = 20.0;
  Kp = 250.0;
  myPID.setCoefficients(Kp, Ki, Kd);
  // Configuration pour 250 ms
    //stepper.setAcceleration(10000);
    //myPID.SetOutputLimits(-2000, 2000);
    //myPID.SetTunings(150.0, 0.2, 0.5);
    //Kp=150.0;
    //Ki=0.2;
    //Kd=0.5;
  // Configuration pour 500 ms
    //stepper.setAcceleration(9000);
    //myPID.SetOutputLimits(-1000, 1000);
    //myPID.SetTunings(150.0, 0.1, 0.2);
    //Kp=150.0;
    //Ki=0.1;
    //Kd=0.2;
  // Configuration pour 1000 ms
    //stepper.setAcceleration(2000);
    //myPID.SetOutputLimits(-500, 500);
    //myPID.SetTunings(150.0, 0.05, 0.1);
    //Kp=150.0;
    //Ki=0.05;
    //Kd=0.1;

  analogReadResolution(14); // ADC à 14 bits
}

void loop() {
  loopStartTime = micros();
  unsigned long currentTime = millis();

  // Gestion du bouton pour changer d'écran
  static bool lastButtonState = HIGH;
  static unsigned long lastButtonTime = 0;
  bool buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH && currentTime - lastButtonTime > 100) {
    currentScreen = (currentScreen + 1) % NB_SCREENS;
    lastButtonTime = currentTime;
  }
  lastButtonState = buttonState;

  // Gestion de l'encodeur
  int currentCLK = digitalRead(ENCODER_PIN_A);
  int currentDT = digitalRead(ENCODER_PIN_B);
  if (currentCLK != prevCLK) {
    if (currentCLK == HIGH && currentDT == LOW && prevCLK == LOW && prevDT == HIGH) {
      encoderPos++;
    }
    if (currentCLK == HIGH && currentDT == HIGH && prevCLK == LOW && prevDT == LOW) {
      encoderPos--;
    }
    prevCLK = currentCLK;
    prevDT = currentDT;
  }

  // Gestion des changements de l'encodeur
  static int lastEncoderPos = 0;
  int currentEncoderPos = encoderPos;
  int delta = currentEncoderPos - lastEncoderPos;
  lastEncoderPos = currentEncoderPos;

  // Gestion des écrans et ajustements des paramètres
  switch (currentScreen) {
    case 1: // Tension cible
      Setpoint += delta;
      if (Setpoint < 50) Setpoint = 50;
      if (Setpoint > 200) Setpoint = 200;
      break;
    case 2: // Facteur de correction de tension
      tension_correction_factor += delta * 0.01;
      break;
    case 3: // Vitesse de coupe
      cut_speed += delta * 100;
      if (cut_speed < 0) cut_speed = 0;
      threshold_speed = cut_speed * threshold_ratio;
      break;
    case 4: // Seuil de vitesse
      threshold_ratio += delta * 0.1;
      threshold_speed = cut_speed * threshold_ratio;
      break;
    case 5: // PID Kp
      Kp += delta * 10;
      if (Kp < 0) Kp = 0;
      myPID.setCoefficients(Kp, Ki, Kd);
      break;
    case 6: // PID Ki
      Ki += delta * 0.01;
      if (Ki < 0) Ki = 0;
      myPID.setCoefficients(Kp, Ki, Kd);
      break;
    case 7: // PID Kd
      Kd += delta * 0.01;
      if (Kd < 0) Kd = 0;
      myPID.setCoefficients(Kp, Ki, Kd);
      break;
  }

  if (currentTime - lastSpeedTime >= SPEED_INTERVAL) {
    calculateSpeed();
  }

  if (currentTime - lastLogTime >= LOG_INTERVAL) {
    bool plasma_pin_low = (digitalRead(PLASMA_PIN) == LOW);
    
    if (plasma_pin_low) {
      Serial.print("Vitesse torche: ");
      Serial.print(vitesse_torche_filtre);
      Serial.println(" mm/min");
      Serial.print("PLASMA_PIN: LOW | ENABLE_PIN: ");
      Serial.print(digitalRead(ENABLE_PIN) == LOW ? "LOW" : "HIGH");
      Serial.print(" | Stabilisation: ");
      Serial.print(plasma_stabilise ? "OK" : "En attente");
      Serial.print(" | Arc détecté: ");
      Serial.print(tension_reelle_filtre > seuil_arc ? "Oui" : "Non");
      Serial.print(" | THC actif: ");
      Serial.println(thc_actif ? "Oui" : "Non");
      Serial.print(" V | Tension fast: ");
      Serial.print(tension_fast);
      Serial.print(" V | Tension slow: ");
      Serial.print(tension_slow);
      Serial.print(" V | Anti-dive: ");
      Serial.print(anti_dive_active ? "Actif" : "Inactif");
      Serial.print(" | Alpha: ");
      Serial.print(ALPHA);  // Coefficient alpha utilisé pour le filtrage
      Serial.print(" | Tension cible: ");
      Serial.print(Setpoint);
      Serial.print(" V | PID Output: ");
      Serial.println(Output);
    } else {
      Serial.print(" V | Tension fast: ");
      Serial.print(tension_fast);
      Serial.println(" V | PLASMA_PIN: HIGH");
    }
    
    lastLogTime = currentTime;
  }
  managePlasmaAndTHC();
  readAndFilterTension();

  if (currentTime - dernier_affichage >= intervalle_affichage) {
    updateLCD();
    dernier_affichage = currentTime;
  }

  loopEndTime = micros();
  unsigned long loopExecutionTime = loopEndTime - loopStartTime;
  loopExecutionTimeSum += loopExecutionTime;
  loopCount++;

  if (currentTime - lastLoopLogTime >= LOOP_LOG_INTERVAL) {
    if (loopCount > 0) {
      unsigned long averageLoopTime = loopExecutionTimeSum / loopCount;
      float loopFrequency = 1000000.0 / averageLoopTime;
      Serial.print("Temps d'exécution moyen: ");
      Serial.print(averageLoopTime);
      Serial.print(" us | Fréquence: ");
      Serial.print(loopFrequency, 1);
      Serial.println(" Hz");
    }
    loopExecutionTimeSum = 0;
    loopCount = 0;
    lastLoopLogTime = currentTime;
  }
}

void calculateSpeed() {
  unsigned long currentTime = millis();
  noInterrupts();
  unsigned long tempX = stepCountX;
  unsigned long tempY = stepCountY;
  stepCountX = 0;
  stepCountY = 0;
  interrupts();

  totalStepX += tempX;
  totalStepY += tempY;

  float vitesse_torche = 0.0;
  unsigned long deltaTime = currentTime - lastSpeedTime;
  if (tempX + tempY >= MIN_STEPS && deltaTime > 0) {
    float distanceX = tempX * DIST_PER_STEP_X;
    float distanceY = tempY * DIST_PER_STEP_Y;
    float totalDistance = sqrt(distanceX * distanceX + distanceY * distanceY);
    vitesse_torche = (totalDistance / (deltaTime / 1000.0)) * 60.0 * SPEED_CORRECTION;
  }

  somme_lectures_vitesse -= lectures_vitesse[index_lecture_vitesse];
  lectures_vitesse[index_lecture_vitesse] = vitesse_torche;
  somme_lectures_vitesse += vitesse_torche;
  index_lecture_vitesse = (index_lecture_vitesse + 1) % taille_filtre_vitesse;
  vitesse_torche_filtre = somme_lectures_vitesse / taille_filtre_vitesse;
  if (vitesse_torche_filtre < 0) vitesse_torche_filtre = 0.0;

  if (tempX + tempY == 0 && currentTime - lastStepTime > STEP_TIMEOUT) {
    totalStepX = 0;
    totalStepY = 0;
    for (int i = 0; i < taille_filtre_vitesse; i++) {
      lectures_vitesse[i] = 0.0;
    }
    somme_lectures_vitesse = 0.0;
    vitesse_torche_filtre = 0.0 + vitesse_test;
  }
  if (tempX + tempY > 0) lastStepTime = currentTime;

  lastSpeedTime = currentTime;
}

// Fonction de lecture et filtrage des tensions
void readAndFilterTension() {
  int reading = analogRead(PLASMA_VOLTAGE);
  float tension_reelle = (reading / 16383.0) * 5.0 * 83.27 * tension_correction_factor + plasma_test_V;

  // Initialisation des tableaux à la première lecture
  if (!arrays_initialized) {
    for (int i = 0; i < N_SLOW_MAX; i++) {
      tension_samples_fast[i] = tension_reelle;
      tension_samples_slow[i] = tension_reelle;
    }
    sum_fast = tension_reelle * N_FAST_MIN;
    sum_slow = tension_reelle * N_SLOW_MIN;
    arrays_initialized = true;
  }

  // Interpolation du nombre d'échantillons selon la vitesse de coupe
  float ratio = (cut_speed - 500.0) / (5000.0 - 500.0);
  N_fast = N_FAST_MIN + (int)(ratio * (N_FAST_MAX - N_FAST_MIN));
  N_slow = N_SLOW_MIN + (int)(ratio * (N_SLOW_MAX - N_SLOW_MIN));
  N_fast = constrain(N_fast, N_FAST_MIN, N_FAST_MAX);
  N_slow = constrain(N_slow, N_SLOW_MIN, N_SLOW_MAX);

  // Mise à jour de la moyenne glissante pour tension rapide
  sum_fast -= tension_samples_fast[index_fast];
  tension_samples_fast[index_fast] = tension_reelle;
  sum_fast += tension_reelle;
  index_fast = (index_fast + 1) % N_fast;
  tension_fast = sum_fast / N_fast;

  // Mise à jour de la moyenne glissante pour tension lente
  sum_slow -= tension_samples_slow[index_slow];
  tension_samples_slow[index_slow] = tension_reelle;
  sum_slow += tension_reelle;
  index_slow = (index_slow + 1) % N_slow;
  tension_slow = sum_slow / N_slow;

  // Utiliser tension_fast pour le PID
  Input = tension_fast;

  // Détection anti-dive
  const float SEUIL_CHUTE = 2.0;  // Volts
  static bool last_anti_dive_state = false;
  if ((tension_fast < tension_slow - SEUIL_CHUTE || tension_fast > tension_slow + SEUIL_CHUTE) && !anti_dive_active) {
    anti_dive_active = true;
    anti_dive_start_time = millis();
    if (!last_anti_dive_state) { // Transition OFF -> ON
      Serial.print("Anti-dive ON | Tension de coupe: ");
      Serial.print(tension_reelle);
      Serial.print(" V | Tension rapide: ");
      Serial.print(tension_fast);
      Serial.println(" V");
    }
  }

  // Durée de désactivation adaptative
  unsigned long ANTI_DIVE_DURATION = ANTI_DIVE_DURATION_MAX - 
    (ANTI_DIVE_DURATION_MAX - ANTI_DIVE_DURATION_MIN) * ratio;
  ANTI_DIVE_DURATION = constrain(ANTI_DIVE_DURATION, ANTI_DIVE_DURATION_MIN, ANTI_DIVE_DURATION_MAX);

  // Durée anti-dive adaptative
  //const unsigned long ANTI_DIVE_DURATION = 200;  // ms
  if (anti_dive_active && (millis() - anti_dive_start_time >= ANTI_DIVE_DURATION)) {
    anti_dive_active = false;
    if (last_anti_dive_state) { // Transition ON -> OFF
      Serial.print("Anti-dive OFF | Tension de coupe: ");
      Serial.print(tension_reelle);
      Serial.print(" V | Tension rapide: ");
      Serial.print(tension_fast);
      Serial.println(" V");
      }
    }
    last_anti_dive_state = anti_dive_active; // Mettre à jour l'état précédent
}

void managePlasmaAndTHC() {
  bool plasma_pin_low = (digitalRead(PLASMA_PIN) == LOW);
  bool enable_pin_low = (digitalRead(ENABLE_PIN) == LOW);
  bool arc_detecte = (tension_reelle_filtre > seuil_arc);

  if (plasma_pin_low) {
    digitalWrite(SWITCH1, LOW);
    digitalWrite(SWITCH2, LOW);
  } else {
    digitalWrite(SWITCH1, HIGH);
    digitalWrite(SWITCH2, HIGH);
  }

  if (plasma_pin_low && !plasma_stabilise) {
    if (temps_plasma_active == 0) temps_plasma_active = millis();
    if (millis() - temps_plasma_active >= delai_stabilisation) plasma_stabilise = true;
  } else if (!plasma_pin_low) {
    temps_plasma_active = 0;
    plasma_stabilise = false;
  }

  static bool thc_etat = false;
  if (!thc_etat && vitesse_torche_filtre >= threshold_speed) {
    thc_etat = true;
  } else if (thc_etat && vitesse_torche_filtre < threshold_speed) {
    thc_etat = false;
  }
  thc_actif = enable_pin_low && plasma_pin_low && plasma_stabilise && arc_detecte && thc_etat && !anti_dive_active;;

  myPID.compute();
  if (thc_actif) {
    double error = Setpoint - Input;
      stepper.setSpeed(Output);
      stepper.runSpeed();
    } else {
      stepper.setSpeed(0);
    }

}

void updateLCD() {
  static int lastScreen = -1;
  static float lastTensionReelle = -1;
  static float lastSetpoint = -1;
  static int lastVitesse = -1;
  static bool lastThcActif = false;
  static double lastOutput = 0;
  static bool lastEnableLow = false;
  static bool lastPlasmaLow = false;

  if (currentScreen != lastScreen) {
    lcd.clear();
    switch (currentScreen) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("TRC:     V");
        lcd.setCursor(0, 1);
        lcd.print("TGT:     V");
        break;
      case 1:
        lcd.setCursor(0, 0);
        lcd.print("Set V:      V");
        break;
      case 2:
        lcd.setCursor(0, 0);
        lcd.print("V factor:   ");
        break;
      case 3:
        lcd.setCursor(0, 0);
        lcd.print("CutSpd:     ");
        break;
      case 4:
        lcd.setCursor(0, 0);
        lcd.print("CutSpdThs %:");
        break;
      case 5:
        lcd.setCursor(0, 0);
        lcd.print("PID Kp:     ");
        break;
      case 6:
        lcd.setCursor(0, 0);
        lcd.print("PID Ki:     ");
        break;
      case 7:
        lcd.setCursor(0, 0);
        lcd.print("PID Kd:     ");
        break;
    }
    lastScreen = currentScreen;
    lastTensionReelle = -1;
    lastSetpoint = -1;
    lastVitesse = -1;
    lastThcActif = false;
    lastOutput = 0;
    lastEnableLow = false;
    lastPlasmaLow = false;
  }

  if (currentScreen == 0) {
    if (tension_reelle_filtre != lastTensionReelle) {
      lcd.setCursor(4, 0);
      lcd.print(tension_reelle_filtre, 1);
      lastTensionReelle = tension_reelle_filtre;
    }

    if (Setpoint != lastSetpoint) {
      lcd.setCursor(4, 1);
      lcd.print(Setpoint, 1);
      lastSetpoint = Setpoint;
    }

    int vitesse_affichee = (int)(vitesse_torche_filtre);
    if (vitesse_affichee > 9999) vitesse_affichee = 9999;
    if (vitesse_torche_filtre < 0.1) {
      if (lastVitesse != 0) {
        lcd.setCursor(12, 0);
        lcd.print("   0");
        lastVitesse = 0;
      }
    } else if (vitesse_affichee != lastVitesse) {
      lcd.setCursor(12, 0);
      lcd.print(vitesse_affichee);
      lastVitesse = vitesse_affichee;
    }

    bool enable_low = digitalRead(ENABLE_PIN) == LOW;
    bool plasma_low = digitalRead(PLASMA_PIN) == LOW;
    if (thc_actif != lastThcActif || enable_low != lastEnableLow || plasma_low != lastPlasmaLow) {
      lcd.setCursor(11, 1);
      if (thc_actif) {
        lcd.write(2);
      } else if (enable_low && plasma_low) {
        lcd.write(6);
      } else if (enable_low) {
        lcd.write(0);
      } else if (plasma_low) {
        lcd.write(1);
      } else {
        lcd.print(" ");
      }
      lastThcActif = thc_actif;
      lastEnableLow = enable_low;
      lastPlasmaLow = plasma_low;
    }

    if (thc_actif) {
      lcd.setCursor(13, 1);
      if (Output > 10) {
        lcd.write(3);
      } else if (Output < -10) {
        lcd.write(4);
      } else {
        lcd.write(5);
      }
      int intensity = map(abs(Output), 0, 1000, 0, 2);
      lcd.setCursor(14, 1);
      for (int i = 0; i < intensity; i++) {
        lcd.print("|");
      }
      for (int i = intensity; i < 2; i++) {
        lcd.print(" ");
      }
    } else if (lastOutput != 0) {
      lcd.setCursor(13, 1);
      lcd.print("   ");
    }
    lastOutput = Output;
  } else {
    switch (currentScreen) {
      case 1:
        lcd.setCursor(7, 0);
        lcd.print(Setpoint, 1);
        break;
      case 2:
        lcd.setCursor(10, 0);
        lcd.print(tension_correction_factor, 2);
        break;
      case 3:
        lcd.setCursor(8, 0);
        lcd.print(cut_speed, 0);
        break;
      case 4:
        lcd.setCursor(12, 0);
        lcd.print(threshold_ratio, 1);
        break;
      case 5:
        lcd.setCursor(8, 0);
        lcd.print(Kp, 1);
        break;
      case 6:
        lcd.setCursor(8, 0);
        lcd.print(Ki, 2);
        break;
      case 7:
        lcd.setCursor(8, 0);
        lcd.print(Kd, 2);
        break;
    }
  }
}

void countStepX() {
  unsigned long currentTime = micros();
  if (currentTime - lastStepXTime >= DEBOUNCE_US) {
    stepCountX++;
    pulseTimeX = currentTime;
    lastStepXTime = currentTime;
  } else bounceCountX++;
}

void countStepY() {
  unsigned long currentTime = micros();
  if (currentTime - lastStepYTime >= DEBOUNCE_US) {
    stepCountY++;
    pulseTimeY = currentTime;
    lastStepYTime = currentTime;
  } else bounceCountY++;
}