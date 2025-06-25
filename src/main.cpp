#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <ArduPID.h>
#include <EEPROM.h>

// Define pins for Arduino Uno R4 Minima
#define PLASMA_PIN 12  // Input pin for plasma arc OK signal
#define ENABLE_PIN 10  // Changed to avoid conflict with SWITCH2
#define SWITCH1 9      // Formerly PIN_2
#define SWITCH2 13     // Formerly PIN_3
#define DIR_PIN 8
#define STEP_PIN 6
#define STEP_X_PIN 2   // Interrupt pin
#define STEP_Y_PIN 3   // Interrupt pin
#define ENCODER_PIN_A 4 // Encoder CLK
#define ENCODER_PIN_B 5 // Encoder DT
#define BUTTON_PIN 7    // Encoder SW
#define PLASMA_VOLTAGE A0 // ADC input for plasma voltage
#define THC_OFF_PIN 11  // New input pin for THC_OFF

// EEPROM addresses for parameters
#define EEPROM_SETPOINT_ADDR 0
#define EEPROM_CORRECTION_FACTOR_ADDR 4
#define EEPROM_CUT_SPEED_ADDR 8
#define EEPROM_THRESHOLD_RATIO_ADDR 12
#define EEPROM_KP_ADDR 16
#define EEPROM_KI_ADDR 20
#define EEPROM_KD_ADDR 24
#define EEPROM_INITIALIZED_FLAG 28 // Address for initialization flag

// Default parameter values
const float DEFAULT_SETPOINT = 100.0;
const float DEFAULT_CORRECTION_FACTOR = 1.0;
const float DEFAULT_CUT_SPEED = 1700.0;
const float DEFAULT_THRESHOLD_RATIO = 0.7;
const float DEFAULT_KP = 350.0;
const float DEFAULT_KI = 0.0;
const float DEFAULT_KD = 20.0;

// Initialize objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

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
double Setpoint = DEFAULT_SETPOINT;
double Input = 0;
double Output = 0;
double Kp = DEFAULT_KP;
double Ki = DEFAULT_KI;
double Kd = DEFAULT_KD;
ArduPID myPID;

// Speed thresholds and correction factor
float cut_speed = DEFAULT_CUT_SPEED;
float threshold_ratio = DEFAULT_THRESHOLD_RATIO;
float threshold_speed = cut_speed * threshold_ratio;
float tension_correction_factor = DEFAULT_CORRECTION_FACTOR;

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

// Variable globale pour thc_etat
bool thc_etat = false; // État de la vitesse pour le THC

// Function declarations
void initializeEEPROM();
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
  pinMode(THC_OFF_PIN, INPUT); // Configure THC_OFF_PIN as input
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
  myPID.begin(&Input, &Output, &Setpoint, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
  myPID.setOutputLimits(-10000, 10000);  // Limites de sortie
  Ki = DEFAULT_KI;
  Kd = DEFAULT_KD;
  Kp = DEFAULT_KP;
  myPID.setCoefficients(Kp, Ki, Kd);

  // Initialize EEPROM with default values if not already initialized
  initializeEEPROM();

  // Load parameters from EEPROM
  EEPROM.get(EEPROM_SETPOINT_ADDR, Setpoint);
  EEPROM.get(EEPROM_CORRECTION_FACTOR_ADDR, tension_correction_factor);
  EEPROM.get(EEPROM_CUT_SPEED_ADDR, cut_speed);
  EEPROM.get(EEPROM_THRESHOLD_RATIO_ADDR, threshold_ratio);
  EEPROM.get(EEPROM_KP_ADDR, Kp);
  EEPROM.get(EEPROM_KI_ADDR, Ki);
  EEPROM.get(EEPROM_KD_ADDR, Kd);

  // Validate loaded values to prevent garbage data or NaN
  if (isnan(Setpoint) || Setpoint < 50 || Setpoint > 200) Setpoint = DEFAULT_SETPOINT;
  if (isnan(tension_correction_factor) || tension_correction_factor < 0.5 || tension_correction_factor > 2.0) tension_correction_factor = DEFAULT_CORRECTION_FACTOR;
  if (isnan(cut_speed) || cut_speed < 0 || cut_speed > 10000) cut_speed = DEFAULT_CUT_SPEED;
  if (isnan(threshold_ratio) || threshold_ratio < 0 || threshold_ratio > 1) threshold_ratio = DEFAULT_THRESHOLD_RATIO;
  if (isnan(Kp) || Kp < 0 || Kp > 1000) Kp = DEFAULT_KP;
  if (isnan(Ki) || Ki < 0 || Ki > 1) Ki = DEFAULT_KI;
  if (isnan(Kd) || Kd < 0 || Kd > 100) Kd = DEFAULT_KD;

  myPID.setCoefficients(Kp, Ki, Kd);
  threshold_speed = cut_speed * threshold_ratio;

  analogReadResolution(14); // ADC à 14 bits
}

void initializeEEPROM() {
  // Check if EEPROM has been initialized
  byte initializedFlag;
  EEPROM.get(EEPROM_INITIALIZED_FLAG, initializedFlag);
  
  if (initializedFlag != 0xAA) {
    // Initialize EEPROM with default values
    EEPROM.put(EEPROM_SETPOINT_ADDR, DEFAULT_SETPOINT);
    EEPROM.put(EEPROM_CORRECTION_FACTOR_ADDR, DEFAULT_CORRECTION_FACTOR);
    EEPROM.put(EEPROM_CUT_SPEED_ADDR, DEFAULT_CUT_SPEED);
    EEPROM.put(EEPROM_THRESHOLD_RATIO_ADDR, DEFAULT_THRESHOLD_RATIO);
    EEPROM.put(EEPROM_KP_ADDR, DEFAULT_KP);
    EEPROM.put(EEPROM_KI_ADDR, DEFAULT_KI);
    EEPROM.put(EEPROM_KD_ADDR, DEFAULT_KD);
    
    // Set the initialized flag
    initializedFlag = 0xAA;
    EEPROM.put(EEPROM_INITIALIZED_FLAG, initializedFlag);
    
    Serial.println("EEPROM initialized with default values");
  }
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

  // Gestion des écrans et ajustements des paramètres avec sauvegarde EEPROM
  static float lastSetpoint = Setpoint;
  static float lastTensionCorrectionFactor = tension_correction_factor;
  static float lastCutSpeed = cut_speed;
  static float lastThresholdRatio = threshold_ratio;
  static float lastKp = Kp;
  static float lastKi = Ki;
  static float lastKd = Kd;

  switch (currentScreen) {
    case 1: // Tension cible
      Setpoint += delta;
      if (Setpoint < 50) Setpoint = 50;
      if (Setpoint > 200) Setpoint = 200;
      if (Setpoint != lastSetpoint) {
        EEPROM.put(EEPROM_SETPOINT_ADDR, Setpoint);
        lastSetpoint = Setpoint;
      }
      break;
    case 2: // Facteur de correction de tension
      tension_correction_factor += delta * 0.01;
      if (tension_correction_factor != lastTensionCorrectionFactor) {
        EEPROM.put(EEPROM_CORRECTION_FACTOR_ADDR, tension_correction_factor);
        lastTensionCorrectionFactor = tension_correction_factor;
      }
      break;
    case 3: // Vitesse de coupe
      cut_speed += delta * 100;
      if (cut_speed < 0) cut_speed = 0;
      if (cut_speed != lastCutSpeed) {
        EEPROM.put(EEPROM_CUT_SPEED_ADDR, cut_speed);
        lastCutSpeed = cut_speed;
      }
      threshold_speed = cut_speed * threshold_ratio;
      break;
    case 4: // Seuil de vitesse
      threshold_ratio += delta * 0.1;
      if (threshold_ratio != lastThresholdRatio) {
        EEPROM.put(EEPROM_THRESHOLD_RATIO_ADDR, threshold_ratio);
        lastThresholdRatio = threshold_ratio;
      }
      threshold_speed = cut_speed * threshold_ratio;
      break;
    case 5: // PID Kp
      Kp += delta * 10;
      if (Kp < 0) Kp = 0;
      if (Kp != lastKp) {
        EEPROM.put(EEPROM_KP_ADDR, Kp);
        lastKp = Kp;
      }
      myPID.setCoefficients(Kp, Ki, Kd);
      break;
    case 6: // PID Ki
      Ki += delta * 0.01;
      if (Ki < 0) Ki = 0;
      if (Ki != lastKi) {
        EEPROM.put(EEPROM_KI_ADDR, Ki);
        lastKi = Ki;
      }
      myPID.setCoefficients(Kp, Ki, Kd);
      break;
    case 7: // PID Kd
      Kd += delta * 0.01;
      if (Kd < 0) Kd = 0;
      if (Kd != lastKd) {
        EEPROM.put(EEPROM_KD_ADDR, Kd);
        lastKd = Kd;
      }
      myPID.setCoefficients(Kp, Ki, Kd);
      break;
  }

  if (currentTime - lastSpeedTime >= SPEED_INTERVAL) {
    calculateSpeed();
  }

  if (currentTime - lastLogTime >= LOG_INTERVAL) {
    bool plasma_pin_low = (digitalRead(PLASMA_PIN) == LOW);
    bool thc_off = (digitalRead(THC_OFF_PIN) == HIGH);
    bool enable_pin_low = (digitalRead(ENABLE_PIN) == LOW);
    bool arc_detecte = (tension_fast > seuil_arc);

    if (plasma_pin_low) {
      Serial.print("TGT / Vitesse torche: ");
      Serial.print(cut_speed);
      Serial.print(" / ");
      Serial.print(vitesse_torche_filtre);
      Serial.print(" mm/min | Threshold speed: ");
      Serial.print(threshold_speed);
      Serial.println(" mm/min");
      Serial.print("PLASMA_PIN: LOW | ENABLE_PIN: ");
      Serial.print(enable_pin_low ? "LOW" : "HIGH");
      Serial.print(" | Stabilisation: ");
      Serial.print(plasma_stabilise ? "OK" : "En attente");
      Serial.print(" | Arc détecté: ");
      Serial.print(arc_detecte ? "Oui" : "Non");
      Serial.print(" | THC actif: ");
      Serial.print(thc_actif ? "Oui" : "Non");
      Serial.print(" | THC_SIG: ");
      Serial.print(thc_off ? "OFF" : "ON");
      Serial.print(" | THC état (vitesse): ");
      Serial.println(thc_etat ? "OK" : "Insuffisant");
      Serial.print("V | Tension fast: ");
      Serial.print(tension_fast);
      Serial.print(" V | Tension slow: ");
      Serial.print(tension_slow);
      Serial.print(" V | Anti-dive: ");
      Serial.print(anti_dive_active ? "Actif" : "Inactif");
      Serial.print(" | Tension cible: ");
      Serial.print(Setpoint);
      Serial.print(" V | PID Output: ");
      Serial.println(Output);
      
      // Log explicite si THC inactif
      if (!thc_actif && !thc_off) {
        Serial.print("Raison THC inactif: ");
        if (!enable_pin_low) Serial.println("ENABLE_PIN HIGH");
        else if (!plasma_pin_low) Serial.println("PLASMA_PIN HIGH");
        else if (!plasma_stabilise) Serial.println("Stabilisation non atteinte");
        else if (!arc_detecte) Serial.println("Arc non détecté (tension faible)");
        else if (!thc_etat) Serial.println("Vitesse torche < seuil");
        else if (anti_dive_active) Serial.println("Anti-dive actif");
        else Serial.println("Condition inconnue");
      }
    } else {
      Serial.print("Cut Speed setup: ");
      Serial.print(cut_speed);
      Serial.print(" mm/min | Tension fast: ");
      Serial.print(tension_fast);
      Serial.print(" V | PLASMA_PIN: HIGH | THC_SIG: ");
      Serial.println(thc_off ? "OFF" : "ON");
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

void readAndFilterTension() {
  int reading = analogRead(PLASMA_VOLTAGE);
  float tension_reelle = (reading / 16383.0) * 5.0 * 83.27 * tension_correction_factor + plasma_test_V;

  if (!arrays_initialized) {
    for (int i = 0; i < N_SLOW_MAX; i++) {
      tension_samples_fast[i] = tension_reelle;
      tension_samples_slow[i] = tension_reelle;
    }
    sum_fast = tension_reelle * N_FAST_MIN;
    sum_slow = tension_reelle * N_SLOW_MIN;
    arrays_initialized = true;
  }

  float ratio = (cut_speed - 500.0) / (5000.0 - 500.0);
  N_fast = N_FAST_MIN + (int)(ratio * (N_FAST_MAX - N_FAST_MIN));
  N_slow = N_SLOW_MIN + (int)(ratio * (N_SLOW_MAX - N_SLOW_MIN));
  N_fast = constrain(N_fast, N_FAST_MIN, N_FAST_MAX);
  N_slow = constrain(N_slow, N_SLOW_MIN, N_SLOW_MAX);

  sum_fast -= tension_samples_fast[index_fast];
  tension_samples_fast[index_fast] = tension_reelle;
  sum_fast += tension_reelle;
  index_fast = (index_fast + 1) % N_fast;
  tension_fast = sum_fast / N_fast;

  sum_slow -= tension_samples_slow[index_slow];
  tension_samples_slow[index_slow] = tension_reelle;
  sum_slow += tension_reelle;
  index_slow = (index_slow + 1) % N_slow;
  tension_slow = sum_slow / N_slow;

  Input = tension_fast;

  const float SEUIL_CHUTE = 2.0;  // Volts
  static bool last_anti_dive_state = false;
  if (tension_fast > tension_slow + SEUIL_CHUTE && !anti_dive_active && digitalRead(PLASMA_PIN) == LOW) {
    anti_dive_active = true;
    anti_dive_start_time = millis();
    if (!last_anti_dive_state) {
      Serial.print("Anti-dive ON | Tension de coupe: ");
      Serial.print(tension_reelle);
      Serial.print(" V | Tension rapide: ");
      Serial.print(tension_fast);
      Serial.println(" V");
    }
  }

  unsigned long ANTI_DIVE_DURATION = ANTI_DIVE_DURATION_MAX - 
    (ANTI_DIVE_DURATION_MAX - ANTI_DIVE_DURATION_MIN) * ratio;
  ANTI_DIVE_DURATION = constrain(ANTI_DIVE_DURATION, ANTI_DIVE_DURATION_MIN, ANTI_DIVE_DURATION_MAX);

  if (anti_dive_active && (millis() - anti_dive_start_time >= ANTI_DIVE_DURATION)) {
    anti_dive_active = false;
    if (last_anti_dive_state) {
      Serial.print("Anti-dive OFF | Tension de coupe: ");
      Serial.print(tension_reelle);
      Serial.print(" V | Tension rapide: ");
      Serial.print(tension_fast);
      Serial.println(" V");
    }
  }
  last_anti_dive_state = anti_dive_active;
}

void managePlasmaAndTHC() {
  bool plasma_pin_low = (digitalRead(PLASMA_PIN) == LOW);
  bool enable_pin_low = (digitalRead(ENABLE_PIN) == LOW);
  bool arc_detecte = (tension_fast > seuil_arc);
  bool thc_off = (digitalRead(THC_OFF_PIN) == HIGH);

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

  if (!thc_etat && vitesse_torche_filtre >= threshold_speed) {
    thc_etat = true;
  } else if (thc_etat && vitesse_torche_filtre < threshold_speed) {
    thc_etat = false;
  }

  // THC is disabled if THC_OFF_PIN is HIGH
  if (thc_off) {
    thc_actif = false;
  } else {
    thc_actif = enable_pin_low && plasma_pin_low && plasma_stabilise && arc_detecte && thc_etat && !anti_dive_active;
  }

  myPID.compute();
  if (thc_actif) {
    double error = Setpoint - Input;
    if (abs(error) > 0.5) {
      stepper.setSpeed(Output);
      stepper.runSpeed();
    } else {
      Output = 0.0;
      stepper.setSpeed(Output);
      stepper.runSpeed();
    }
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
  static bool lastThcOff = false;

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
    lastThcOff = false;
  }

  if (currentScreen == 0) {
    if (tension_fast != lastTensionReelle) {
      lcd.setCursor(4, 0);
      lcd.print(tension_fast, 1);
      lastTensionReelle = tension_fast;
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
    bool thc_off = digitalRead(THC_OFF_PIN) == HIGH;

    if (enable_low != lastEnableLow || plasma_low != lastPlasmaLow || 
        thc_actif != lastThcActif || thc_off != lastThcOff || Output != lastOutput) {
      
      lcd.setCursor(11, 1);
      if (enable_low) {
        lcd.print("S"); // Standby
      } else {
        lcd.print("O"); // Off
      }

      lcd.setCursor(12, 1);
      if (thc_actif) {
        if (Output > 10) {
          lcd.write(3); // arrowUp
        } else if (Output < -10) {
          lcd.write(4); // arrowDown
        } else {
          lcd.write(5); // stableChar
        }
      } else {
        lcd.print(" "); // Espace si THC inactif
      }

      lcd.setCursor(13, 1);
      if (thc_off) {
        lcd.print(" "); // THC désactivé par THC_OFF_PIN
      } else if (thc_actif) {
        lcd.write(2); // thcActifChar
      } else {
        lcd.print(" "); // THC inactif
      }

      lcd.setCursor(14, 1);
      if (plasma_low) {
        lcd.write(1); // plasmaChar
      } else {
        lcd.print(" "); // Pas d'arc
      }

      lastEnableLow = enable_low;
      lastPlasmaLow = plasma_low;
      lastThcActif = thc_actif;
      lastThcOff = thc_off;
      lastOutput = Output;
    }
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
  stepCountX++;
}

void countStepY() {
  stepCountY++;
}