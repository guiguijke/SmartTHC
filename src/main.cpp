#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <ArduPID.h>
#include <EEPROM.h>
#include <limits.h>

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
const float DEFAULT_SETPOINT = 110.0;
const float DEFAULT_CORRECTION_FACTOR = 1.0;
const float DEFAULT_THRESHOLD_RATIO = 0.8;
const float DEFAULT_KP = 30;
const float DEFAULT_KI = 7.5;
const float DEFAULT_KD = 2;

#ifndef STEPS_PER_MM_X
#define STEPS_PER_MM_X 200.0  // Default if not defined in platformio.ini
#endif
#ifndef STEPS_PER_MM_Y
#define STEPS_PER_MM_Y 200.0
#endif
#ifndef STEPS_PER_MM_Z
#define STEPS_PER_MM_Z 200.0
#endif

const float MM_PER_INCH = 25.4;

#if USE_IMPERIAL
const float DEFAULT_CUT_SPEED = 51.0; // Approx 1300 mm/min in IPM
const float MAX_CUT_SPEED = 400.0;
const float CUT_SPEED_ADJUST_STEP = 5.0;
const float DIST_PER_STEP_X = (1.0 / STEPS_PER_MM_X) / MM_PER_INCH;  // inch/step
const float DIST_PER_STEP_Y = (1.0 / STEPS_PER_MM_Y) / MM_PER_INCH;
const char* SPEED_UNIT = "IPM";
const char* SPEED_UNIT_LONG = "Inches/min";
#else
const float DEFAULT_CUT_SPEED = 1300.0;
const float MAX_CUT_SPEED = 10000.0;
const float CUT_SPEED_ADJUST_STEP = 100.0;
const float DIST_PER_STEP_X = 1.0 / STEPS_PER_MM_X;  // mm/step
const float DIST_PER_STEP_Y = 1.0 / STEPS_PER_MM_Y;
const char* SPEED_UNIT = "mm/min";
const char* SPEED_UNIT_LONG = "mm/min";
#endif

// For Z axis (anti-dive), always in mm
const float DIST_PER_STEP_Z = 1.0 / STEPS_PER_MM_Z;  // mm/step (added for consistency with X/Y)

byte initializedFlag = 0xAA;

// Initialize objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Speed filtering variables
const int speed_filter_size = 1;
float speed_readings[speed_filter_size];
int speed_reading_index = 0;
float sum_speed_readings = 0.0;
float filtered_torch_speed = 0.0;
float test_speed = 0; // For testing
float test_plasma_voltage = 0; // For testing

// Step counting variables
volatile uint16_t stepCountX = 0;
volatile uint16_t stepCountY = 0;
unsigned long totalStepX = 0;
unsigned long totalStepY = 0;
const unsigned long SPEED_INTERVAL = 50;
unsigned long lastSpeedTime = 0;
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
float voltage_correction_factor = DEFAULT_CORRECTION_FACTOR;

// Display and logging variables
unsigned long last_display_time = 0;
const unsigned long display_interval = 250;
const float arc_threshold = 10.0;
const unsigned long stabilization_delay = 500;
unsigned long plasma_active_time = 0;
bool plasma_stabilized = false;
const unsigned long LOG_INTERVAL = 150;
unsigned long lastLogTime = 0;
bool thc_active = false;

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

// Global variables for timing measurement
unsigned long loopStartTime;
unsigned long loopEndTime;
unsigned long lastLoopLogTime = 0;
const unsigned long LOOP_LOG_INTERVAL = 10000;  // 10 seconds
unsigned long loopExecutionTimeSum = 0;
unsigned int loopCount = 0;
bool just_anti_dive_activated = false;

// Global variables for anti-dive method
float fast_voltage = 0.0;              // Fast filtered voltage (corrected)
float slow_voltage = 0.0;              // Slow filtered voltage (corrected)
float uncorrected_fast = 0.0;          // Uncorrected fast voltage
float uncorrected_slow = 0.0;          // Uncorrected slow voltage
bool anti_dive_active = false;         // Anti-dive state
unsigned long anti_dive_start_time = 0;// Anti-dive start time
const unsigned long ANTI_DIVE_DURATION_MIN = 50; // Min duration (ms) at high speed
const unsigned long ANTI_DIVE_DURATION_MAX = 300; // Max duration (ms) at low speed

// New for improved anti-dive: position history buffer
const int POSITION_HISTORY_INTERVAL = 100; // ms between records
const int POSITION_HISTORY_SIZE = 20; // Enough for ~2 seconds
struct PosHistory {
  unsigned long time;
  long position;
};
PosHistory position_history[POSITION_HISTORY_SIZE];
int position_history_index = 0;
unsigned long last_position_record_time = 0;

// New constants for lift during anti-dive
const long ANTI_DIVE_BONUS_STEPS = ANTI_DIVE_BONUS_MM * STEPS_PER_MM_Z; // Positive for up (using STEPS_PER_MM_Z)
const float ANTI_DIVE_LIFT_SPEED = 1000.0; // Steps/s for lift (adjust as needed)
const float ANTI_DIVE_LIFT_ACCEL = 5000.0; // Acceleration for position move

// Global variable for thc_speed_state
bool thc_speed_state = false; // Speed state for THC

// EEPROM write delay
static unsigned long last_eeprom_write = 0;
const unsigned long EEPROM_WRITE_INTERVAL = 1000;

// Temporary variable for voltage correction factor adjustment
float temp_voltage_correction_factor = DEFAULT_CORRECTION_FACTOR;

// === OVERSAMPLING NON-BLOQUANT pour PID ultra-stable ===
#define OVERSAMPLE_TARGET 10  // 10 samples ~10ms @1kHz
static float oversample_sum = 0.0;
static uint8_t oversample_count = 0;
static float last_pid_input = 0.0;  // Dernière moyenne pour low-pass
const float INPUT_ALPHA = 0.7;      // Low-pass fort sur moyenne

// Function declarations
void initializeEEPROM();
void calculateSpeed();
void readAndFilterVoltage();
void managePlasmaAndTHC();
void updateLCD();
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

  // Initialize previous states for encoder
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

  for (int i = 0; i < speed_filter_size; i++) {
    speed_readings[i] = 0.0;
  }

  stepper.setMaxSpeed(5000);  // 5000 steps/s = 100 mm/s with 50 steps/mm
  stepper.setAcceleration(5000);  
  myPID.begin(&Input, &Output, &Setpoint, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
  myPID.setOutputLimits(-2500, 2500);  // ±2500 steps/s = ±50 mm/s
  myPID.setWindUpLimits(-100, 100); // Limits of ±100 steps/s for integral term
  Ki = DEFAULT_KI;
  Kd = DEFAULT_KD;
  Kp = DEFAULT_KP;
  myPID.setCoefficients(Kp, Ki, Kd);
  
  // Initialize EEPROM with default values if not already initialized
  initializeEEPROM();

  // Validate loaded values to prevent garbage data or NaN
  if (isnan(Setpoint) || Setpoint < 50 || Setpoint > 200) Setpoint = DEFAULT_SETPOINT;
  if (isnan(voltage_correction_factor) || voltage_correction_factor < 0.5 || voltage_correction_factor > 2.0) voltage_correction_factor = DEFAULT_CORRECTION_FACTOR;
  if (isnan(cut_speed) || cut_speed < 0 || cut_speed > MAX_CUT_SPEED) cut_speed = DEFAULT_CUT_SPEED;
  if (isnan(threshold_ratio) || threshold_ratio < 0 || threshold_ratio > 1) threshold_ratio = DEFAULT_THRESHOLD_RATIO;
  if (isnan(Kp) || Kp < 0.0 || Kp > 1500) Kp = DEFAULT_KP;
  if (isnan(Ki) || Ki < 0.0 || Ki > 1) Ki = DEFAULT_KI;
  if (isnan(Kd) || Kd < 0.0 || Kd > 100) Kd = DEFAULT_KD;
  myPID.setCoefficients(Kp, Ki, Kd);
  
  threshold_speed = cut_speed * threshold_ratio;

  analogReadResolution(14); // ADC at 14 bits

  // New: Initialize position history
  for (int i = 0; i < POSITION_HISTORY_SIZE; i++) {
    position_history[i].time = 0;
    position_history[i].position = 0;
  }
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
  // Load parameters from EEPROM
  EEPROM.get(EEPROM_SETPOINT_ADDR, Setpoint);
  Serial.println("Loaded Setpoint: " + String(Setpoint, 2));
  EEPROM.get(EEPROM_CORRECTION_FACTOR_ADDR, voltage_correction_factor);
  Serial.println("Loaded voltage_correction_factor: " + String(voltage_correction_factor, 2));
  EEPROM.get(EEPROM_CUT_SPEED_ADDR, cut_speed);
  Serial.println("Loaded cut_speed: " + String(cut_speed, 2));
  EEPROM.get(EEPROM_THRESHOLD_RATIO_ADDR, threshold_ratio);
  Serial.println("Loaded threshold_ratio: " + String(threshold_ratio, 2));
  EEPROM.get(EEPROM_KP_ADDR, Kp);
  Serial.println("Loaded Kp: " + String(Kp, 3));
  EEPROM.get(EEPROM_KI_ADDR, Ki);
  Serial.println("Loaded Ki: " + String(Ki, 4));
  EEPROM.get(EEPROM_KD_ADDR, Kd);
  Serial.println("Loaded Kd: " + String(Kd, 4));
}

void loop() {
  loopStartTime = micros();
  unsigned long currentTime = millis();

  // New: Record position history every POSITION_HISTORY_INTERVAL ms
  if (currentTime - last_position_record_time >= POSITION_HISTORY_INTERVAL) {
    position_history[position_history_index].time = currentTime;
    position_history[position_history_index].position = stepper.currentPosition();
    position_history_index = (position_history_index + 1) % POSITION_HISTORY_SIZE;
    last_position_record_time = currentTime;
  }

  // Button management to change screen
  static bool lastButtonState = HIGH;
  static unsigned long lastButtonTime = 0;
  static int prevScreen = -1; // To detect screen changes
  bool buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH && currentTime - lastButtonTime > 100) {
    prevScreen = currentScreen;
    currentScreen = (currentScreen + 1) % NB_SCREENS;
    lastButtonTime = currentTime;
  }
  lastButtonState = buttonState;

  // Detection of entry/exit to screen 2 for temporary factor
  if (currentScreen == 2 && prevScreen != 2) {
    temp_voltage_correction_factor = voltage_correction_factor;
    Serial.println("Entering temporary voltage_correction_factor adjustment mode");
  }
  if (prevScreen == 2 && currentScreen != 2) {
    if (abs(temp_voltage_correction_factor - voltage_correction_factor) > 0.001) {
      voltage_correction_factor = temp_voltage_correction_factor;
      EEPROM.put(EEPROM_CORRECTION_FACTOR_ADDR, voltage_correction_factor);
      last_eeprom_write = millis();
      Serial.println("Exiting adjustment mode: voltage_correction_factor saved = " + String(voltage_correction_factor, 2));
    }
  }
  prevScreen = currentScreen; // Update after management

  // Check serial commands for EEPROM reset
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "RESET_EEPROM") {
      EEPROM.put(EEPROM_SETPOINT_ADDR, DEFAULT_SETPOINT);
      EEPROM.put(EEPROM_CORRECTION_FACTOR_ADDR, DEFAULT_CORRECTION_FACTOR);
      EEPROM.put(EEPROM_CUT_SPEED_ADDR, DEFAULT_CUT_SPEED);
      EEPROM.put(EEPROM_THRESHOLD_RATIO_ADDR, DEFAULT_THRESHOLD_RATIO);
      EEPROM.put(EEPROM_KP_ADDR, DEFAULT_KP);
      EEPROM.put(EEPROM_KI_ADDR, DEFAULT_KI);
      EEPROM.put(EEPROM_KD_ADDR, DEFAULT_KD);
      initializedFlag = 0xAA;
      EEPROM.put(EEPROM_INITIALIZED_FLAG, initializedFlag);
      EEPROM.get(EEPROM_SETPOINT_ADDR, Setpoint);
      EEPROM.get(EEPROM_CORRECTION_FACTOR_ADDR, voltage_correction_factor);
      EEPROM.get(EEPROM_CUT_SPEED_ADDR, cut_speed);
      EEPROM.get(EEPROM_THRESHOLD_RATIO_ADDR, threshold_ratio);
      EEPROM.get(EEPROM_KP_ADDR, Kp);
      EEPROM.get(EEPROM_KI_ADDR, Ki);
      EEPROM.get(EEPROM_KD_ADDR, Kd);
      myPID.setCoefficients(Kp, Ki, Kd);
      threshold_speed = cut_speed * threshold_ratio;
      Serial.println("EEPROM reset via serial command");
    }
  }

  // Encoder management
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

  // Management of encoder changes
  static int lastEncoderPos = 0;
  int currentEncoderPos = encoderPos;
  int delta = currentEncoderPos - lastEncoderPos;
  lastEncoderPos = currentEncoderPos;

  // Screen management and parameter adjustments with EEPROM save
  static float lastSetpoint = Setpoint;
  static float lastCutSpeed = cut_speed;
  static float lastThresholdRatio = threshold_ratio;
  static float lastKp = Kp;
  static float lastKi = Ki;
  static float lastKd = Kd;

  switch (currentScreen) {
    case 1:
      Setpoint += delta;
      if (Setpoint < 50) Setpoint = 50;
      if (Setpoint > 200) Setpoint = 200;
      if (abs(Setpoint - lastSetpoint) > 0.01 && millis() - last_eeprom_write >= EEPROM_WRITE_INTERVAL) {
        EEPROM.put(EEPROM_SETPOINT_ADDR, Setpoint);
        lastSetpoint = Setpoint;
        last_eeprom_write = millis();
      }
      break;
    case 2:
      temp_voltage_correction_factor += delta * 0.01;
      if (temp_voltage_correction_factor < 0.5) temp_voltage_correction_factor = 0.5;
      if (temp_voltage_correction_factor > 2.0) temp_voltage_correction_factor = 2.0;
      // No save here, only on screen exit
      break;
    case 3:
      cut_speed += delta * CUT_SPEED_ADJUST_STEP;
      if (cut_speed < 0) cut_speed = 0;
      if (cut_speed > MAX_CUT_SPEED) cut_speed = MAX_CUT_SPEED;
      if (abs(cut_speed - lastCutSpeed) > 0.01 && millis() - last_eeprom_write >= EEPROM_WRITE_INTERVAL) {
        EEPROM.put(EEPROM_CUT_SPEED_ADDR, cut_speed);
        lastCutSpeed = cut_speed;
        last_eeprom_write = millis();
      }
      threshold_speed = cut_speed * threshold_ratio;
      break;
    case 4:
      threshold_ratio += delta * 0.1;
      if (threshold_ratio < 0) threshold_ratio = 0;
      if (threshold_ratio > 1) threshold_ratio = 1;
      if (abs(threshold_ratio - lastThresholdRatio) > 0.001 && millis() - last_eeprom_write >= EEPROM_WRITE_INTERVAL) {
        EEPROM.put(EEPROM_THRESHOLD_RATIO_ADDR, threshold_ratio);
        lastThresholdRatio = threshold_ratio;
        last_eeprom_write = millis();
      }
      threshold_speed = cut_speed * threshold_ratio;
      break;
    case 5:
      Kp += delta * 1.0;
      if (Kp < 0) Kp = 0;
      if (abs(Kp - lastKp) > 0.01 && millis() - last_eeprom_write >= EEPROM_WRITE_INTERVAL) {
        EEPROM.put(EEPROM_KP_ADDR, Kp);
        lastKp = Kp;
        last_eeprom_write = millis();
      }
      myPID.setCoefficients(Kp, Ki, Kd);
      break;
    case 6:
      Ki += delta * 0.5;
      if (Ki < 0) Ki = 0;
      if (abs(Ki - lastKi) > 0.0001 && millis() - last_eeprom_write >= EEPROM_WRITE_INTERVAL) {
        EEPROM.put(EEPROM_KI_ADDR, Ki);
        lastKi = Ki;
        last_eeprom_write = millis();
      }
      myPID.setCoefficients(Kp, Ki, Kd);
      break;
    case 7:
      Kd += delta * 0.5;
      if (Kd < 0) Kd = 0;
      if (abs(Kd - lastKd) > 0.0001 && millis() - last_eeprom_write >= EEPROM_WRITE_INTERVAL) {
        EEPROM.put(EEPROM_KD_ADDR, Kd);
        lastKd = Kd;
        last_eeprom_write = millis();
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
    bool arc_detected = (fast_voltage > arc_threshold);

    if (plasma_pin_low) {
      Serial.print("TGT / Torch speed: ");
      Serial.print(cut_speed);
      Serial.print(" / ");
      Serial.print(filtered_torch_speed);
      Serial.print(" ");
      Serial.print(SPEED_UNIT);
      Serial.print(" | Threshold speed: ");
      Serial.print(threshold_speed);
      Serial.print(" ");
      Serial.println(SPEED_UNIT);
      Serial.print("PLASMA_PIN: LOW | ENABLE_PIN: ");
      Serial.print(enable_pin_low ? "LOW" : "HIGH");
      Serial.print(" | Stabilization: ");
      Serial.print(plasma_stabilized ? "OK" : "Waiting");
      Serial.print(" | Arc detected: ");
      Serial.print(arc_detected ? "Yes" : "No");
      Serial.print(" | THC active: ");
      Serial.print(thc_active ? "Yes" : "No");
      Serial.print(" | THC_SIG: ");
      Serial.print(thc_off ? "ACTIVE" : "INACTIVE");
      Serial.print(" | THC state (speed): ");
      Serial.println(thc_speed_state ? "OK" : "Insufficient");
      Serial.print("V | Fast voltage: ");
      Serial.print(fast_voltage);
      Serial.print(" V | Slow voltage: ");
      Serial.print(slow_voltage);
      Serial.print(" V | Anti-dive: ");
      Serial.println(anti_dive_active ? "Active" : "Inactive");
 
      // Explicit log if THC inactive
      if (!thc_active) {
        Serial.print("Reason THC inactive: ");
        if (!thc_off) Serial.println("THC_OFF_PIN LOW");
        else if (!enable_pin_low) Serial.println("ENABLE_PIN HIGH");
        else if (!plasma_pin_low) Serial.println("PLASMA_PIN HIGH");
        else if (!plasma_stabilized) Serial.println("Stabilization not reached");
        else if (!arc_detected) Serial.println("Arc not detected (low voltage)");
        else if (!thc_speed_state) Serial.println("Torch speed < threshold");
        else if (anti_dive_active) Serial.println("Anti-dive active");
        else Serial.println("Unknown condition");
      }
      myPID.debug(&Serial, "myPID", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                                PRINT_OUTPUT   | // in the Serial plotter
                                                PRINT_SETPOINT |
                                                //PRINT_BIAS     |
                                                PRINT_P        |
                                                PRINT_I        |
                                                PRINT_D);
    } else {
      Serial.print("Cut Speed setup: ");
      Serial.print(cut_speed);
      Serial.print(" ");
      Serial.print(SPEED_UNIT);
      Serial.print(" | Fast voltage: ");
      Serial.print(fast_voltage);
      Serial.print(" V | PLASMA_PIN: HIGH | THC_SIG: ");
      Serial.print(thc_off ? "ACTIVE" : "INACTIVE");
      Serial.print(" p: " + String(Kp, 3));
      Serial.print(" i: " + String(Ki, 4));
      Serial.println(" d: " + String(Kd, 4));
      myPID.debug(&Serial, "myPID", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                                PRINT_OUTPUT   | // in the Serial plotter
                                                PRINT_SETPOINT |
                                                //PRINT_BIAS     |
                                                PRINT_P        |
                                                PRINT_I        |
                                                PRINT_D);
    }
    
    lastLogTime = currentTime;
  }
  
  if (currentTime - last_display_time >= display_interval) {
    updateLCD();
    last_display_time = currentTime;
  }

  static unsigned long lastPidTime = 0;
  unsigned long currentMicros = micros();
  if (currentMicros - lastPidTime >= 1000) { // 333 us = 3 kHz
    readAndFilterVoltage();  
    managePlasmaAndTHC();
    lastPidTime = currentMicros;
  }

  loopEndTime = micros();
  unsigned long loopExecutionTime = loopEndTime - loopStartTime;
  loopExecutionTimeSum += loopExecutionTime;
  loopCount++;

  if (currentTime - lastLoopLogTime >= LOOP_LOG_INTERVAL) {
    if (loopCount > 0) {
      unsigned long averageLoopTime = loopExecutionTimeSum / loopCount;
      float loopFrequency = 1000000.0 / averageLoopTime;
      Serial.print("Average execution time: ");
      Serial.print(averageLoopTime);
      Serial.print(" us | Frequency: ");
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

  float torch_speed = 0.0;
  unsigned long deltaTime = currentTime - lastSpeedTime;
  if (tempX + tempY >= MIN_STEPS && deltaTime > 0) {
    float distanceX = tempX * DIST_PER_STEP_X;
    float distanceY = tempY * DIST_PER_STEP_Y;
    float totalDistance = sqrt(distanceX * distanceX + distanceY * distanceY);
    torch_speed = (totalDistance / (deltaTime / 1000.0)) * 60.0 * SPEED_CORRECTION;
  }

  sum_speed_readings -= speed_readings[speed_reading_index];
  speed_readings[speed_reading_index] = torch_speed;
  sum_speed_readings += torch_speed;
  speed_reading_index = (speed_reading_index + 1) % speed_filter_size;
  filtered_torch_speed = sum_speed_readings / speed_filter_size;
  if (filtered_torch_speed < 0) filtered_torch_speed = 0.0;

  if (tempX + tempY == 0 && currentTime - lastStepTime > STEP_TIMEOUT) {
    totalStepX = 0;
    totalStepY = 0;
    for (int i = 0; i < speed_filter_size; i++) {
      speed_readings[i] = 0.0;
    }
    sum_speed_readings = 0.0;
    filtered_torch_speed = 0.0 + test_speed;
  }
  if (tempX + tempY > 0) lastStepTime = currentTime;

  lastSpeedTime = currentTime;
}

void readAndFilterVoltage() {
  // === WARM-UP ADC (1s) ===
  static unsigned long start_time = 0;
  static bool warmed_up = false;
  if (start_time == 0) start_time = millis();
  if (millis() - start_time >= 1000) warmed_up = true;

  // === 1x ADC NON-BLOQUANT : Accumule pour avg ultra-stable ===
  int reading = analogRead(PLASMA_VOLTAGE);
  float raw = (reading / 16383.0f) * 5.0f * DEFAULT_VOLTAGEDIVIDER + test_plasma_voltage;

  // === FAST : Oversample 10x + Low-pass (PID input) ===
  oversample_sum += raw;
  oversample_count++;
  if (oversample_count >= OVERSAMPLE_TARGET) {
    float avg_raw = oversample_sum / OVERSAMPLE_TARGET;
    const float INPUT_ALPHA = 0.7f;  // **Fort** : ±0.3V max
    uncorrected_fast = INPUT_ALPHA * avg_raw + (1.0f - INPUT_ALPHA) * last_pid_input;
    last_pid_input = uncorrected_fast;
    fast_voltage = uncorrected_fast * voltage_correction_factor;
    Input = fast_voltage;  // PID prêt !

    // Reset cycle
    oversample_sum = 0.0f;
    oversample_count = 0;
  }

  // === SLOW : Moyenne 200 + Low-pass (anti-dive ref) ===
  const int N_SLOW = 200;
  static float slow_samples[N_SLOW];
  static int slow_idx = 0;
  static float slow_sum = 0.0f;
  static bool slow_init = false;
  static float slow_lp = 0.0f;
  const float ALPHA_SLOW = 0.8f;

  if (!slow_init) {
    for (int i = 0; i < N_SLOW; i++) slow_samples[i] = raw;
    slow_sum = raw * N_SLOW;
    slow_init = true;
  }
  slow_sum -= slow_samples[slow_idx];
  slow_samples[slow_idx] = raw;
  slow_sum += raw;
  slow_idx = (slow_idx + 1) % N_SLOW;

  float slow_raw_avg = slow_sum / N_SLOW;
  uncorrected_slow = slow_raw_avg;
  slow_voltage = ALPHA_SLOW * (slow_raw_avg * voltage_correction_factor) + 
                 (1.0f - ALPHA_SLOW) * slow_lp;
  slow_lp = slow_voltage;

  // === ANTI-DIVE : SÉCURISÉ → UNIQUEMENT si THC ACTIVE ! ===
  static float voltage_at_activation = 0.0f;
  const float DROP_THRESHOLD = 5.0f;
  const float RETURN_THRESHOLD = 3.0f;
  const unsigned long MAX_ANTI_DIVE_DURATION = 1000;  // 1s max
  static bool last_anti_dive_state = false;

  // **FIX CRITIQUE** : && thc_active (THC_SIG ACTIVE)
  if (warmed_up && 
      fast_voltage > slow_voltage + DROP_THRESHOLD && 
      !anti_dive_active && 
      digitalRead(PLASMA_PIN) == LOW && 
      plasma_stabilized && 
      thc_active) {  // ← **SÉCURITÉ CAM**
    
    anti_dive_active = true;
    just_anti_dive_activated = true;
    anti_dive_start_time = millis();
    voltage_at_activation = slow_voltage;
    
    Serial.print("**Anti-dive ON (THC ACTIVE)** | Cut: ");
    Serial.print(fast_voltage, 1);
    Serial.print("V | Slow: ");
    Serial.print(slow_voltage, 1);
    Serial.print("V | Saved: ");
    Serial.println(voltage_at_activation, 1);
  }

  if (anti_dive_active) {
    bool deactivate = (fast_voltage <= voltage_at_activation + RETURN_THRESHOLD) ||
                      (millis() - anti_dive_start_time >= MAX_ANTI_DIVE_DURATION);
    if (deactivate) {
      anti_dive_active = false;
      just_anti_dive_activated = false;
      Serial.print("**Anti-dive OFF** | Cut: ");
      Serial.print(fast_voltage, 1);
      Serial.print("V | Slow: ");
      Serial.println(slow_voltage, 1);
    }
  }
  last_anti_dive_state = anti_dive_active;
}

void managePlasmaAndTHC() {
  bool plasma_pin_low = (digitalRead(PLASMA_PIN) == LOW);
  bool enable_pin_low = (digitalRead(ENABLE_PIN) == LOW);
  bool arc_detected = (fast_voltage > arc_threshold);
  bool thc_off = (digitalRead(THC_OFF_PIN) == HIGH);
  unsigned long currentTime = millis();

  if (plasma_pin_low) {
    digitalWrite(SWITCH1, LOW);
    digitalWrite(SWITCH2, LOW);
  } else {
    digitalWrite(SWITCH1, HIGH);
    digitalWrite(SWITCH2, HIGH);
  }

  static bool last_plasma_stabilized = false;
  if (plasma_pin_low && !plasma_stabilized) {
    if (plasma_active_time == 0) plasma_active_time = millis();
    if (millis() - plasma_active_time >= stabilization_delay) {
      plasma_stabilized = true;
      if (!last_plasma_stabilized) {
        myPID.reset(); // Reset I and D terms
        Serial.println("PID reset (new cut - plasma stabilized)");
      }
    }
  } else if (!plasma_pin_low) {
    plasma_active_time = 0;
    plasma_stabilized = false;
  }
  last_plasma_stabilized = plasma_stabilized;

  if (!thc_speed_state && filtered_torch_speed >= threshold_speed) {
    thc_speed_state = true;
  } else if (thc_speed_state && filtered_torch_speed < threshold_speed) {
    thc_speed_state = false;
  }

  // Determine THC state
  bool thc_active_new = thc_off && enable_pin_low && plasma_pin_low && plasma_stabilized && arc_detected && thc_speed_state && !anti_dive_active;

  // Management of PID start and stop
  static bool last_thc_active = false;
  if (thc_active_new != last_thc_active) {
    if (thc_active_new) {
      myPID.start(); // Start PID when thc_active becomes true
      myPID.reset(); // Reset I and D terms at start
      Serial.println("PID started and reset (THC inactive to active)");
    } else {
      myPID.stop();  // Stop PID when thc_active becomes false
      Serial.println("PID stopped (THC active to inactive)");
    }
    last_thc_active = thc_active_new;
  }
  thc_active = thc_active_new;

  static double smoothedOutput = 0.0;
  const float alpha = 0.5; // Smoothing factor

  if (anti_dive_active) {
    if (just_anti_dive_activated) {
      // Compute target position from 1 sec ago + bonus
      unsigned long target_time = currentTime - 1000;
      long closest_pos = stepper.currentPosition();
      unsigned long closest_diff = ULONG_MAX;
      for (int i = 0; i < POSITION_HISTORY_SIZE; i++) {
        unsigned long h_time = position_history[i].time;
        if (h_time == 0) continue; // Not initialized
        unsigned long diff = abs((long)target_time - (long)h_time);
        if (diff < closest_diff) {
          closest_diff = diff;
          closest_pos = position_history[i].position;
        }
      }
      long target_pos = closest_pos + ANTI_DIVE_BONUS_STEPS; // Positive for up
      stepper.setMaxSpeed(ANTI_DIVE_LIFT_SPEED);
      stepper.setAcceleration(ANTI_DIVE_LIFT_ACCEL);
      stepper.moveTo(target_pos);
      Serial.print("Anti-dive lift to position: ");
      Serial.print(target_pos);
      Serial.println(" steps");
      just_anti_dive_activated = false;  // Reset after handling
    }
    // Execute position move during anti-dive
    stepper.run();
  } else if (thc_active) {
    myPID.compute(); // Compute output only if thc_active is true
    double error = Setpoint - Input;
    if (abs(error) > 1) { // Dead zone of ±1 V
      smoothedOutput = alpha * Output + (1 - alpha) * smoothedOutput;
      stepper.setSpeed(Output);
      stepper.runSpeed();
    } else {
      smoothedOutput = 0.0;
      Output = 0.0;
      stepper.setSpeed(0);
    }
  } else {
    smoothedOutput = 0.0;
    stepper.setSpeed(0);
  }
}

void updateLCD() {
  static int lastScreen = -1;
  static float lastActualVoltage = -1;
  static float lastSetpoint = -1;
  static int lastSpeed = -1;
  static bool lastThcActive = false;
  static double lastOutput = 0;
  static bool lastEnableLow = false;
  static bool lastPlasmaLow = false;
  static bool lastThcOff = false;
  static float lastCutSpeed = -1;
  static float lastTempCorrectionFactor = -1;
  static float lastUncorrectedFast = -1;
  static float lastAdjustedVoltage = -1;

  if (currentScreen != lastScreen) {
    lcd.clear();
    switch (currentScreen) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("Act:     V");
        lcd.setCursor(0, 1);
        lcd.print("Tgt:     V");
        break;
      case 1:
        lcd.setCursor(0, 0);
        lcd.print("Set V:      V");
        break;
      case 2:
        lcd.setCursor(0, 0);
        lcd.print("V Corr:     ");
        lcd.setCursor(0, 1);
        lcd.print("Base:    Adj:   ");
        break;
      case 3:
        lcd.setCursor(0, 0);
        lcd.print("CutSpd:     ");
        lcd.setCursor(0, 1);
        lcd.print("Unit: ");
        lcd.print(SPEED_UNIT_LONG);
        break;
      case 4:
        lcd.setCursor(0, 0);
        lcd.print("Spd Ths %:  ");
        break;
      case 5:
        lcd.setCursor(0, 0);
        lcd.print("PID Kp:      ");
        break;
      case 6:
        lcd.setCursor(0, 0);
        lcd.print("PID Ki:      ");
        break;
      case 7:
        lcd.setCursor(0, 0);
        lcd.print("PID Kd:      ");
        break;
    }
    lastScreen = currentScreen;
    lastActualVoltage = -1;
    lastSetpoint = -1;
    lastSpeed = -1;
    lastThcActive = false;
    lastOutput = 0;
    lastEnableLow = false;
    lastPlasmaLow = false;
    lastThcOff = false;
    lastCutSpeed = -1;
    lastTempCorrectionFactor = -1;
    lastUncorrectedFast = -1;
    lastAdjustedVoltage = -1;
  }

  if (currentScreen == 0) {
    if (fast_voltage != lastActualVoltage) {
      lcd.setCursor(4, 0);
      lcd.print(fast_voltage, 1);
      lastActualVoltage = fast_voltage;
    }

    if (Setpoint != lastSetpoint) {
      lcd.setCursor(4, 1);
      lcd.print(Setpoint, 1);
      lastSetpoint = Setpoint;
    }

    int displayed_speed = (int)(filtered_torch_speed);
    if (displayed_speed > 9999) displayed_speed = 9999;
    if (filtered_torch_speed < 0.1) {
      if (lastSpeed != 0) {
        lcd.setCursor(12, 0);
        lcd.print("   0");
        lastSpeed = 0;
      }
    } else if (displayed_speed != lastSpeed) {
      lcd.setCursor(12, 0);
      char buffer[5];
      snprintf(buffer, sizeof(buffer), "%4d", displayed_speed); // Right alignment
      lcd.print(buffer);
      lastSpeed = displayed_speed;
    }

    bool enable_low = digitalRead(ENABLE_PIN) == LOW;
    bool plasma_low = (digitalRead(PLASMA_PIN) == LOW);
    bool thc_off = (digitalRead(THC_OFF_PIN) == HIGH);

    if (enable_low != lastEnableLow || plasma_low != lastPlasmaLow || 
        thc_active != lastThcActive || thc_off != lastThcOff || Output != lastOutput) {
      
      lcd.setCursor(11, 1);
      if (enable_low) {
        lcd.print("S");
      } else {
        lcd.print("O");
      }

      lcd.setCursor(12, 1);
      if (thc_active) {
        if (Output > 10) {
          lcd.write(3);
        } else if (Output < -10) {
          lcd.write(4);
        } else {
          lcd.write(5);
        }
      } else {
        lcd.print(" ");
      }

      lcd.setCursor(13, 1);
      if (thc_off && !thc_active) {
        lcd.print("o");
      } else if (thc_off && thc_active) {
        lcd.write(2);
      } else {
        lcd.print("-");
      }

      lcd.setCursor(14, 1);
      if (plasma_low) {
        lcd.write(1);
      } else {
        lcd.print(" ");
      }

      lastEnableLow = enable_low;
      lastPlasmaLow = plasma_low;
      lastThcActive = thc_active;
      lastThcOff = thc_off;
      lastOutput = Output;
    }
  } else {
    switch (currentScreen) {
      case 1:
        lcd.setCursor(7, 0);
        lcd.print(Setpoint, 1);
        break;
      case 2: {  // Scoped block to fix "crosses initialization" error
        if (temp_voltage_correction_factor != lastTempCorrectionFactor) {
          lcd.setCursor(8, 0);
          lcd.print(temp_voltage_correction_factor, 2);
          lastTempCorrectionFactor = temp_voltage_correction_factor;
        }
        float preview_adj = uncorrected_fast * temp_voltage_correction_factor;
        if (uncorrected_fast != lastUncorrectedFast || preview_adj != lastAdjustedVoltage) {
          lcd.setCursor(5, 1);
          lcd.print(uncorrected_fast, 1);
          lcd.setCursor(13, 1);
          lcd.print(preview_adj, 1);
          lastUncorrectedFast = uncorrected_fast;
          lastAdjustedVoltage = preview_adj;
        }
        break;
      }
      case 3:
        if (cut_speed != lastCutSpeed) {
          lcd.setCursor(8, 0);
          char buffer[5];
          snprintf(buffer, sizeof(buffer), "%4.0f", cut_speed); // Right alignment
          lcd.print(buffer);
          lastCutSpeed = cut_speed;
        }
        break;
      case 4:
        lcd.setCursor(11, 0);
        lcd.print(threshold_ratio, 1);
        break;
      case 5:
        lcd.setCursor(8, 0);
        lcd.print(Kp, 3);
        break;
      case 6:
        lcd.setCursor(8, 0);
        lcd.print(Ki, 4);
        break;
      case 7:
        lcd.setCursor(8, 0);
        lcd.print(Kd, 3);
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