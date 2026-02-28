/**
 * SmartTHC - Contrôleur THC
 * 
 * Gère la logique THC : lecture tension, PID, anti-dive, contrôle moteur Z
 */

#ifndef THC_CONTROLLER_H
#define THC_CONTROLLER_H

#include <Arduino.h>
#include <AccelStepper.h>
#include <ArduPID.h>
#include "Config.h"
#include "SpeedMonitor.h"

// Événement anti-dive pour l'affichage
struct AntiDiveEvent {
    bool active;
    unsigned long startTime;
    bool justActivated;
    
    AntiDiveEvent() : active(false), startTime(0), justActivated(false) {}
};

class THCController {
public:
    THCController();
    
    // Initialisation
    void begin();
    
    // Mise à jour principale (appelée à 1kHz)
    void update(unsigned long currentTime);
    
    // Mise à jour du moteur (appelée aussi souvent que possible)
    void runMotor();
    
    // Configuration PID
    void setSetpoint(float value);
    void setKp(double value);
    void setKi(double value);
    void setKd(double value);
    void setCorrectionFactor(float value);
    
    // Accesseurs
    float getSetpoint() const { return (float)Setpoint; }
    double getKp() const { return Kp; }
    double getKi() const { return Ki; }
    double getKd() const { return Kd; }
    float getCorrectionFactor() const { return voltageCorrectionFactor; }
    float getFastVoltage() const { return fastVoltage; }
    float getSlowVoltage() const { return slowVoltage; }
    float getUncorrectedFast() const { return uncorrectedFast; }
    double getPidOutput() const { return Output; }
    
    // États
    bool isPlasmaActive() const { return plasmaPinLow; }
    bool isPlasmaStabilized() const { return plasmaStabilized; }
    bool isArcDetected() const { return arcDetected; }
    bool isTHCActive() const { return thcActive; }
    bool isAntiDiveActive() const { return antiDiveActive; }
    bool isEnableLow() const { return enablePinLow; }
    bool isTHCOff() const { return thcOff; }
    
    // Événement anti-dive pour l'affichage
    AntiDiveEvent getAntiDiveEvent() const;
    void clearAntiDiveJustActivated();
    
    // Position moteur
    long getMotorPosition() { return stepper.currentPosition(); }
    void setMotorPosition(long position) { stepper.setCurrentPosition(position); }
    
    // Reset
    void reset();

private:
    // Moteur
    AccelStepper stepper;
    
    // PID
    ArduPID pid;
    double Setpoint;
    double Input;
    double Output;
    double Kp;
    double Ki;
    double Kd;
    
    // Paramètres
    float voltageCorrectionFactor;
    float tempVoltageCorrectionFactor;
    
    // Tensions
    float fastVoltage;
    float slowVoltage;
    float uncorrectedFast;
    float uncorrectedSlow;
    
    // Oversampling
    float oversampleSum;
    uint8_t oversampleCount;
    float lastPidInput;
    
    // Slow filter
    float slowSamples[N_SLOW];
    int slowIdx;
    float slowSum;
    bool slowInit;
    float slowLp;
    
    // État plasma
    bool plasmaPinLow;
    bool enablePinLow;
    bool arcDetected;
    bool thcOff;
    bool plasmaStabilized;
    unsigned long plasmaActiveTime;
    
    // État THC
    bool thcActive;
    bool lastThcActive;
    
    // Anti-dive
    bool antiDiveActive;
    unsigned long antiDiveStartTime;
    float voltageAtActivation;
    bool justAntiDiveActivated;
    
    // Smoothing
    double smoothedOutput;
    
    // Warm-up
    bool adcWarmedUp;
    unsigned long adcStartTime;
    
    // Méthodes internes
    void readAndFilterVoltage(unsigned long currentTime);
    void updateAntiDive(unsigned long currentTime);
    void updatePlasmaState(unsigned long currentTime);
    void updateTHCState(unsigned long currentTime);
    void controlMotor(unsigned long currentTime);
    void performAntiDiveLift(unsigned long currentTime);
    void normalTHCControl();
    
    float readRawVoltage();
    void resetSlowFilter(float initialValue);
};

#endif // THC_CONTROLLER_H
