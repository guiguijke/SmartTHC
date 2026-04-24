/**
 * SmartTHC - THC Controller
 *
 * Handles THC logic: voltage reading, PID, anti-dive, Z motor control
 */

#ifndef THC_CONTROLLER_H
#define THC_CONTROLLER_H

#include <Arduino.h>
#include <AccelStepper.h>
#include <ArduPID.h>
#include "Config.h"
#include "SpeedMonitor.h"

// Anti-dive event for display notification
struct AntiDiveEvent {
    bool active;
    unsigned long startTime;
    bool justActivated;

    AntiDiveEvent() : active(false), startTime(0), justActivated(false) {}
};

class THCController {
public:
    THCController();

    // Initialization
    void begin();

    // Main update (called at 1kHz)
    void update(unsigned long currentTime);
    void setSpeedMonitor(SpeedMonitor* monitor);

    // Motor update (called as often as possible)
    void runMotor();

    // PID configuration
    void setSetpoint(float value);
    void setKp(double value);
    void setKi(double value);
    void setKd(double value);
    void setCorrectionFactor(float value);

    // Getters
    float getSetpoint() const { return (float)Setpoint; }
    double getKp() const { return Kp; }
    double getKi() const { return Ki; }
    double getKd() const { return Kd; }
    float getCorrectionFactor() const { return voltageCorrectionFactor; }
    float getFastVoltage() const { return fastVoltage; }
    float getSlowVoltage() const { return slowVoltage; }
    float getUncorrectedFast() const { return uncorrectedFast; }
    double getPidOutput() const { return Output; }

    // States
    bool isPlasmaActive() const { return plasmaPinLow; }
    bool isPlasmaStabilized() const { return plasmaStabilized; }
    bool isArcDetected() const { return arcDetected; }
    bool isTHCActive() const { return thcActive; }
    bool isAntiDiveActive() const { return antiDiveActive; }
    bool isEnableLow() const { return enablePinLow; }
    bool isTHCOff() const { return thcOff; }
    bool isThcOnReStabilized() const { return thcOnStabilized; }
    bool isCutMotionGateReady() const { return cutMotionGateReady; }

    // Anti-dive event for display
    AntiDiveEvent getAntiDiveEvent() const;
    void clearAntiDiveJustActivated();

    // Motor position
    long getMotorPosition() { return stepper.currentPosition(); }
    void setMotorPosition(long position) { stepper.setCurrentPosition(position); }

    // Reset
    void reset();

private:
    // Motor
    AccelStepper stepper;

    // PID
    ArduPID pid;
    double Setpoint;
    double Input;
    double Output;
    double Kp;
    double Ki;
    double Kd;

    // Parameters
    float voltageCorrectionFactor;
    float tempVoltageCorrectionFactor;

    // Voltages
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

    // Plasma state
    bool plasmaPinLow;
    bool enablePinLow;
    bool arcDetected;
    bool thcOff;
    bool plasmaStabilized;
    unsigned long plasmaActiveTime;

    // THC state
    bool thcActive;
    bool lastThcActive;
    SpeedMonitor* speedMonitor;

    // Re-stabilization after THC_OFF -> THC_ON
    bool prevThcOff;                    // previous thcOff state
    bool thcOnStabilized;               // true when re-stab delay has elapsed
    unsigned long thcOnTransitionTime;   // transition timestamp

    // Cut-motion gate (last computed value, exposed for diagnostics)
    bool cutMotionGateReady;

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

    // Internal methods
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
