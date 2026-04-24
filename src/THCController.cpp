/**
 * SmartTHC - THC Controller
 *
 * THC logic implementation
 */

#include "THCController.h"

THCController::THCController()
    : stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN)
    , Setpoint(DEFAULT_SETPOINT)
    , Input(0)
    , Output(0)
    , Kp(DEFAULT_KP)
    , Ki(DEFAULT_KI)
    , Kd(DEFAULT_KD)
    , voltageCorrectionFactor(DEFAULT_CORRECTION_FACTOR)
    , tempVoltageCorrectionFactor(DEFAULT_CORRECTION_FACTOR)
    , fastVoltage(0.0f)
    , slowVoltage(0.0f)
    , uncorrectedFast(0.0f)
    , uncorrectedSlow(0.0f)
    , oversampleSum(0.0f)
    , oversampleCount(0)
    , lastPidInput(0.0f)
    , slowIdx(0)
    , slowSum(0.0f)
    , slowInit(false)
    , slowLp(0.0f)
    , slowGateWasOn(false)
    , plasmaPinLow(false)
    , enablePinLow(false)
    , arcDetected(false)
    , thcOff(false)
    , plasmaStabilized(false)
    , plasmaActiveTime(0)
    , thcActive(false)
    , lastThcActive(false)
    , speedMonitor(nullptr)
    , prevThcOff(false)
    , thcOnStabilized(true)
    , thcOnTransitionTime(0)
    , cutMotionGateReady(false)
    , antiDiveActive(false)
    , antiDiveStartTime(0)
    , voltageAtActivation(0.0f)
    , justAntiDiveActivated(false)
    , smoothedOutput(0.0)
    , adcWarmedUp(false)
    , adcStartTime(0)
{
    // Initialize slowSamples array
    for (int i = 0; i < N_SLOW; i++) {
        slowSamples[i] = 0.0f;
    }
}

void THCController::begin() {
    // Configure pins
    pinMode(PLASMA_PIN, INPUT);
    pinMode(ENABLE_PIN, INPUT_PULLUP);
    pinMode(THC_OFF_PIN, INPUT);
    pinMode(SWITCH1, OUTPUT);
    pinMode(SWITCH2, OUTPUT);
    pinMode(PLASMA_VOLTAGE, INPUT);

    // Initial switch state
    digitalWrite(SWITCH1, HIGH);
    digitalWrite(SWITCH2, HIGH);

    // Configure motor
    stepper.setMaxSpeed(STEPPER_MAX_SPEED);
    stepper.setAcceleration(STEPPER_ACCELERATION);

    // Configure PID
    pid.begin(&Input, &Output, &Setpoint, Kp, Ki, Kd);
    pid.setOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    pid.setWindUpLimits(PID_WINDUP_MIN, PID_WINDUP_MAX);

    // ADC resolution
    analogReadResolution(14);

    // Start ADC warm-up
    adcStartTime = millis();
}

void THCController::update(unsigned long currentTime) {
    // ADC warm-up
    if (!adcWarmedUp) {
        if (currentTime - adcStartTime >= 1000) {
            adcWarmedUp = true;
        }
    }

    // Read and filter voltage
    readAndFilterVoltage(currentTime);

    // Update anti-dive state
    updateAntiDive(currentTime);

    // Update plasma state
    updatePlasmaState(currentTime);

    // Update THC state
    updateTHCState(currentTime);

    // Control motor
    controlMotor(currentTime);
}

void THCController::runMotor() {
    stepper.run();
}

void THCController::setSpeedMonitor(SpeedMonitor* monitor) {
    speedMonitor = monitor;
}

void THCController::readAndFilterVoltage(unsigned long currentTime) {
    // Raw ADC reading
    int reading = analogRead(PLASMA_VOLTAGE);
    float raw = (reading / 16383.0f) * 5.0f * DEFAULT_VOLTAGEDIVIDER;

    // === FAST: Oversample 10x + Low-pass (PID input) ===
    // Always runs so arc detection still sees the ADC rise when plasma
    // re-ignites.
    oversampleSum += raw;
    oversampleCount++;

    if (oversampleCount >= OVERSAMPLE_TARGET) {
        float avgRaw = oversampleSum / OVERSAMPLE_TARGET;
        uncorrectedFast = INPUT_ALPHA * avgRaw + (1.0f - INPUT_ALPHA) * lastPidInput;
        lastPidInput = uncorrectedFast;
        fastVoltage = uncorrectedFast * voltageCorrectionFactor;
        Input = fastVoltage;

        oversampleSum = 0.0f;
        oversampleCount = 0;
    }

    // Arc detection from new fast voltage.
    arcDetected = (fastVoltage > ARC_THRESHOLD);

    // === SLOW filter gating ===
    // The slow filter (200-sample avg + LP, used as the anti-dive reference)
    // must only advance while the plasma is physically cutting. Gating on
    // arcDetected alone is not enough: ARC_THRESHOLD is 10 V, and after
    // PLASMA_PIN goes HIGH the inductive tail of the cut head keeps
    // fastVoltage above 10 V for several hundred ms, polluting the buffer
    // with garbage readings (seen in the field: slow frozen at 31.8 V
    // instead of ~122 V). Gate on the GCode-commanded plasma pin as well,
    // and re-seed on every off→on transition so the anti-dive reference
    // starts from a genuine arc reading.
    //
    // plasmaPinLow is refreshed in updatePlasmaState which runs after this
    // function, so we read its previous-iteration value here (<1 ms stale
    // at 1 kHz, acceptable).
    const bool gateNow = plasmaPinLow && arcDetected;

    if (gateNow) {
        if (!slowInit || !slowGateWasOn) {
            for (int i = 0; i < N_SLOW; i++) {
                slowSamples[i] = raw;
            }
            slowSum = raw * N_SLOW;
            slowLp = raw * voltageCorrectionFactor;
            slowIdx = 0;
            slowInit = true;
        } else {
            slowSum -= slowSamples[slowIdx];
            slowSamples[slowIdx] = raw;
            slowSum += raw;
            slowIdx = (slowIdx + 1) % N_SLOW;
        }

        float slowRawAvg = slowSum / N_SLOW;
        uncorrectedSlow = slowRawAvg;
        slowVoltage = ALPHA_SLOW * (slowRawAvg * voltageCorrectionFactor) +
                      (1.0f - ALPHA_SLOW) * slowLp;
        slowLp = slowVoltage;
    }
    // When the gate is off, slow* values are held at their last on-arc
    // value — better than having them drift toward ambient noise.

    slowGateWasOn = gateNow;
}

void THCController::updateAntiDive(unsigned long currentTime) {
    static bool lastAntiDiveState = false;

    // Anti-dive activation
    if (adcWarmedUp &&
        fastVoltage > slowVoltage + DROP_THRESHOLD &&
        !antiDiveActive &&
        plasmaPinLow &&
        plasmaStabilized &&
        thcActive) {

        antiDiveActive = true;
        justAntiDiveActivated = true;
        antiDiveStartTime = currentTime;
        voltageAtActivation = slowVoltage;
    }

    // Anti-dive deactivation
    if (antiDiveActive) {
        bool shouldDeactivate = (fastVoltage <= voltageAtActivation + RETURN_THRESHOLD) ||
                               (currentTime - antiDiveStartTime >= MAX_ANTI_DIVE_DURATION);

        if (shouldDeactivate) {
            antiDiveActive = false;
            justAntiDiveActivated = false;
        }
    }

    lastAntiDiveState = antiDiveActive;
}

void THCController::updatePlasmaState(unsigned long currentTime) {
    // Read inputs
    plasmaPinLow = (digitalRead(PLASMA_PIN) == LOW);
    enablePinLow = (digitalRead(ENABLE_PIN) == LOW);
    thcOff = (digitalRead(THC_OFF_PIN) == HIGH);

    // Control switches
    if (plasmaPinLow) {
        digitalWrite(SWITCH1, LOW);
        digitalWrite(SWITCH2, LOW);
    } else {
        digitalWrite(SWITCH1, HIGH);
        digitalWrite(SWITCH2, HIGH);
    }

    // Stabilization handling
    static bool lastPlasmaStabilized = false;

    if (plasmaPinLow && !plasmaStabilized) {
        if (plasmaActiveTime == 0) {
            plasmaActiveTime = currentTime;
        }
        if (currentTime - plasmaActiveTime >= STABILIZATION_DELAY) {
            plasmaStabilized = true;
            if (!lastPlasmaStabilized) {
                pid.reset();
            }
        }
    } else if (!plasmaPinLow) {
        plasmaActiveTime = 0;
        plasmaStabilized = false;
    }

    lastPlasmaStabilized = plasmaStabilized;
}

void THCController::updateTHCState(unsigned long currentTime) {
    // Detect THC_OFF -> THC_ON transition (thcOff goes from false to true)
    if (thcOff && !prevThcOff) {
        // THC just re-enabled: enforce re-stabilization delay
        thcOnStabilized = false;
        thcOnTransitionTime = currentTime;
    }
    if (!thcOff) {
        thcOnStabilized = false;
    }
    // Check if re-stabilization delay has elapsed
    if (!thcOnStabilized && thcOff &&
        (currentTime - thcOnTransitionTime >= THC_ON_RESTAB_DELAY)) {
        thcOnStabilized = true;
    }
    prevThcOff = thcOff;

    cutMotionGateReady = true;
    if (speedMonitor != nullptr) {
        if (currentTime >= THC_AFTER_CUT_START_DELAY) {
            unsigned long requiredCutStartTime = currentTime - THC_AFTER_CUT_START_DELAY;
            cutMotionGateReady = speedMonitor->hasCutMotionStableSince(requiredCutStartTime);
        } else {
            cutMotionGateReady = false;
        }
    }

    // Determine new THC state (thcOnStabilized prevents immediate activation)
    bool thcActiveNew = thcOff && thcOnStabilized && enablePinLow &&
                        plasmaPinLow && plasmaStabilized && arcDetected && cutMotionGateReady;

    // PID start/stop handling
    if (thcActiveNew != lastThcActive) {
        if (thcActiveNew) {
            pid.start();
            pid.reset();
        } else {
            pid.stop();
        }
        lastThcActive = thcActiveNew;
    }

    thcActive = thcActiveNew;
}

void THCController::controlMotor(unsigned long currentTime) {
    if (antiDiveActive) {
        performAntiDiveLift(currentTime);
    } else if (thcActive) {
        normalTHCControl();
    } else {
        smoothedOutput = 0.0;
        stepper.setSpeed(0);
    }
}

void THCController::performAntiDiveLift(unsigned long currentTime) {
    if (justAntiDiveActivated) {
        // Calculate target position (1 second back + bonus)
        // Note: history must be provided by caller
        justAntiDiveActivated = false;
    }

    stepper.run();
}

void THCController::normalTHCControl() {
    pid.compute();

    double error = Setpoint - Input;
    const float alpha = 0.5f; // Smoothing factor

    if (abs(error) > STEPPER_DEADZONE) {
        smoothedOutput = alpha * Output + (1.0 - alpha) * smoothedOutput;
        stepper.setSpeed(Output);
        stepper.runSpeed();
    } else {
        smoothedOutput = 0.0;
        Output = 0.0;
        stepper.setSpeed(0);
    }
}

void THCController::setSetpoint(float value) {
    if (value >= SETPOINT_MIN && value <= SETPOINT_MAX) {
        Setpoint = value;
    }
}

void THCController::setKp(double value) {
    if (value >= 0 && value <= KP_MAX) {
        Kp = value;
        pid.setCoefficients(Kp, Ki, Kd);
    }
}

void THCController::setKi(double value) {
    if (value >= 0 && value <= KI_MAX) {
        Ki = value;
        pid.setCoefficients(Kp, Ki, Kd);
    }
}

void THCController::setKd(double value) {
    if (value >= 0 && value <= KD_MAX) {
        Kd = value;
        pid.setCoefficients(Kp, Ki, Kd);
    }
}

void THCController::setCorrectionFactor(float value) {
    if (value >= CORRECTION_FACTOR_MIN && value <= CORRECTION_FACTOR_MAX) {
        voltageCorrectionFactor = value;
    }
}

AntiDiveEvent THCController::getAntiDiveEvent() const {
    AntiDiveEvent event;
    event.active = antiDiveActive;
    event.startTime = antiDiveStartTime;
    event.justActivated = justAntiDiveActivated;
    return event;
}

void THCController::clearAntiDiveJustActivated() {
    justAntiDiveActivated = false;
}

void THCController::reset() {
    pid.reset();
    stepper.setSpeed(0);
    smoothedOutput = 0.0;
}

float THCController::readRawVoltage() {
    int reading = analogRead(PLASMA_VOLTAGE);
    return (reading / 16383.0f) * 5.0f * DEFAULT_VOLTAGEDIVIDER;
}

void THCController::resetSlowFilter(float initialValue) {
    for (int i = 0; i < N_SLOW; i++) {
        slowSamples[i] = initialValue;
    }
    slowSum = initialValue * N_SLOW;
    slowLp = initialValue;
}
