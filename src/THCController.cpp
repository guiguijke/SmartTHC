/**
 * SmartTHC - THC Controller
 *
 * THC logic implementation
 */

#include "THCController.h"
#include <stdlib.h>  // qsort for trimmed average

// qsort comparator for float array
static int compareFloat(const void* a, const void* b) {
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

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
    , lastFastInput(0.0f)
    , fastAverageIdx(0)
    , fastAverageReady(false)
    , slowAverageIdx(0)
    , slowAverageSum(0.0f)
    , slowAverageReady(false)
    , lastSlowSampleTime(0)
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
    , lastPlasmaStabilized(false)
    , plasmaActiveTime(0)
    , thcActive(false)
    , lastThcActive(false)
    , speedMonitor(nullptr)
    , prevThcOff(false)
    , thcOnStabilized(true)
    , thcOnTransitionTime(0)
    , cutMotionGateReady(false)
    , antiDiveActive(false)
    , antiDivePending(false)
    , antiDivePendingStartTime(0)
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

    // Initialize fast-filter rolling average
    for (int i = 0; i < FAST_AVERAGE_SIZE; i++) {
        fastAverageSamples[i] = 0.0f;
    }

    // Initialize slow-filter rolling average
    for (int i = 0; i < SLOW_AVERAGE_SIZE; i++) {
        slowAverageSamples[i] = 0.0f;
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

    // Configure motor — Z_DIR_INVERT flips the DIR output at the pin level so
    // both the PID (runSpeed) and the anti-dive lift (moveTo) agree on which
    // physical direction is "up". The firmware semantics always treat positive
    // = up; this flag is the single boundary where wiring polarity is
    // reconciled.
    stepper.setPinsInverted(Z_DIR_INVERT, false, false);
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
    // The motor is driven by two different AccelStepper modes depending on
    // state, and they MUST NOT both pump steps at the same time:
    //
    //   - During normal cutting, normalTHCControl() calls runSpeed() at 1 kHz
    //     to step the motor at the PID-commanded constant speed.
    //   - During anti-dive, we instead command a position move via moveTo()
    //     and need run() pumped as fast as possible so the acceleration
    //     profile resolves smoothly within the dive window.
    //
    // run() and runSpeed() share the stepper's internal _speed state; calling
    // run() during normal cutting would fight runSpeed() and produce step
    // glitching. Gating run() on antiDiveActive keeps the two regimes clean.
    if (antiDiveActive) {
        stepper.run();
    }
}

void THCController::setSpeedMonitor(SpeedMonitor* monitor) {
    speedMonitor = monitor;
}

void THCController::reseedSlowFilter(float avgRaw) {
    for (int i = 0; i < N_SLOW; i++) {
        slowSamples[i] = avgRaw;
    }
    slowSum = avgRaw * N_SLOW;
    slowLp = avgRaw * voltageCorrectionFactor;
    slowIdx = 0;
    slowInit = true;
}

void THCController::readAndFilterVoltage(unsigned long currentTime) {
    // Raw ADC reading — one sample per PID tick (1 kHz)
    int reading = analogRead(PLASMA_VOLTAGE);
    float raw = (reading / 16383.0f) * 5.0f * DEFAULT_VOLTAGEDIVIDER;

    // === FAST filter trimmed rolling average + EMA (PID input) ===
    // Maintain a rolling window of raw ADC values. Once full, reject the
    // FAST_AVERAGE_TRIM smallest and largest samples (outliers) and average
    // the rest. This is far more robust to impulse noise than a plain mean.
    fastAverageSamples[fastAverageIdx] = raw;
    fastAverageIdx = (fastAverageIdx + 1) % FAST_AVERAGE_SIZE;

    if (fastAverageIdx == 0) {
        fastAverageReady = true;
    }

    float fastRaw = raw;
    if (fastAverageReady) {
        float sorted[FAST_AVERAGE_SIZE];
        for (int i = 0; i < FAST_AVERAGE_SIZE; i++) {
            sorted[i] = fastAverageSamples[i];
        }
        qsort(sorted, FAST_AVERAGE_SIZE, sizeof(float), compareFloat);

        float sum = 0.0f;
        for (int i = FAST_AVERAGE_TRIM; i < FAST_AVERAGE_SIZE - FAST_AVERAGE_TRIM; i++) {
            sum += sorted[i];
        }
        fastRaw = sum / (FAST_AVERAGE_SIZE - 2 * FAST_AVERAGE_TRIM);
    }

    uncorrectedFast = INPUT_ALPHA * fastRaw + (1.0f - INPUT_ALPHA) * lastFastInput;
    lastFastInput = uncorrectedFast;
    fastVoltage = uncorrectedFast * voltageCorrectionFactor;
    Input = fastVoltage;

    // Arc detection from the fast voltage.
    arcDetected = (fastVoltage > ARC_THRESHOLD);

    // === SLOW filter rolling average ===
    // The slow filter needs a stable 100 Hz input (10 ms period). Maintain
    // a 10-point rolling average of the raw ADC values; every 10 ms feed the
    // average into the 200-sample slow buffer, preserving the original ~2 s
    // anti-dive reference window.
    slowAverageSum -= slowAverageSamples[slowAverageIdx];
    slowAverageSamples[slowAverageIdx] = raw;
    slowAverageSum += raw;
    slowAverageIdx = (slowAverageIdx + 1) % SLOW_AVERAGE_SIZE;

    if (slowAverageIdx == 0) {
        slowAverageReady = true;
    }

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

    if (gateNow && slowAverageReady &&
        (currentTime - lastSlowSampleTime >= SLOW_SAMPLE_INTERVAL_MS)) {

        float avgRaw = slowAverageSum / SLOW_AVERAGE_SIZE;

        if (!slowInit || !slowGateWasOn) {
            reseedSlowFilter(avgRaw);
        } else {
            slowSum -= slowSamples[slowIdx];
            slowSamples[slowIdx] = avgRaw;
            slowSum += avgRaw;
            slowIdx = (slowIdx + 1) % N_SLOW;
        }

        float slowRawAvg = slowSum / N_SLOW;
        uncorrectedSlow = slowRawAvg;
        slowVoltage = ALPHA_SLOW * (slowRawAvg * voltageCorrectionFactor) +
                      (1.0f - ALPHA_SLOW) * slowLp;
        slowLp = slowVoltage;

        lastSlowSampleTime = currentTime;
    }
    // When the gate is off, slow* values are held at their last on-arc
    // value — better than having them drift toward ambient noise.

    slowGateWasOn = gateNow;
}

void THCController::updateAntiDive(unsigned long currentTime) {
    // Conditions required for anti-dive to be considered
    bool canActivate = (adcWarmedUp &&
                        plasmaPinLow &&
                        plasmaStabilized &&
                        thcActive);
    bool dropDetected = (fastVoltage > slowVoltage + DROP_THRESHOLD);

    // Activation with temporal confirmation: ignore short spikes (< 30 ms)
    // that are likely noise or normal transients. Only commit to a lift if
    // the drop condition persists.
    if (!antiDiveActive) {
        if (canActivate && dropDetected) {
            if (!antiDivePending) {
                antiDivePending = true;
                antiDivePendingStartTime = currentTime;
            } else if (currentTime - antiDivePendingStartTime >= ANTI_DIVE_CONFIRM_MS) {
                antiDiveActive = true;
                justAntiDiveActivated = true;
                antiDiveStartTime = currentTime;
                voltageAtActivation = slowVoltage;
                antiDivePending = false;

                // Emergency lift: command Z to retract by ANTI_DIVE_LIFT_STEPS using
                // an aggressive speed/accel envelope so the move completes in tens of
                // ms — fast enough to clear the previously-cut piece before the torch
                // dives into it on the way back over metal. Positive steps = up
                // (verified against the original monolithic firmware).
                stepper.setMaxSpeed(ANTI_DIVE_LIFT_SPEED);
                stepper.setAcceleration(ANTI_DIVE_LIFT_ACCEL);
                stepper.moveTo(stepper.currentPosition() + ANTI_DIVE_LIFT_STEPS);
            }
        } else {
            // Drop condition disappeared before confirmation -> cancel pending
            antiDivePending = false;
        }
    }

    // Anti-dive deactivation
    if (antiDiveActive) {
        bool shouldDeactivate = (fastVoltage <= voltageAtActivation + RETURN_THRESHOLD) ||
                               (currentTime - antiDiveStartTime >= MAX_ANTI_DIVE_DURATION);

        if (shouldDeactivate) {
            antiDiveActive = false;
            justAntiDiveActivated = false;

            // Restore the PID motion envelope and cancel any unfinished lift
            // travel so the next normalTHCControl tick can resume without
            // fighting a stale position target.
            stepper.setMaxSpeed(STEPPER_MAX_SPEED);
            stepper.setAcceleration(STEPPER_ACCELERATION);
            stepper.moveTo(stepper.currentPosition());
        }
    }
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
    if (plasmaPinLow && !plasmaStabilized) {
        if (plasmaActiveTime == 0) {
            plasmaActiveTime = currentTime;
        }
        if (currentTime - plasmaActiveTime >= STABILIZATION_DELAY) {
            plasmaStabilized = true;
            if (!lastPlasmaStabilized) {
                pid.reset();
                // Re-seed the slow filter with the current stabilized voltage
                // instead of the pierce voltage (~145 V). This prevents the
                // anti-dive reference from taking ~2 s to converge and causing
                // false triggers during the first seconds of cutting.
                if (slowAverageReady) {
                    reseedSlowFilter(uncorrectedFast);
                }
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
    (void)currentTime;
    if (antiDiveActive) {
        // Motion is driven by runMotor() against the moveTo() target set in
        // updateAntiDive() at activation. The PID is paused for the duration
        // of the dive — once the lift completes, the stepper sits at the new
        // position until deactivation restores PID control.
        return;
    }
    if (thcActive) {
        normalTHCControl();
    } else {
        smoothedOutput = 0.0;
        stepper.setSpeed(0);
    }
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
    antiDivePending = false;
}
