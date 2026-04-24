/**
 * SmartTHC - Serial Command Handler
 *
 * Serial commands and logging implementation
 *
 * Log format (DEBUG ON):
 *   - One compact "st=" line every LOG_INTERVAL ms with full state + numbers,
 *     KV-formatted so it can be grep/awk'd and plotted directly. Example:
 *       t=125340 st=THC_ACTIVE v=122.1/121.7 tgt=120.0 out=+15 spd=600.0 z=+1234 ad=off
 *   - One "EV:" line on every state transition (plasma, THC, anti-dive,
 *     gates). These are edge-triggered, so the event stream is sparse and
 *     directly corresponds to what the physical system just did.
 */

#include "SerialCommand.h"
#include "EEPROMManager.h"
#include "THCController.h"
#include "SpeedMonitor.h"
#include <ArduPID.h>

SerialCommand::SerialCommand()
    : debugEnabled(false)
    , lastLogTime(0)
    , lastLoopLogTime(0)
    , lastPlasmaOn(false)
    , lastArcDetected(false)
    , lastStabilized(false)
    , lastThcActive(false)
    , lastAntiDiveActive(false)
    , lastThcSig(false)
    , lastMotionGate(false)
    , lastReStab(false)
    , antiDiveStartMs(0)
    , antiDiveStartFast(0.0f)
    , antiDiveStartSlow(0.0f)
    , firstStatusLine(true)
{
}

void SerialCommand::begin(long baudRate) {
    Serial.begin(baudRate);
}

void SerialCommand::update(EEPROMManager* eeprom, THCController* thc, SpeedMonitor* speed) {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        processCommand(command, eeprom, thc, speed);
    }
}

void SerialCommand::processCommand(String& command, EEPROMManager* eeprom,
                                   THCController* thc, SpeedMonitor* speed) {
    if (command == "RESET_EEPROM") {
        eeprom->resetToDefaults();

        // Reload parameters
        THCParameters params;
        eeprom->loadParameters(params);

        thc->setSetpoint(params.setpoint);
        thc->setCorrectionFactor(params.correctionFactor);
        speed->setCutSpeed(params.cutSpeed);
        speed->setThresholdRatio(params.thresholdRatio);
        thc->setKp(params.kp);
        thc->setKi(params.ki);
        thc->setKd(params.kd);

        Serial.println("EEPROM reset via serial command");
    }
    else if (command == "STATUS") {
        printStatus(thc, speed);
    }
    else if (command == "DEBUG") {
        debugEnabled = !debugEnabled;
        Serial.print("Debug logging: ");
        Serial.println(debugEnabled ? "ON" : "OFF");
        if (debugEnabled) {
            // Force a header + initial snapshot on next status tick.
            firstStatusLine = true;
            lastLogTime = 0;
        }
    }
    else if (command == "HELP") {
        Serial.println("Commands: RESET_EEPROM, STATUS, DEBUG, HELP");
        Serial.println("Log format: 'st=' = periodic snapshot, 'EV:' = state transition");
    }
}

const char* SerialCommand::currentStateLabel(THCController* thc, SpeedMonitor* speed) {
    // Mirror the gating chain from THCController::updateTHCState so the state
    // label answers the question "why isn't THC running right now?".
    if (!thc->isPlasmaActive())       return "PLASMA_OFF";
    if (!thc->isPlasmaStabilized())   return "WAIT_STAB";
    if (!thc->isTHCOff())             return "THC_SIG_OFF";
    if (!thc->isEnableLow())          return "ENABLE_OFF";
    if (!thc->isArcDetected())        return "ARC_LOST";
    if (!thc->isThcOnReStabilized())  return "WAIT_RESTAB";
    if (!thc->isCutMotionGateReady()) return "WAIT_MOTION";
    if (!speed->isSpeedOK())          return "WAIT_SPEED";
    if (thc->isAntiDiveActive())      return "ANTI_DIVE";
    if (thc->isTHCActive())           return "THC_ACTIVE";
    return "ARMED";
}

const char* SerialCommand::thcInactiveReason(THCController* thc, SpeedMonitor* speed) {
    // Same chain as currentStateLabel but worded as a reason string for
    // transition events ("THC inactive because X").
    if (!thc->isTHCOff())             return "THC_SIG_LOW";
    if (!thc->isEnableLow())          return "ENABLE_HIGH";
    if (!thc->isPlasmaActive())       return "PLASMA_OFF";
    if (!thc->isPlasmaStabilized())   return "STAB_WAIT";
    if (!thc->isArcDetected())        return "ARC_LOST";
    if (!speed->isSpeedOK())          return "SPEED_LOW";
    if (thc->isAntiDiveActive())      return "ANTI_DIVE";
    if (!thc->isThcOnReStabilized())  return "RESTAB_WAIT";
    if (!thc->isCutMotionGateReady()) return "MOTION_GATE_WAIT";
    return "UNKNOWN";
}

void SerialCommand::detectEvents(unsigned long currentTime, THCController* thc, SpeedMonitor* speed) {
    const bool plasmaOn  = thc->isPlasmaActive();
    const bool arc       = thc->isArcDetected();
    const bool stab      = thc->isPlasmaStabilized();
    const bool thcSig    = thc->isTHCOff();
    const bool thcActive = thc->isTHCActive();
    const bool adActive  = thc->isAntiDiveActive();
    const bool motion    = thc->isCutMotionGateReady();
    const bool reStab    = thc->isThcOnReStabilized();

    if (plasmaOn != lastPlasmaOn) {
        Serial.print("EV: t="); Serial.print(currentTime);
        Serial.println(plasmaOn ? " plasma ON" : " plasma OFF");
    }
    if (arc != lastArcDetected) {
        Serial.print("EV: t="); Serial.print(currentTime);
        Serial.print(arc ? " arc detected (v=" : " arc lost (v=");
        Serial.print(thc->getFastVoltage(), 1);
        Serial.println("V)");
    }
    if (stab != lastStabilized) {
        if (stab) {
            Serial.print("EV: t="); Serial.print(currentTime);
            Serial.print(" plasma stabilized (v=");
            Serial.print(thc->getFastVoltage(), 1);
            Serial.println("V)");
        }
    }
    if (thcSig != lastThcSig) {
        Serial.print("EV: t="); Serial.print(currentTime);
        Serial.println(thcSig ? " THC_SIG asserted" : " THC_SIG released");
    }
    if (motion != lastMotionGate) {
        Serial.print("EV: t="); Serial.print(currentTime);
        Serial.println(motion ? " motion gate READY" : " motion gate lost");
    }
    if (reStab != lastReStab) {
        Serial.print("EV: t="); Serial.print(currentTime);
        Serial.println(reStab ? " THC re-stab delay DONE" : " THC re-stab delay ARMED");
    }
    if (thcActive != lastThcActive) {
        Serial.print("EV: t="); Serial.print(currentTime);
        if (thcActive) {
            Serial.print(" THC active (v=");
            Serial.print(thc->getFastVoltage(), 1);
            Serial.print(" tgt=");
            Serial.print(thc->getSetpoint(), 1);
            Serial.println(")");
        } else {
            Serial.print(" THC inactive reason=");
            Serial.println(thcInactiveReason(thc, speed));
        }
    }
    if (adActive != lastAntiDiveActive) {
        Serial.print("EV: t="); Serial.print(currentTime);
        if (adActive) {
            antiDiveStartMs   = currentTime;
            antiDiveStartFast = thc->getFastVoltage();
            antiDiveStartSlow = thc->getSlowVoltage();
            Serial.print(" anti-dive TRIGGERED fast=");
            Serial.print(antiDiveStartFast, 1);
            Serial.print(" slow=");
            Serial.print(antiDiveStartSlow, 1);
            Serial.print(" drop=+");
            Serial.print(antiDiveStartFast - antiDiveStartSlow, 1);
            Serial.println("V");
        } else {
            unsigned long dur = currentTime - antiDiveStartMs;
            Serial.print(" anti-dive RELEASED duration=");
            Serial.print(dur);
            Serial.print("ms fast=");
            Serial.print(thc->getFastVoltage(), 1);
            Serial.println("V");
        }
    }

    lastPlasmaOn       = plasmaOn;
    lastArcDetected    = arc;
    lastStabilized     = stab;
    lastThcSig         = thcSig;
    lastThcActive      = thcActive;
    lastAntiDiveActive = adActive;
    lastMotionGate     = motion;
    lastReStab         = reStab;
}

void SerialCommand::printStatusLine(unsigned long currentTime, THCController* thc, SpeedMonitor* speed) {
    if (firstStatusLine) {
        Serial.println("# legend: t=ms st=state v=fast/slow tgt=setpoint out=pid_steps_s spd=mm_or_in_per_min z=Z_steps ad=anti_dive");
        firstStatusLine = false;
    }

    Serial.print("t="); Serial.print(currentTime);
    Serial.print(" st="); Serial.print(currentStateLabel(thc, speed));
    Serial.print(" v="); Serial.print(thc->getFastVoltage(), 1);
    Serial.print('/');   Serial.print(thc->getSlowVoltage(), 1);
    Serial.print(" tgt="); Serial.print(thc->getSetpoint(), 1);
    Serial.print(" out=");
    const double out = thc->getPidOutput();
    if (out >= 0) Serial.print('+');
    Serial.print(out, 0);
    Serial.print(" spd="); Serial.print(speed->getFilteredSpeed(), 0);
    Serial.print(" z=");
    long zpos = thc->getMotorPosition();
    if (zpos >= 0) Serial.print('+');
    Serial.print(zpos);
    Serial.print(" ad=");
    Serial.println(thc->isAntiDiveActive() ? "ACTIVE" : "off");
}

void SerialCommand::logStatus(unsigned long currentTime, THCController* thc, SpeedMonitor* speed) {
    // Events always fire (edge-triggered) when debug is on so we never miss a
    // transition even between periodic ticks.
    if (!debugEnabled) {
        // When debug is off, still refresh the snapshot without printing so
        // re-enabling debug doesn't emit stale transitions as if they just
        // happened.
        lastPlasmaOn       = thc->isPlasmaActive();
        lastArcDetected    = thc->isArcDetected();
        lastStabilized     = thc->isPlasmaStabilized();
        lastThcSig         = thc->isTHCOff();
        lastThcActive      = thc->isTHCActive();
        lastAntiDiveActive = thc->isAntiDiveActive();
        lastMotionGate     = thc->isCutMotionGateReady();
        lastReStab         = thc->isThcOnReStabilized();
        return;
    }

    detectEvents(currentTime, thc, speed);

    if (currentTime - lastLogTime < LOG_INTERVAL) {
        return;
    }
    lastLogTime = currentTime;

    printStatusLine(currentTime, thc, speed);
}

void SerialCommand::logLoopStats(unsigned long currentTime, unsigned long avgTime, float frequency) {
    if (debugEnabled && currentTime - lastLoopLogTime >= LOOP_LOG_INTERVAL) {
        Serial.print("# loop avg="); Serial.print(avgTime);
        Serial.print("us freq="); Serial.print(frequency, 0);
        Serial.println("Hz");
        lastLoopLogTime = currentTime;
    }
}

void SerialCommand::printStatus(THCController* thc, SpeedMonitor* speed) {
    Serial.println("=== SmartTHC Status ===");
    Serial.print("state="); Serial.println(currentStateLabel(thc, speed));
    Serial.print("setpoint="); Serial.println(thc->getSetpoint(), 1);
    Serial.print("fast_v="); Serial.println(thc->getFastVoltage(), 1);
    Serial.print("slow_v="); Serial.println(thc->getSlowVoltage(), 1);
    Serial.print("pid_out="); Serial.println(thc->getPidOutput(), 0);
    Serial.print("speed="); Serial.println(speed->getFilteredSpeed(), 0);
    Serial.print("z_pos="); Serial.println(thc->getMotorPosition());
    Serial.print("thc_active="); Serial.println(thc->isTHCActive() ? "yes" : "no");
    Serial.print("anti_dive="); Serial.println(thc->isAntiDiveActive() ? "yes" : "no");
    if (!thc->isTHCActive()) {
        Serial.print("thc_inactive_reason="); Serial.println(thcInactiveReason(thc, speed));
    }
}
