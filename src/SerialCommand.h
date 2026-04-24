/**
 * SmartTHC - Serial Command Handler
 *
 * Manages serial port commands (RESET_EEPROM, etc.)
 */

#ifndef SERIAL_COMMAND_H
#define SERIAL_COMMAND_H

#include <Arduino.h>

// Forward declarations
class EEPROMManager;
class THCController;
class SpeedMonitor;

class SerialCommand {
public:
    SerialCommand();

    // Initialization
    void begin(long baudRate = 115200);

    // Process incoming commands
    void update(EEPROMManager* eeprom, THCController* thc, SpeedMonitor* speed);

    // Periodic logging
    void logStatus(unsigned long currentTime, THCController* thc, SpeedMonitor* speed);
    void logLoopStats(unsigned long currentTime, unsigned long avgTime, float frequency);

private:
    bool debugEnabled;
    unsigned long lastLogTime;
    unsigned long lastLoopLogTime;

    // Edge-trigger state snapshot — used by detectEvents() to emit
    // one-shot event lines on transitions. These fire on every DEBUG
    // cycle regardless of LOG_INTERVAL, so the event stream stays
    // aligned with physical reality.
    bool lastPlasmaOn;
    bool lastArcDetected;
    bool lastStabilized;
    bool lastThcActive;
    bool lastAntiDiveActive;
    bool lastThcSig;
    bool lastMotionGate;
    bool lastReStab;
    unsigned long antiDiveStartMs;
    float antiDiveStartFast;
    float antiDiveStartSlow;
    bool firstStatusLine;

    void processCommand(String& command, EEPROMManager* eeprom, THCController* thc, SpeedMonitor* speed);
    void printStatus(THCController* thc, SpeedMonitor* speed);
    void detectEvents(unsigned long currentTime, THCController* thc, SpeedMonitor* speed);
    void printStatusLine(unsigned long currentTime, THCController* thc, SpeedMonitor* speed);
    const char* currentStateLabel(THCController* thc, SpeedMonitor* speed);
    const char* thcInactiveReason(THCController* thc, SpeedMonitor* speed);
};

#endif // SERIAL_COMMAND_H
