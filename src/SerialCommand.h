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

    void processCommand(String& command, EEPROMManager* eeprom, THCController* thc, SpeedMonitor* speed);
    void printStatus(THCController* thc, SpeedMonitor* speed);
};

#endif // SERIAL_COMMAND_H
