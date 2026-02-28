/**
 * SmartTHC - Gestionnaire de commandes série
 * 
 * Gère les commandes via le port série (RESET_EEPROM, etc.)
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
    
    // Initialisation
    void begin(long baudRate = 115200);
    
    // Traiter les commandes entrantes
    void update(EEPROMManager* eeprom, THCController* thc, SpeedMonitor* speed);
    
    // Logging périodique
    void logStatus(unsigned long currentTime, THCController* thc, SpeedMonitor* speed);
    void logLoopStats(unsigned long currentTime, unsigned long avgTime, float frequency);

private:
    unsigned long lastLogTime;
    unsigned long lastLoopLogTime;
    
    void processCommand(String& command, EEPROMManager* eeprom, THCController* thc, SpeedMonitor* speed);
    void printStatus(THCController* thc, SpeedMonitor* speed);
};

#endif // SERIAL_COMMAND_H
