#!/usr/bin/env python3
import serial
import time
import sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

print(f"→ Ouverture {PORT} à 1200 baud...")
ser = serial.Serial()
ser.port = PORT
ser.baudrate = 1200
ser.dtr = True      # assert DTR
ser.rts = True
ser.open()

print("→ Attente 0.5s pour laisser le sketch détecter...")
time.sleep(0.5)

print("→ Fermeture du port (trigger bootloader)...")
ser.dtr = False
ser.close()

print("Done. L'Arduino devrait être en DFU dans 2-3s.")