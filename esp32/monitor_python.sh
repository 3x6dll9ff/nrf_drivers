#!/bin/bash

PORT="/dev/cu.usbserial-1440"
BAUDRATE=115200

echo "Starting BLE Scanner monitor on $PORT..."
echo "Press Ctrl+C to stop"
echo ""

# Alternative: use Python for serial monitoring
python3 -c "
import serial
import sys

try:
    ser = serial.Serial('$PORT', $BAUDRATE, timeout=1)
    print('Connected! Reading data...\n')
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)
except KeyboardInterrupt:
    print('\nStopped')
    ser.close()
except Exception as e:
    print(f'Error: {e}')
    sys.exit(1)
"

