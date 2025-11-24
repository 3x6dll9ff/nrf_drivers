#!/bin/bash

PORT="/dev/cu.usbserial-1440"

echo "=== BLE Scanner Monitor ==="
echo "Reading from $PORT..."
echo "Press Ctrl+C to stop"
echo ""

python3 -c "
import serial
import sys
import time

try:
    ser = serial.Serial('$PORT', 115200, timeout=1)
    time.sleep(2)  # Wait for ESP32 to boot
    print('Connected! Reading data...\n')
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)
        time.sleep(0.1)
except KeyboardInterrupt:
    print('\n\nStopped')
    ser.close()
except serial.serialutil.SerialException as e:
    print(f'Port error: {e}')
    print('Make sure:')
    print('1. ESP32 is connected')
    print('2. No other program is using the port')
    print('3. Close Arduino IDE if it\'s open')
    sys.exit(1)
except Exception as e:
    print(f'Error: {e}')
    sys.exit(1)
"
