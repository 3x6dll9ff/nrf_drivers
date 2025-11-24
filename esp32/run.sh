#!/bin/bash

PORT="/dev/cu.usbserial-1440"
BOARD="esp32:esp32:esp32"
SKETCH="BLE_Scanner/BLE_Scanner.ino"

echo "=== BLE Scanner Build & Upload ==="
echo ""

# Compile
echo "[1/3] Compiling..."
arduino-cli compile --fqbn $BOARD $SKETCH
if [ $? -ne 0 ]; then
    echo "Compilation failed!"
    exit 1
fi

# Upload with low baud rate
echo ""
echo "[2/3] Uploading (this may take a minute)..."
echo "Note: If upload fails, try holding BOOT button on ESP32"

# Find the build directory
BUILD_DIR=$(arduino-cli compile --fqbn $BOARD $SKETCH 2>&1 | grep -o '/Users/.*/sketches/[0-9A-F]\{32\}' | head -1)
if [ -z "$BUILD_DIR" ]; then
    # Try to find it manually
    BUILD_DIR=$(find ~/Library/Caches/arduino/sketches -name "BLE_Scanner.ino.bin" -type f 2>/dev/null | head -1 | xargs dirname)
fi

if [ -z "$BUILD_DIR" ] || [ ! -d "$BUILD_DIR" ]; then
    echo "Error: Could not find build directory"
    exit 1
fi

BIN_FILE="$BUILD_DIR/BLE_Scanner.ino.bin"
BOOTLOADER="$BUILD_DIR/BLE_Scanner.ino.bootloader.bin"
PARTITIONS="$BUILD_DIR/BLE_Scanner.ino.partitions.bin"
ESPTOOL="/Users/danilakardashevkii/Library/Arduino15/packages/esp32/tools/esptool_py/5.0.0/esptool"
BOOT_APP0="/Users/danilakardashevkii/Library/Arduino15/packages/esp32/hardware/esp32/3.3.0/tools/partitions/boot_app0.bin"

if [ ! -f "$BIN_FILE" ] || [ ! -f "$BOOTLOADER" ] || [ ! -f "$PARTITIONS" ]; then
    echo "Error: Build files not found in $BUILD_DIR"
    exit 1
fi

"$ESPTOOL" --chip esp32 --port $PORT --baud 115200 --before default-reset --after hard-reset write-flash --flash-mode keep --flash-freq keep --flash-size keep 0x1000 "$BOOTLOADER" 0x8000 "$PARTITIONS" 0xe000 "$BOOT_APP0" 0x10000 "$BIN_FILE"

if [ $? -ne 0 ]; then
    echo ""
    echo "Upload failed! Try:"
    echo "1. Hold BOOT button on ESP32"
    echo "2. Press and release RESET while holding BOOT"
    echo "3. Release BOOT"
    echo "4. Run this script again"
    exit 1
fi

echo "Upload complete!"
echo ""
echo "[3/3] Starting monitor..."
echo "Press Ctrl+C to stop"
echo ""

# Monitor
python3 -c "
import serial
import sys
import time

try:
    ser = serial.Serial('$PORT', 115200, timeout=1)
    time.sleep(2)  # Wait for ESP32 to boot
    print('=== BLE Scanner Output ===\n')
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)
        time.sleep(0.1)
except KeyboardInterrupt:
    print('\n\nStopped')
    ser.close()
except Exception as e:
    print(f'Error: {e}')
    sys.exit(1)
"

