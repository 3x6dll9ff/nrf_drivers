#!/bin/bash

# ESP32 Sensors App Build & Upload Script
# Run this script from the 'esp32' directory!

SKETCH="Sensors_App/Sensors_App.ino"
# Using min_spiffs partition scheme
BOARD_FQBN="esp32:esp32:esp32:PartitionScheme=min_spiffs"
PORT="/dev/cu.usbserial-1440"
BUILD_DIR="build"

echo "=== ESP32 Sensors App Build & Upload ==="
echo "Sketch: $SKETCH"
echo "Port:   $PORT"
echo ""

# 1. Install Dependencies
echo "[1/4] Checking libraries..."
arduino-cli lib install "OneWire"
arduino-cli lib install "DallasTemperature"
arduino-cli lib install "DHT sensor library"
arduino-cli lib install "Adafruit Unified Sensor"

# 2. Compile
echo "[2/4] Compiling sketch..."
mkdir -p "$BUILD_DIR"
arduino-cli compile --fqbn "$BOARD_FQBN" "$SKETCH" \
  --build-path "$BUILD_DIR" \
  --build-property "upload.maximum_size=1966080" || {
    echo "Compilation failed!"
    exit 1
}

# 3. Upload
echo ""
echo "[3/4] Uploading firmware..."
echo "Note: If upload fails, hold BOOT button on ESP32"

# Locate compiled binaries in the build directory (excluding merged.bin!)
PART_TABLE=$(find "$BUILD_DIR" -name "*.partitions.bin" | head -1)
BOOTLOADER=$(find "$BUILD_DIR" -name "*.bootloader.bin" | head -1)
# Explicitly exclude .merged.bin to avoid the 4MB size error
APP_BIN=$(find "$BUILD_DIR" -name "*.bin" ! -name "*.partitions.bin" ! -name "*.bootloader.bin" ! -name "*.merged.bin" | head -1)
BOOT_APP0=$(find ~/Library/Arduino15/packages/esp32/hardware/esp32/3.3.0/tools/partitions -name "boot_app0.bin" | head -1)

if [[ -z "$PART_TABLE" || -z "$BOOTLOADER" || -z "$APP_BIN" ]]; then
    echo "Error: Could not find compiled binaries in $BUILD_DIR"
    exit 1
fi

ESPTOOL=$(find ~/Library/Arduino15/packages/esp32/tools/esptool_py -name "esptool" | head -1)

"$ESPTOOL" --chip esp32 --port "$PORT" --baud 460800 --before default-reset --after hard-reset write-flash \
  -z --flash_mode dio --flash_freq 80m --flash_size 4MB \
  0x1000 "$BOOTLOADER" \
  0x8000 "$PART_TABLE" \
  0xe000 "$BOOT_APP0" \
  0x10000 "$APP_BIN" || {
    echo "Upload failed! Trying lower baud rate..."
    "$ESPTOOL" --chip esp32 --port "$PORT" --baud 115200 --before default-reset --after hard-reset write-flash \
      -z --flash_mode dio --flash_freq 80m --flash_size 4MB \
      0x1000 "$BOOTLOADER" \
      0x8000 "$PART_TABLE" \
      0xe000 "$BOOT_APP0" \
      0x10000 "$APP_BIN" || {
        echo "Upload failed!"
        exit 1
    }
}

echo ""
echo "[4/4] Success! Starting monitor..."
echo "Press Ctrl+C to stop monitor"
echo ""

# 4. Monitor
python3 -c "
import serial
import sys
import time

try:
    ser = serial.Serial('$PORT', 115200, timeout=0.1)
    print(f'Connected to {ser.name}')
    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line: print(line)
            except:
                pass
        time.sleep(0.01)
except KeyboardInterrupt:
    print('\nExiting...')
except Exception as e:
    print(f'Monitor error: {e}')
"

