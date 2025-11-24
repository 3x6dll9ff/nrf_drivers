#!/bin/bash

# Set up variables
SKETCH="BLE_Scanner/BLE_Scanner.ino"
BOARD="esp32:esp32:esp32"
PORT="/dev/cu.usbserial-1440"

# Compile
echo "Compiling..."
arduino-cli compile --fqbn $BOARD $SKETCH
if [ $? -ne 0 ]; then
    echo "Compilation failed!"
    exit 1
fi

# Upload
echo "Uploading..."
echo "IMPORTANT: Hold BOOT button on ESP32 when you see 'Connecting...'"
echo "Press Enter when ready..."
read

# Try with lower baud rate
arduino-cli upload -p $PORT --fqbn $BOARD $SKETCH --input-dir /tmp/esp32_build 2>&1 | head -30
# If that fails, try direct esptool with lower speed
if [ $? -ne 0 ]; then
    echo "Trying with lower baud rate (115200)..."
    # Get the compiled binary path
    BUILD_DIR="/Users/danilakardashevkii/Library/Caches/arduino/sketches"
    SKETCH_HASH=$(arduino-cli sketch new /tmp/test_sketch 2>&1 | grep -o '[0-9A-F]\{32\}' | head -1 || echo "0EF719C308E4DFE00DD5F8B5914B8182")
    BIN_FILE="$BUILD_DIR/$SKETCH_HASH/BLE_Scanner.ino.bin"
    
    if [ -f "$BIN_FILE" ]; then
        esptool.py --chip esp32 --port $PORT --baud 115200 write_flash 0x10000 "$BIN_FILE"
    fi
fi
if [ $? -ne 0 ]; then
    echo "Upload failed!"
    echo "Try:"
    echo "1. Hold BOOT button on ESP32"
    echo "2. Press and release RESET button while holding BOOT"
    echo "3. Release BOOT button"
    echo "4. Run upload again"
    exit 1
fi

echo "Done! Monitoring serial port..."
# Monitor (exit with Ctrl+C)
arduino-cli monitor -p $PORT --config baudrate=115200

