/*
   BLE Scanner for ESP32 - Minimal Version
   Scans for BLE devices and prints them to Serial Monitor
*/

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int scanTime = 5; // In seconds
BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("Device: ");
      Serial.print(advertisedDevice.getName().c_str());
      Serial.print(" | Address: ");
      Serial.print(advertisedDevice.getAddress().toString().c_str());
      Serial.print(" | RSSI: ");
      Serial.println(advertisedDevice.getRSSI());
      
      if (advertisedDevice.getName() == "Game_Score") {
        Serial.println("*** FOUND GAME_SCORE! ***");
      }
    }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("BLE Scanner starting...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  
  Serial.println("Scanner ready!");
}

void loop() {
  Serial.println("Scanning...");
  BLEScanResults* foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("Found ");
  Serial.print(foundDevices->getCount());
  Serial.println(" devices");
  pBLEScan->clearResults();
  delay(2000);
}
