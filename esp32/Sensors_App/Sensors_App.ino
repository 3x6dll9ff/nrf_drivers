#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <limits.h>
#include <map>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// --- Настройки BLE ---
String targetMAC = "dd:b7:ce:3f:51:0f"; // nRF52 MAC address
BLEScan* pBLEScan;
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pSensorCharacteristic = nullptr;

static BLEUUID sensorServiceUUID("0fb0a001-7654-3210-01ef-cdab89674523");
static BLEUUID sensorCharUUID("0fb0a002-7654-3210-01ef-cdab89674523");

bool bleConnected = false;
unsigned long lastPayloadSent = 0;
const unsigned long PAYLOAD_INTERVAL_MS = 2000;
const uint16_t BLE_CONN_RETRY_DELAY_MS = 2000;

// --- Пины сенсоров ---
#define LIGHT_SENSOR_PIN 32      // LDR
#define WATER_SENSOR_PIN 33      // water sensor
#define ONE_WIRE_PIN    13       // DS18B20 (внутренняя температура)
#define DHT_PIN          23      // DHT22 (внешняя температура + влажность)
#define DHT_TYPE     DHT22

// --- Объекты сенсоров ---
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature tempSensor(&oneWire);   // DS18B20
DHT dht(DHT_PIN, DHT_TYPE);               // DHT22

class SensorClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) override {
        Serial.println("[BLE] Connected to Game_Score");
    }

    void onDisconnect(BLEClient* pclient) override {
        Serial.println("[BLE] Disconnected from Game_Score");
        bleConnected = false;
        pSensorCharacteristic = nullptr;
    }
};

SensorClientCallbacks clientCallbacks;

struct __attribute__((packed)) SensorPayload {
    uint16_t lightRaw;
    uint16_t waterRaw;
    int16_t  tempInsideCx100;
    int16_t  tempOutsideCx100;
    uint16_t humOutsidePctCx100;
};

int16_t encodeTemperature(float value) {
    if (isnan(value)) return INT16_MIN;
    return (int16_t)(value * 100.0f);
}

uint16_t encodeHumidity(float value) {
    if (isnan(value)) return UINT16_MAX;
    float scaled = value * 100.0f;
    if (scaled < 0) scaled = 0;
    if (scaled > 65535) scaled = 65535;
    return (uint16_t)scaled;
}

bool ensureBleConnection(BLEAdvertisedDevice& advertisedDevice) {
    if (bleConnected && pClient != nullptr && pClient->isConnected() && pSensorCharacteristic != nullptr) {
        return true;
    }

    if (pClient == nullptr) {
        pClient = BLEDevice::createClient();
        pClient->setClientCallbacks(&clientCallbacks);
    }

    Serial.println("[BLE] Attempting connection...");
    BLEAddress address = advertisedDevice.getAddress();
    uint8_t addrType = advertisedDevice.getAddressType();
    if (!pClient->connect(address, addrType)) {
        Serial.println("[BLE] Connection failed");
        return false;
    }

    delay(200);
    if (!pClient->isConnected()) {
        Serial.println("[BLE] Lost connection immediately after connect");
        return false;
    }

    Serial.print("[BLE] Conn ID: ");
    Serial.println(pClient->getConnId());
    Serial.print("[BLE] Negotiated MTU: ");
    Serial.println(pClient->getMTU());

    std::map<std::string, BLERemoteService *>* services = pClient->getServices();
    if (services == nullptr || services->empty()) {
        Serial.println("[BLE] No GATT services discovered");
        delay(200);
        services = pClient->getServices();
    }

    if (services == nullptr || services->empty()) {
        Serial.println("[BLE] Service discovery failed again, scheduling rescan");
        if (pClient->isConnected()) {
            pClient->disconnect();
        }
        delay(100);
        return false;
    } else {
        Serial.println("[BLE] Discovered services:");
        for (auto const& kv : *services) {
            Serial.print("  - ");
            Serial.println(kv.first.c_str());
        }
    }

    BLERemoteService* service = pClient->getService(sensorServiceUUID);
    if (service == nullptr) {
        Serial.println("[BLE] Sensor service not found, disconnecting");
        pClient->disconnect();
        return false;
    }

    pSensorCharacteristic = service->getCharacteristic(sensorCharUUID);
    if (pSensorCharacteristic == nullptr) {
        Serial.println("[BLE] Sensor characteristic not found, disconnecting");
        pClient->disconnect();
        return false;
    }

    bleConnected = true;
    Serial.println("[BLE] Ready to send payloads");
    return true;
}

void sendSensorPayload(int lightRaw, int waterRaw, float tempInside, float tempOutside, float humOutside) {
    if (!bleConnected || pSensorCharacteristic == nullptr) {
        return;
    }

    unsigned long now = millis();
    if (now - lastPayloadSent < PAYLOAD_INTERVAL_MS) {
        return;
    }

    SensorPayload payload;
    payload.lightRaw = static_cast<uint16_t>(lightRaw);
    payload.waterRaw = static_cast<uint16_t>(waterRaw);
    payload.tempInsideCx100 = encodeTemperature(tempInside);
    payload.tempOutsideCx100 = encodeTemperature(tempOutside);
    payload.humOutsidePctCx100 = encodeHumidity(humOutside);

    bool ok = pSensorCharacteristic->writeValue((uint8_t*)&payload, sizeof(payload), false);
    if (ok) {
        Serial.println("[BLE] Sensor payload sent");
        lastPayloadSent = now;
    } else {
        Serial.println("[BLE] Failed to send payload, disconnecting");
        bleConnected = false;
        if (pClient != nullptr && pClient->isConnected()) {
            pClient->disconnect();
        }
        pSensorCharacteristic = nullptr;
    }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32 Sensor Hub + BLE Scanner ===");

  // 1. Инициализация сенсоров
  tempSensor.begin();
  dht.begin();

  // 2. Инициализация BLE
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true); // Активное сканирование (быстрее находит)
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  
  Serial.println("Init done. Looking for device: " + targetMAC);
}

void loop() {
  Serial.println("\n--- Reading Cycle ---");

  // --- 1. BLE Scanning (только если ещё не подключены) ---
  if (!bleConnected) {
    Serial.print("Scanning BLE... ");
    BLEScanResults* foundDevices = pBLEScan->start(2, false);
    bool deviceFound = false;
    int rssi = 0;
    BLEAdvertisedDevice targetDevice;

    if (foundDevices != nullptr) {
      for (int i = 0; i < foundDevices->getCount(); i++) {
        BLEAdvertisedDevice device = foundDevices->getDevice(i);
        String addr = device.getAddress().toString();
        addr.toLowerCase(); // Приводим к нижнему регистру для сравнения

        if (addr == targetMAC) {
          deviceFound = true;
          rssi = device.getRSSI();
          targetDevice = device;
          break;
        }
      }
      pBLEScan->clearResults(); // Очищаем память
    } else {
      Serial.println(" [SCAN ERROR]");
    }
    
    if (deviceFound) {
      Serial.print(" [FOUND] Game_Score (RSSI: ");
      Serial.print(rssi);
      Serial.println(" dBm)");
      ensureBleConnection(targetDevice);
    } else {
      Serial.println(" [NOT FOUND]");
    }
  } else {
    Serial.println("Scanning BLE... skipped (already connected)");
  }

  // --- 2. Sensors Reading ---
  
  // Свет
  int lightRaw = analogRead(LIGHT_SENSOR_PIN);
  
  // Вода
  int waterRaw = analogRead(WATER_SENSOR_PIN);
  
  // Внутренняя температура (DS18B20)
  tempSensor.requestTemperatures();
  float tempInside = tempSensor.getTempCByIndex(0);

  // Внешняя температура и влажность (DHT22)
  float tempOutside = dht.readTemperature();
  float humOutside  = dht.readHumidity();

  // --- 3. Output ---
  Serial.print("Light Raw:      "); Serial.println(lightRaw);
  Serial.print("Water Raw:      "); Serial.println(waterRaw);
  
  Serial.print("Temp (Inside):  "); 
  if (tempInside == -127.0) Serial.println("Error");
  else { Serial.print(tempInside); Serial.println(" C"); }

  Serial.print("Temp (Outside): ");
  if (isnan(tempOutside)) Serial.println("Error");
  else { Serial.print(tempOutside); Serial.println(" C"); }

  Serial.print("Hum (Outside):  ");
  if (isnan(humOutside)) Serial.println("Error");
  else { Serial.print(humOutside); Serial.println(" %"); }

  sendSensorPayload(lightRaw, waterRaw, tempInside, tempOutside, humOutside);
}
