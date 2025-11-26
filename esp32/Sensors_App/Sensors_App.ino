#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <limits.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// --- Настройки BLE (ESP32 только шлёт рекламу) ---
static BLEAdvertising* pAdvertising = nullptr;
static const uint16_t MANUF_ID = 0x1234;

unsigned long lastPayloadSent = 0;
const unsigned long PAYLOAD_INTERVAL_MS = 2000;

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

void advertiseSensorPayload(int lightRaw, int waterRaw, float tempInside, float tempOutside, float humOutside) {
    unsigned long now = millis();
    if (now - lastPayloadSent < PAYLOAD_INTERVAL_MS) {
        return;
    }

    if (pAdvertising == nullptr) {
        return;
    }

    // Упакуем данные в ASCII-строку: "L,W,Ti,To,H" (температуры и влажность умножены на 10)
    int ti10 = (int)(tempInside * 10);
    int to10 = (int)(tempOutside * 10);
    int h10 = (int)(humOutside * 10);
    
    String payloadStr = String(lightRaw) + "," +
                        String(waterRaw) + "," +
                        String(ti10) + "," +
                        String(to10) + "," +
                        String(h10);

    // Manufacturer data: [company_id_lo][company_id_hi][ascii...]
    String mData;
    mData.reserve(2 + payloadStr.length());
    mData += char(MANUF_ID & 0xFF);
    mData += char((MANUF_ID >> 8) & 0xFF);
    mData += payloadStr;

    BLEAdvertisementData advData;
    advData.setManufacturerData(mData);
    advData.setName("ESP32_SENS");

    pAdvertising->setAdvertisementData(advData);
    pAdvertising->start();

    Serial.println("[BLE] Sensor payload advertised");
    lastPayloadSent = now;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32 Sensor Hub ===");

  // 1. Инициализация сенсоров
  tempSensor.begin();
  dht.begin();

  // 2. Инициализация BLE (ESP32 только шлёт рекламу с данными сенсоров)
  BLEDevice::init("ESP32_SENS");
  BLEServer* pServer = BLEDevice::createServer();
  (void)pServer;
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setScanResponse(false);
  
  Serial.println("Init done. Advertising sensor data over BLE");
}

void loop() {
  Serial.println("\n--- Reading Cycle ---");

  // --- Чтение сенсоров ---
 
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

  // --- Реклама данных по BLE ---
  advertiseSensorPayload(lightRaw, waterRaw, tempInside, tempOutside, humOutside);
}
