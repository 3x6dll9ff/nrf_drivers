#include <Arduino.h>
#line 1 "/Users/danilakardashevkii/Developer/embedded/aiot_play_fw/esp32/Sensors_App/Sensors_App.ino"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// --- Настройки BLE ---
String targetMAC = "dd:b7:ce:3f:51:0f"; // nRF52 MAC address
BLEScan* pBLEScan;

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

#line 25 "/Users/danilakardashevkii/Developer/embedded/aiot_play_fw/esp32/Sensors_App/Sensors_App.ino"
void setup();
#line 44 "/Users/danilakardashevkii/Developer/embedded/aiot_play_fw/esp32/Sensors_App/Sensors_App.ino"
void loop();
#line 25 "/Users/danilakardashevkii/Developer/embedded/aiot_play_fw/esp32/Sensors_App/Sensors_App.ino"
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

  // --- 1. BLE Scanning ---
  // Сканируем 2 секунды (это заменяет delay в конце)
  Serial.print("Scanning BLE... ");
  BLEScanResults foundDevices = pBLEScan->start(2, false);
  
  bool deviceFound = false;
  int rssi = 0;

  for (int i = 0; i < foundDevices.getCount(); i++) {
    BLEAdvertisedDevice device = foundDevices.getDevice(i);
    String addr = device.getAddress().toString();
    addr.toLowerCase(); // Приводим к нижнему регистру для сравнения

    if (addr == targetMAC) {
      deviceFound = true;
      rssi = device.getRSSI();
      break;
    }
  }
  pBLEScan->clearResults(); // Очищаем память
  
  if (deviceFound) {
    Serial.print(" [FOUND] Game_Score (RSSI: ");
    Serial.print(rssi);
    Serial.println(" dBm)");
  } else {
    Serial.println(" [NOT FOUND]");
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
}

