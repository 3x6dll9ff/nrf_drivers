#include <OneWire.h>
#include <DallasTemperature.h>

// Пин для DS18B20 (можно изменить)
#define ONE_WIRE_PIN 13

// Объекты для работы с датчиком
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature tempSensor(&oneWire);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== DS18B20 Test ===");
  Serial.print("Pin: ");
  Serial.println(ONE_WIRE_PIN);
  
  // Инициализация датчика
  tempSensor.begin();
  
  // Проверка наличия датчика
  int deviceCount = tempSensor.getDeviceCount();
  Serial.print("Found devices: ");
  Serial.println(deviceCount);
  
  if (deviceCount == 0) {
    Serial.println("ERROR: No DS18B20 sensors found!");
    Serial.println("Check wiring:");
    Serial.println("  - VDD -> 3.3V nRF (or ESP32)");
    Serial.println("  - GND -> GND nRF (and connect nRF GND to ESP32 GND!)");
    Serial.println("  - DQ -> GPIO pin ESP32 (with 4.7k pullup to VDD)");
    Serial.println("  IMPORTANT: nRF GND must be connected to ESP32 GND!");
  }
  
  Serial.println("\nReading temperature every 2 seconds...\n");
}

void loop() {
  // Запрашиваем температуру
  tempSensor.requestTemperatures();
  
  // Читаем температуру
  float tempC = tempSensor.getTempCByIndex(0);
  
  // Выводим результат
  Serial.print("Temperature: ");
  
  if (tempC == -127.0 || tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("ERROR - Sensor not responding!");
  } else {
    Serial.print(tempC);
    Serial.println(" C");
  }
  
  delay(2000); // Ждем 2 секунды
}

