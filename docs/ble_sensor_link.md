## BLE Sensor Link

- **Service UUID**: `0fb0a001-7654-3210-01ef-cdab89674523`
- **Characteristic UUID**: `0fb0a002-7654-3210-01ef-cdab89674523` (`Write` / `Write Without Response`)
- **Payload (Little-endian, 10 bytes)**  
  | Offset | Type    | Field                    | Unit            |
  | ------ | ------- | ------------------------ | --------------- |
  | 0      | uint16  | `light_raw`              | ADC units       |
  | 2      | uint16  | `water_raw`              | ADC units       |
  | 4      | int16   | `temp_inside_cx100`      | °C × 100        |
  | 6      | int16   | `temp_outside_cx100`     | °C × 100        |
  | 8      | uint16  | `hum_outside_pct_x100`   | %RH × 100       |

Special values: `temp_* = INT16_MIN` и `hum = UINT16_MAX` означают ошибку датчика.

### nRF52 (GATT сервер)
- Файл: `main.c`
- Добавлен сервис `SensorData` и обработчик записи `sensor_payload_process` (логирует значения).
- UUID сервиса регистрируется через `sd_ble_uuid_vs_add`, добавлен в рекламные данные.

### ESP32 (центральное устройство)
- Файл: `esp32/Sensors_App/Sensors_App.ino`
- После обнаружения устройства `Game_Score` ESP32 подключается как Central, находит сервис/характеристику и каждые 2 с отправляет структуру `SensorPayload`.
- Переподключение происходит автоматически после обрыва.

