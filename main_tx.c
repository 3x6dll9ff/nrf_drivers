#include "SEGGER_RTT.h"
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include "nrf.h"
#include "radio_config.h"

// Макросы логирования
#define LOG_INFO(...)  SEGGER_RTT_printf(0, "[INFO] " __VA_ARGS__); SEGGER_RTT_printf(0, "\r\n")
#define LOG_ERROR(...) SEGGER_RTT_printf(0, "[ERROR] " __VA_ARGS__); SEGGER_RTT_printf(0, "\r\n")

// Радио
// RADIO_CHANNEL определен в radio_config.h, переопределяем
#undef RADIO_CHANNEL
#define RADIO_CHANNEL       10
#define RADIO_PAYLOAD_LEN   sizeof(packet_t)
#define RADIO_PDU_LEN       (2 + RADIO_PAYLOAD_LEN) // header + length + payload

static uint8_t pdu[RADIO_PDU_LEN] = {0};

// Глобальные переменные
static volatile bool radio_tx_busy = false;
static volatile bool radio_tx_done = true;

// ========== ADC/SENSORS ==========
#define SAADC_BASE          0x40007000UL

// SAADC регистры (прямой доступ)
#define SAADC_ENABLE        (*(volatile uint32_t *)(SAADC_BASE + 0x500))
#define SAADC_RESOLUTION    (*(volatile uint32_t *)(SAADC_BASE + 0x5F0))
#define SAADC_OVERSAMPLE    (*(volatile uint32_t *)(SAADC_BASE + 0x5F4))
#define SAADC_CH0_PSELP     (*(volatile uint32_t *)(SAADC_BASE + 0x510))
#define SAADC_CH0_CONFIG    (*(volatile uint32_t *)(SAADC_BASE + 0x518))
#define SAADC_CH1_PSELP     (*(volatile uint32_t *)(SAADC_BASE + 0x520))
#define SAADC_CH1_CONFIG    (*(volatile uint32_t *)(SAADC_BASE + 0x528))
#define SAADC_RESULT_PTR    (*(volatile uint32_t *)(SAADC_BASE + 0x62C))
#define SAADC_RESULT_MAXCNT (*(volatile uint32_t *)(SAADC_BASE + 0x630))
#define SAADC_TASKS_START   (*(volatile uint32_t *)(SAADC_BASE + 0x000))
#define SAADC_TASKS_SAMPLE  (*(volatile uint32_t *)(SAADC_BASE + 0x004))  // ВАЖНО: 0x004, а не 0x008!
#define SAADC_EVENTS_STARTED (*(volatile uint32_t *)(SAADC_BASE + 0x100))
#define SAADC_EVENTS_END    (*(volatile uint32_t *)(SAADC_BASE + 0x108))
#define SAADC_EVENTS_DONE   (*(volatile uint32_t *)(SAADC_BASE + 0x118))

// Пины сенсоров
#define LIGHT_ADC_PIN       2   // P0.02 (AIN0)
#define WATER_ADC_PIN       3   // P0.03 (AIN1)
#define DS18B20_PIN         4   // P0.04 (OneWire для DS18B20 - temp_substrate)
#define DHT22_PIN           5   // P0.05 (DHT22 - temp_air + humidity)

// Альтернативные пины для DS18B20 (если P0.06 не подойдет):
// P0.07, P0.08, P0.09, P0.10, P0.11, P0.12
// Или P0.17-P0.31 (много свободных пинов)

// SAADC константы (из старого рабочего кода)
#define SAADC_PSELP_AIN0    1   // Analog input 0 (P0.02) - ВАЖНО: 1, а не 2!
#define SAADC_PSELP_AIN1    2   // Analog input 1 (P0.03) - ВАЖНО: 2, а не 3!
#define SAADC_RESOLUTION_12BIT 2  // 12-bit resolution (0-4095)
#define SAADC_GAIN_1_6      1   // Gain 1/6 (бит 8)
#define SAADC_REFERENCE_INTERNAL 0  // Internal reference (0.6V)
#define SAADC_ACQTIME_10US  3   // Acquisition time 10us (биты 16-20)

static int16_t adc_result[2] = {0, 0};

static void saadc_init(void) {
    // Включаем SAADC
    SAADC_ENABLE = 1;
    
    // Разрешение 12 бит (0-4095) - как в старом коде
    SAADC_RESOLUTION = SAADC_RESOLUTION_12BIT;
    
    // Без oversampling
    SAADC_OVERSAMPLE = 0;
    
    // Result buffer
    SAADC_RESULT_PTR = (uint32_t)adc_result;
    SAADC_RESULT_MAXCNT = 1;
    
    // SAADC initialized
}

static uint16_t read_adc_channel(uint8_t channel) {
    // Настраиваем канал 0 на нужный пин (как в старом коде)
    uint32_t pselp = 0;
    if (channel == 0) {
        pselp = SAADC_PSELP_AIN0;  // 1 для AIN0 (P0.02)
    } else {
        pselp = SAADC_PSELP_AIN1;  // 2 для AIN1 (P0.03)
    }
    
    SAADC_CH0_PSELP = pselp;
    
    // CONFIG: RESP=0, RESN=0, GAIN=1/6 (бит 8), REFSEL=0, TACQ=10us (биты 16-20), MODE=SE
    SAADC_CH0_CONFIG = (0 << 0) | (0 << 4) | (SAADC_GAIN_1_6 << 8) | 
                       (SAADC_REFERENCE_INTERNAL << 12) | 
                       (SAADC_ACQTIME_10US << 16) | (0 << 20);
    
    // Очищаем событие END
    SAADC_EVENTS_END = 0;
    
    // Запускаем ADC (как в старом коде - без ожидания STARTED)
    SAADC_TASKS_START = 1;
    
    // Сразу запускаем семплирование
    SAADC_TASKS_SAMPLE = 1;
    
    // Ждем END (как в старом коде)
    while (SAADC_EVENTS_END == 0) {
        __NOP();
    }
    
    // Очищаем событие
    SAADC_EVENTS_END = 0;
    
    // Читаем результат (12-bit: 0-4095)
    int16_t raw_value = adc_result[0];
    
           // Логирование убрано для чистоты вывода
    
    // Конвертируем в положительное значение (0-4095 для 12-bit)
    if (raw_value < 0) {
        raw_value = 0;
    } else if (raw_value > 4095) {
        raw_value = 4095;
    }
    
    return (uint16_t)raw_value;
}

static uint16_t read_adc(uint8_t pin) {
    uint16_t value = 0;
    if (pin == LIGHT_ADC_PIN) {
        value = read_adc_channel(0);
        // Логирование убрано
    } else if (pin == WATER_ADC_PIN) {
        value = read_adc_channel(1);
        // Логирование убрано
    }
    return value;
}

// GPIO макросы для сенсоров
#define GPIO_BASE          0x50000000UL
#define GPIO_OUTSET         (*(volatile uint32_t *)(GPIO_BASE + 0x508))
#define GPIO_OUTCLR         (*(volatile uint32_t *)(GPIO_BASE + 0x50C))
#define GPIO_IN             (*(volatile uint32_t *)(GPIO_BASE + 0x510))
#define GPIO_DIRSET         (*(volatile uint32_t *)(GPIO_BASE + 0x514))
#define GPIO_DIRCLR         (*(volatile uint32_t *)(GPIO_BASE + 0x518))
#define GPIO_PIN_CNF(n)     (*(volatile uint32_t *)(GPIO_BASE + 0x700 + (n) * 4))

#define GPIO_PIN_SET(pin)   (GPIO_OUTSET = (1UL << (pin)))
#define GPIO_PIN_CLR(pin)   (GPIO_OUTCLR = (1UL << (pin)))
#define GPIO_PIN_READ(pin)  ((GPIO_IN >> (pin)) & 1)
#define GPIO_PIN_OUT(pin)   (GPIO_DIRSET = (1UL << (pin)))
#define GPIO_PIN_IN(pin)    (GPIO_DIRCLR = (1UL << (pin)))

// DWT для точных задержек (как в старом коде)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL   (*(volatile uint32_t *)0xE0001000)
#define DEMCR      (*(volatile uint32_t *)0xE000EDFC)

static void dwt_init(void) {
    if (!(DEMCR & 0x01000000)) {
        DEMCR |= 0x01000000;
        DWT_CTRL |= 1;
    }
}

static void delay_us(uint32_t us) {
    uint32_t start = DWT_CYCCNT;
    uint32_t cycles = us * 64; // 64MHz clock
    while ((DWT_CYCCNT - start) < cycles) {
        __NOP();
    }
}

// delay_ms используем из radio_config.h, но для DHT22 нужен DWT
// Переопределяем для использования DWT
static void delay_ms_dwt(uint32_t ms) {
    delay_us(ms * 1000);
}

static void sensors_gpio_init(void) {
    // DS18B20 (P0.04) - OneWire
    // ВАЖНО: Для стабильной работы нужен ВНЕШНИЙ подтягивающий резистор 4.7 кОм
    // между DQ и VDD! Внутренний pullup nRF52840 (~11 кОм) может быть недостаточным.
    // Используем обычный режим с pullup, переключение DIR для open-drain эмуляции
    GPIO_PIN_CNF(DS18B20_PIN) = (0 << 0) |  // DIR = Input (будет переключаться)
                                (0 << 1) |  // INPUT = Connect
                                (3 << 2) |  // PULL = Pullup (внутренний, но лучше внешний 4.7k)
                                (0 << 8) |  // DRIVE = S0S1 (обычный режим)
                                (0 << 16);  // SENSE = Disabled
    
    // DHT22 (P0.05) - обычный GPIO
    GPIO_PIN_CNF(DHT22_PIN) = (0 << 0) |   // DIR = Input
                              (0 << 1) |   // INPUT = Connect
                              (3 << 2) |   // PULL = Pullup
                              (0 << 8) |   // DRIVE = S0S1
                              (0 << 16);   // SENSE = Disabled
}

// ========== OneWire для DS18B20 (адаптировано из стандартной библиотеки) ==========
static void onewire_low(uint8_t pin) {
    GPIO_PIN_OUT(pin);
    __asm volatile("dsb" ::: "memory");
    GPIO_PIN_CLR(pin);
    __asm volatile("dsb" ::: "memory");
}

static void onewire_high(uint8_t pin) {
    GPIO_PIN_IN(pin);  // Отпускаем линию (pullup подтянет вверх)
    __asm volatile("dsb" ::: "memory");
    // Небольшая задержка для стабилизации после переключения режима
    __NOP(); __NOP(); __NOP(); __NOP();
}

// Чтение бита (точные тайминги из стандартной библиотеки OneWire)
static uint8_t onewire_read_bit(uint8_t pin) {
    uint8_t bit = 0;
    
    __disable_irq();
    
    // Мастер тянет линию вниз минимум на 1us
    onewire_low(pin);
    delay_us(6);  // Увеличено для надежности
    
    // Освобождаем линию
    onewire_high(pin);
    
    // Ждем 9us после освобождения (сенсор должен ответить)
    // В стандартной библиотеке: 1us низко, затем 9us ждать = 10us от начала
    delay_us(9);
    
    // Читаем состояние линии
    bit = GPIO_PIN_READ(pin);
    
    // Завершаем таймслот (минимум 45us recovery time)
    delay_us(55);  // 6 + 9 + 55 = 70us total
    
    __enable_irq();
    
    return bit;
}

// Запись бита (точные тайминги из стандартной библиотеки OneWire)
static void onewire_write_bit(uint8_t pin, uint8_t bit) {
    __disable_irq();
    
    // Мастер тянет линию вниз минимум на 1us
    onewire_low(pin);
    delay_us(1);
    
    // Если бит = 1, сразу отпускаем (pullup подтянет вверх)
    // Если бит = 0, продолжаем держать низко
    if (bit) {
        onewire_high(pin);
    }
    
    // Ждем 60us от начала таймслота
    delay_us(59);  // 1 + 59 = 60us
    
    // Отпускаем линию (если еще не отпущена)
    onewire_high(pin);
    
    // Recovery time 1us
    delay_us(1);
    
    __enable_irq();
}

static uint8_t onewire_read_byte(uint8_t pin) {
    uint8_t byte = 0;
    // Читаем биты с младшего к старшему (LSB first)
    for (uint8_t i = 0; i < 8; i++) {
        if (onewire_read_bit(pin)) {
            byte |= (1 << i);  // Устанавливаем бит i
        }
    }
    return byte;
}

static void onewire_write_byte(uint8_t pin, uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        onewire_write_bit(pin, (byte >> i) & 1);
    }
}

static bool onewire_reset(uint8_t pin) {
    // Отключаем прерывания для точных таймингов
    __disable_irq();
    
    onewire_low(pin);
    delay_us(480);  // 480us reset pulse
    onewire_high(pin);
    delay_us(70);   // Ждем presence pulse
    uint8_t pin_state = GPIO_PIN_READ(pin);
    uint8_t presence = !pin_state;  // Должен быть низкий
    delay_us(410);  // Завершаем reset
    
    // Включаем прерывания обратно
    __enable_irq();
    
    // Логирование убрано
    return presence;
}

static int16_t read_temp_substrate(void) {
    uint8_t pin = DS18B20_PIN;
    uint8_t data[9];
    
    // Reset и проверка presence
    if (!onewire_reset(pin)) {
        LOG_ERROR("DS18B20 (P0.%02d): No presence pulse - sensor not responding", pin);
        return INT16_MIN;  // Ошибка: сенсор не отвечает
    }
    
    // Skip ROM (0xCC) - если один сенсор на шине
    onewire_write_byte(pin, 0xCC);
    
    // Convert T (0x44)
    onewire_write_byte(pin, 0x44);
    
    // Ждем конвертацию (750ms для 12-bit resolution)
    onewire_high(pin);
    delay_ms_dwt(750);
    
    // Reset и чтение данных
    if (!onewire_reset(pin)) {
        return INT16_MIN;
    }
    
    // Skip ROM
    onewire_write_byte(pin, 0xCC);
    
    // Read Scratchpad (0xBE)
    onewire_write_byte(pin, 0xBE);
    
    // Задержка перед началом чтения данных
    delay_us(10);
    
    // Читаем 9 байт
    for (uint8_t i = 0; i < 9; i++) {
        data[i] = onewire_read_byte(pin);
        delay_us(10);  // Небольшая задержка между байтами
    }
    
    // Логирование убрано
    
    // Проверка CRC (упрощенная - проверяем только что не все 0xFF)
    if (data[0] == 0xFF && data[1] == 0xFF) {
        LOG_ERROR("DS18B20 (P0.%02d): Invalid data (0xFF)", pin);
        return INT16_MIN;  // Ошибка чтения
    }
    
    // Температура в формате DS18B20: младший байт = целая часть, старший = дробная
    int16_t raw_temp = (int16_t)((data[1] << 8) | data[0]);
    // Конвертируем в градусы * 10 (например, 23.5°C = 235)
    int16_t temp_cx10 = (raw_temp * 10) / 16;  // DS18B20: 0.0625°C per bit
    
    // Логирование убрано
    
    return temp_cx10;
}

// ========== DHT22 ==========
static bool dht22_read(uint8_t pin, uint16_t *humidity, int16_t *temperature) {
    uint8_t data[5] = {0};
    
    // 1. Старт: пин в выход, низкий уровень >18ms
    GPIO_PIN_OUT(pin);
    GPIO_PIN_CLR(pin);
    delay_ms_dwt(20);  // >18ms как в старом коде
    
    GPIO_PIN_SET(pin);
    delay_us(30);
    GPIO_PIN_IN(pin);
    
    // 2. Ждем ответа (Low 80us, High 80us)
    uint32_t timeout = 10000;
    while (GPIO_PIN_READ(pin) == 1 && --timeout) {
        __NOP();
    }
    if (timeout == 0) {
        LOG_ERROR("DHT22: No response (waiting for low)");
        return false;
    }
    
    timeout = 10000;
    while (GPIO_PIN_READ(pin) == 0 && --timeout) {
        __NOP();
    }
    if (timeout == 0) {
        LOG_ERROR("DHT22: Response timeout (waiting for high)");
        return false;
    }
    
    timeout = 10000;
    while (GPIO_PIN_READ(pin) == 1 && --timeout) {
        __NOP();
    }
    if (timeout == 0) {
        LOG_ERROR("DHT22: Response timeout (waiting for low after high)");
        return false;
    }
    
    // 3. Читаем 40 бит данных (как в старом коде - измеряем длительность)
    for (uint8_t i = 0; i < 40; i++) {
        // Ждем низкий уровень
        timeout = 10000;
        while (GPIO_PIN_READ(pin) == 0 && --timeout) {
            __NOP();
        }
        if (timeout == 0) {
            LOG_ERROR("DHT22: Bit %d low timeout", i);
            return false;
        }
        
        // Замеряем длительность высокого уровня
        uint32_t t_start = DWT_CYCCNT;
        
        timeout = 10000;
        while (GPIO_PIN_READ(pin) == 1 && --timeout) {
            __NOP();
        }
        if (timeout == 0) {
            LOG_ERROR("DHT22: Bit %d high timeout", i);
            return false;
        }
        
        uint32_t duration = DWT_CYCCNT - t_start;
        
        // 0 это ~26-28us (26*64 = 1664 cycles)
        // 1 это ~70us (70*64 = 4480 cycles)
        // Threshold ~50us = 3200 cycles
        if (duration > 3200) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }
    
    // Логируем сырые данные
    // Логирование убрано
    
    // 4. Проверка CRC (как в старом коде)
    uint8_t crc = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (crc != data[4]) {
        LOG_ERROR("DHT22: CRC error (calc=0x%02X, recv=0x%02X)", crc, data[4]);
        return false;
    }
    
    // 5. Конвертация данных
    *humidity = (data[0] << 8) | data[1];      // Влажность * 10
    int16_t temp_raw = (data[2] << 8) | data[3];
    
    // Температура: старший бит = знак
    if (temp_raw & 0x8000) {
        temp_raw &= 0x7FFF;
        temp_raw = -temp_raw;
    }
    *temperature = temp_raw;
    
    return true;
}

// Статическая переменная для хранения последних данных DHT22
static uint16_t dht22_last_hum = 0;
static int16_t dht22_last_temp = 0;
static uint32_t dht22_last_read_cycles = 0;
static bool dht22_data_valid = false;

// Читаем DHT22 один раз и кешируем результат
static bool dht22_read_cached(uint16_t *humidity, int16_t *temperature) {
    uint32_t current_cycles = DWT_CYCCNT;
    uint32_t cycles_since_last = 0;
    
    if (dht22_last_read_cycles != 0) {
        if (current_cycles > dht22_last_read_cycles) {
            cycles_since_last = current_cycles - dht22_last_read_cycles;
        } else {
            // Переполнение счетчика
            cycles_since_last = (0xFFFFFFFF - dht22_last_read_cycles) + current_cycles;
        }
    }
    
    // DHT22 требует минимум 2 секунды между чтениями (2 * 64MHz = 128000000 cycles)
    uint32_t min_cycles = 128000000;
    
    if (dht22_data_valid && cycles_since_last < min_cycles) {
        // Используем кешированные данные
        *humidity = dht22_last_hum;
        *temperature = dht22_last_temp;
        return true;
    }
    
    // Читаем новые данные
    if (dht22_read(DHT22_PIN, humidity, temperature)) {
        dht22_last_hum = *humidity;
        dht22_last_temp = *temperature;
        dht22_last_read_cycles = DWT_CYCCNT;
        dht22_data_valid = true;
        return true;
    }
    
    // При ошибке используем последние данные, если они есть
    if (dht22_data_valid) {
        *humidity = dht22_last_hum;
        *temperature = dht22_last_temp;
        return true;
    }
    
    return false;
}

static int16_t read_temp_air(void) {
    uint16_t hum;
    int16_t temp;
    
    if (dht22_read_cached(&hum, &temp)) {
        // Логирование убрано
        return temp;
    }
    
    LOG_ERROR("DHT22 (P0.05): Read failed");
    return INT16_MIN;
}

static uint16_t read_humidity(void) {
    uint16_t hum;
    int16_t temp;
    
    if (dht22_read_cached(&hum, &temp)) {
        return hum;
    }
    
    return UINT16_MAX;
}

// ========== РАДИО ==========

static void radio_init(void) {
    // Запускаем высокочастотный генератор
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}
    
    NRF_RADIO->MODE    = (RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);
    NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);
    
    NRF_RADIO->PCNF0 = (8 << RADIO_PCNF0_LFLEN_Pos) |
                       (1 << RADIO_PCNF0_S0LEN_Pos) |
                       (0 << RADIO_PCNF0_S1LEN_Pos) |
                       (2 << RADIO_PCNF0_CILEN_Pos) |
                       (RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos) |
                       (3 << RADIO_PCNF0_TERMLEN_Pos);
    
    NRF_RADIO->PCNF1 = (RADIO_PDU_LEN << RADIO_PCNF1_MAXLEN_Pos) |
                       (0 << RADIO_PCNF1_STATLEN_Pos) |
                       (3 << RADIO_PCNF1_BALEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
                       (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);
    
    NRF_RADIO->BASE0     = 0xAAAAAAAAUL;
    NRF_RADIO->TXADDRESS = 0;
    NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Msk;
    NRF_RADIO->TIFS      = 1000U;
    
    NRF_RADIO->CRCCNF  = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                         (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCINIT = 0xFFFFUL;
    NRF_RADIO->CRCPOLY = 0x00065B;
    
    NRF_RADIO->FREQUENCY = RADIO_CHANNEL;
    NRF_RADIO->PACKETPTR = (uint32_t)pdu;
    
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk |
                         RADIO_SHORTS_END_DISABLE_Msk);

    NRF_RADIO->INTENSET = (RADIO_INTENSET_END_Msk |
                           RADIO_INTENSET_DISABLED_Msk);
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_SetPriority(RADIO_IRQn, 1);
    NVIC_EnableIRQ(RADIO_IRQn);
    
    LOG_INFO("Radio configured (channel %d)", RADIO_CHANNEL);
}

static void prepare_payload(packet_t *packet) {
    // Читаем данные сенсоров
    packet->len = sizeof(packet_t) - 1; // длина без поля len
    packet->light = read_adc(LIGHT_ADC_PIN);
    packet->water = read_adc(WATER_ADC_PIN);
    packet->temp_substrate = read_temp_substrate();
    packet->temp_air = read_temp_air();
    packet->humidity = read_humidity();
    
    // Упаковываем в PDU
    pdu[0] = 0x00;
    pdu[1] = RADIO_PAYLOAD_LEN;
    uint8_t *pkt_bytes = (uint8_t *)packet;
    for (uint8_t i = 0; i < RADIO_PAYLOAD_LEN; i++) {
        pdu[2 + i] = pkt_bytes[i];
    }
}

static bool radio_start_tx(void) {
    if (radio_tx_busy) {
        return false;
    }

    radio_tx_busy = true;
    radio_tx_done = false;

    NRF_RADIO->PACKETPTR = (uint32_t)pdu;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_TXEN = RADIO_TASKS_TXEN_TASKS_TXEN_Trigger;
    return true;
}

static void radio_wait_for_tx(void) {
    while (!radio_tx_done) {
        __WFE();
    }
}

void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;
    }

    if (NRF_RADIO->EVENTS_DISABLED) {
        NRF_RADIO->EVENTS_DISABLED = 0;
        radio_tx_busy = false;
        radio_tx_done = true;
        __SEV();
    }
}

static bool radio_send_packet(void) {
    static uint32_t tx_count = 0;

    if (radio_tx_busy) {
        radio_wait_for_tx();
    }

    if (!radio_start_tx()) {
        return false;
    }

    radio_wait_for_tx();

    tx_count++;
    if (tx_count <= 3) {
        LOG_INFO("TX #%d done", tx_count);
    }
    return true;
}

// ========== MAIN ==========

int main(void) {
    SEGGER_RTT_Init();
    
    LOG_INFO("=== TRANSMITTER ===");
    
    dwt_init();  // Инициализируем DWT для точных задержек
    sensors_gpio_init();
    saadc_init();
    radio_init();
    
    // Sensors initialized
    
    uint32_t packet_count = 0;
    packet_t sensor_packet;
    
    while (1) {
        packet_count++;
        
        prepare_payload(&sensor_packet);
        
        if (radio_send_packet()) {
            // Output data in readable format
            LOG_INFO("=== TX Packet #%d ===", packet_count);
            LOG_INFO("Water: %u", sensor_packet.water);
            LOG_INFO("Light: %u", sensor_packet.light);
            
            if (sensor_packet.temp_substrate != INT16_MIN) {
                LOG_INFO("Substrate temperature: %d.%d°C", 
                         sensor_packet.temp_substrate / 10, sensor_packet.temp_substrate % 10);
            } else {
                LOG_INFO("Substrate temperature: ERROR");
            }
            
            if (sensor_packet.temp_air != INT16_MIN) {
                LOG_INFO("Air temperature: %d.%d°C", 
                         sensor_packet.temp_air / 10, sensor_packet.temp_air % 10);
            } else {
                LOG_INFO("Air temperature: ERROR");
            }
            
            if (sensor_packet.humidity != UINT16_MAX) {
                LOG_INFO("Humidity: %u.%u%%", 
                         sensor_packet.humidity / 10, sensor_packet.humidity % 10);
            } else {
                LOG_INFO("Humidity: ERROR");
            }
        } else {
            LOG_ERROR("Packet #%d send failed", packet_count);
        }
        
        // Задержка 15 секунд
        for (volatile uint32_t i = 0; i < 15000000; i++) {
            __NOP();
        }
    }
    
    return 0;
}
