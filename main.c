#include "SEGGER_RTT.h"
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include "nrf.h"
#include "radio_config.h"

// Макросы логирования
#define LOG_INFO(...)  SEGGER_RTT_printf(0, "[INFO] " __VA_ARGS__); SEGGER_RTT_printf(0, "\r\n")
#define LOG_ERROR(...) SEGGER_RTT_printf(0, "[ERROR] " __VA_ARGS__); SEGGER_RTT_printf(0, "\r\n")
#define LOG_DEBUG(...) SEGGER_RTT_printf(0, "[DEBUG] " __VA_ARGS__); SEGGER_RTT_printf(0, "\r\n")

// Радио (должно совпадать с передатчиком)
#define RADIO_CHANNEL       10
#define RADIO_PAYLOAD_LEN   sizeof(packet_t)
#define RADIO_PDU_LEN       (2 + RADIO_PAYLOAD_LEN)

static uint8_t rx_pdu[RADIO_PDU_LEN];

static volatile bool radio_packet_ready = false;
static volatile uint32_t radio_address_matches = 0;
static volatile uint32_t radio_crc_errors_isr = 0;

static packet_t rx_packet;

// ========== GPIO/LED ==========
#define LED_BLUE_PIN  15  // P0.15
#define LED_GREEN_PIN 14  // P0.14

// Используем прямую работу с регистрами GPIO через указатели
#define GPIO_BASE      0x50000000UL
#define GPIO_OUTSET    (*(volatile uint32_t *)(GPIO_BASE + 0x508))
#define GPIO_OUTCLR    (*(volatile uint32_t *)(GPIO_BASE + 0x50C))
#define GPIO_PIN_CNF(n) (*(volatile uint32_t *)(GPIO_BASE + 0x700 + (n) * 4))

static void led_init(void) {
    // Настройка пинов как выходы
    GPIO_PIN_CNF(LED_BLUE_PIN) = (1 << 0);  // DIR = Output
    GPIO_PIN_CNF(LED_GREEN_PIN) = (1 << 0); // DIR = Output
    
    // Выключить все LED
    GPIO_OUTCLR = (1UL << LED_BLUE_PIN) | (1UL << LED_GREEN_PIN);
}

static void led_set_blue(void) {
    GPIO_OUTCLR = (1UL << LED_GREEN_PIN);
    GPIO_OUTSET = (1UL << LED_BLUE_PIN);
}

static void led_set_green(void) {
    GPIO_OUTCLR = (1UL << LED_BLUE_PIN);
    GPIO_OUTSET = (1UL << LED_GREEN_PIN);
}

// ========== РАДИО ==========

static void radio_init(void) {
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}
    
    NRF_RADIO->MODE    = (RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);
    NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);
    
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
    
    NRF_RADIO->BASE0 = 0xAAAAAAAAUL;
    NRF_RADIO->PREFIX0 = 0;
    NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Msk;
    
    NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                        (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCINIT = 0xFFFFUL;
    NRF_RADIO->CRCPOLY = 0x00065B;
    
    NRF_RADIO->FREQUENCY = RADIO_CHANNEL;
    NRF_RADIO->PACKETPTR = (uint32_t)rx_pdu;
    
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk |
                         RADIO_SHORTS_END_DISABLE_Msk);

    NRF_RADIO->INTENSET = (RADIO_INTENSET_ADDRESS_Msk |
                           RADIO_INTENSET_END_Msk |
                           RADIO_INTENSET_DISABLED_Msk);
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_SetPriority(RADIO_IRQn, 1);
    NVIC_EnableIRQ(RADIO_IRQn);
    
    LOG_INFO("Radio RX configured (ch=%d)", RADIO_CHANNEL);
}

static void radio_start_rx(void) {
    NRF_RADIO->PACKETPTR = (uint32_t)rx_pdu;
    NRF_RADIO->EVENTS_ADDRESS = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_RXEN = RADIO_TASKS_RXEN_TASKS_RXEN_Trigger;
}

static void radio_handle_packet_from_isr(void) {
    // Копируем данные из PDU в структуру пакета
    uint8_t *pkt_bytes = (uint8_t *)&rx_packet;
    for (uint8_t i = 0; i < RADIO_PAYLOAD_LEN; i++) {
        pkt_bytes[i] = rx_pdu[2 + i];
    }
    radio_packet_ready = true;
}

void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_ADDRESS && (NRF_RADIO->INTENSET & RADIO_INTENSET_ADDRESS_Msk)) {
        NRF_RADIO->EVENTS_ADDRESS = 0;
        radio_address_matches++;
    }

    if (NRF_RADIO->EVENTS_END && (NRF_RADIO->INTENSET & RADIO_INTENSET_END_Msk)) {
        NRF_RADIO->EVENTS_END = 0;

        if ((NRF_RADIO->CRCSTATUS == 1) && (rx_pdu[1] == RADIO_PAYLOAD_LEN)) {
            radio_handle_packet_from_isr();
        } else {
            radio_crc_errors_isr++;
        }
    }

    if (NRF_RADIO->EVENTS_DISABLED && (NRF_RADIO->INTENSET & RADIO_INTENSET_DISABLED_Msk)) {
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->PACKETPTR = (uint32_t)rx_pdu;
        NRF_RADIO->TASKS_RXEN = RADIO_TASKS_RXEN_TASKS_RXEN_Trigger;
    }
}

// ========== MAIN ==========

int main(void) {
    SEGGER_RTT_Init();
    
    LOG_INFO("=== RECEIVER ===");
    
    led_init();
    led_set_blue(); // Синий - ожидание
    
    radio_init();
    
    radio_start_rx();
    
    uint32_t packet_count = 0;
    
    while (1) {

        if (radio_packet_ready) {
            __disable_irq();
            packet_t pkt = rx_packet;
            radio_packet_ready = false;
            __enable_irq();

            packet_count++;
            
            // Выключить синий, включить зеленый - данные пришли
            GPIO_OUTCLR = (1UL << LED_BLUE_PIN);
            GPIO_OUTSET = (1UL << LED_GREEN_PIN);

            // Output data in readable format
            LOG_INFO("=== RX Packet #%d ===", packet_count);
            LOG_INFO("Water: %u", pkt.water);
            LOG_INFO("Light: %u", pkt.light);
            
            if (pkt.temp_substrate != INT16_MIN) {
                LOG_INFO("Substrate temperature: %d.%d°C", 
                         pkt.temp_substrate / 10, pkt.temp_substrate % 10);
            } else {
                LOG_INFO("Substrate temperature: ERROR");
            }
            
            if (pkt.temp_air != INT16_MIN) {
                LOG_INFO("Air temperature: %d.%d°C", 
                         pkt.temp_air / 10, pkt.temp_air % 10);
            } else {
                LOG_INFO("Air temperature: ERROR");
            }
            
            if (pkt.humidity != UINT16_MAX) {
                LOG_INFO("Humidity: %u.%u%%", 
                         pkt.humidity / 10, pkt.humidity % 10);
            } else {
                LOG_INFO("Humidity: ERROR");
            }
            
            // Задержка 1 секунда с зеленым LED
            for (volatile uint32_t i = 0; i < 1000000; i++) {
                __NOP();
            }
            
            // Выключить зеленый, включить синий - ожидание следующего пакета
            GPIO_OUTCLR = (1UL << LED_GREEN_PIN);
            GPIO_OUTSET = (1UL << LED_BLUE_PIN);
        }
        
        // Небольшая задержка
        for (volatile uint32_t i = 0; i < 10000; i++) {
            __NOP();
        }
    }
    
    return 0;
}
