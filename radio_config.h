#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

#include <stdint.h>

// ========== HARDWARE ADDRESSES ==========
#define RADIO ((volatile uint32_t *)0x40001000)
#define GPIO ((volatile uint32_t *)0x50000000)
#define UART ((volatile uint32_t *)0x40002000)

// ========== RADIO REGISTERS ==========
#define R_RXEN (RADIO[0x004 / 4])
#define R_TXEN (RADIO[0x008 / 4]) // Added TXEN
#define R_TXPOWER (RADIO[0x50C / 4])
#define R_FREQ (RADIO[0x508 / 4])
#define R_MODE (RADIO[0x510 / 4])
#define R_PCNF0 (RADIO[0x514 / 4])
#define R_PCNF1 (RADIO[0x518 / 4])
#define R_BASE0 (RADIO[0x51C / 4])
#define R_PREFIX0 (RADIO[0x524 / 4])
#define R_RXADDRESSES (RADIO[0x530 / 4])
#define R_PACKETPTR (RADIO[0x504 / 4])
#define R_SHORTS (RADIO[0x200 / 4])
#define R_EVENT_END (RADIO[0x10C / 4])
#define R_EVENT_READY (RADIO[0x100 / 4])    // Added EVENT_READY
#define R_EVENT_DISABLED (RADIO[0x110 / 4]) // Added EVENT_DISABLED
#define R_CRCSTATUS (RADIO[0x400 / 4])
#define R_CRCCNF (RADIO[0x53C / 4])      // CRC configuration
#define R_CRCINIT (RADIO[0x540 / 4])     // CRC initial value
#define R_TXADDRESS (RADIO[0x534 / 4])   // TX address select
#define R_TASKS_START (RADIO[0x000 / 4]) // Added TASKS_START
#define R_TASKS_TXEN                                                           \
  (RADIO[0x008 / 4]) // Added TASKS_TXEN (Note: same offset as RXEN/TXEN task
                     // trigger usually)
#define R_TASKS_DISABLE (RADIO[0x010 / 4]) // Added TASKS_DISABLE

// ========== GPIO REGISTERS ==========
#define G_OUTSET (GPIO[0x508 / 4])
#define G_OUTCLR (GPIO[0x50C / 4])
#define G_PIN_CNF(n) (GPIO[(0x700 + (n) * 4) / 4])
#define G_DIR                                                                  \
  (GPIO[0x514 / 4]) // Added DIR register if needed, though PIN_CNF handles it

// ========== UART REGISTERS ==========
#define UART_STARTTX (UART[0x008 / 4])
#define UART_PSELTXD (UART[0x50C / 4])
#define UART_BAUDRATE (UART[0x524 / 4])
#define UART_ENABLE (UART[0x500 / 4])
#define UART_TXD (UART[0x51C / 4])
#define UART_EVENT_TXDRDY (UART[0x11C / 4])

// ========== PIN DEFINITIONS ==========
#define RGB_RED 13    // P0.13
#define RGB_GREEN 14  // P0.14
#define RGB_BLUE 15   // P0.15
#define LED1 13       // Board LED1 (same as RGB_RED)
#define LED4 16       // Board LED4
#define UART_TX_PIN 6 // P0.06

// ========== CONFIGURATION ==========
#define RADIO_CHANNEL 42
#define RADIO_ADDR 0x12345678

// ========== DATA PACKET ==========
typedef struct {
  uint8_t len;
  uint16_t light;
  uint16_t water;
  int16_t temp_substrate;
  int16_t temp_air;
  uint16_t humidity;
} packet_t;

// ========== UTILITIES ==========
static inline void delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms * 4000; i++)
    __asm("nop");
}

#endif // RADIO_CONFIG_H
