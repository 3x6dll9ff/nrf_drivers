#include "radio_config.h"
#include "third_party/external/segger_rtt/SEGGER_RTT.h"

static packet_t tx_packet;

// ========== ADC HARDWARE ==========
#define SAADC ((volatile uint32_t *)0x40007000)
#define SAADC_ENABLE (SAADC[0x500 / 4])
#define SAADC_CH0_PSELP (SAADC[0x510 / 4])
#define SAADC_CH0_CONFIG (SAADC[0x514 / 4])
#define SAADC_RESOLUTION (SAADC[0x5F0 / 4])
#define SAADC_RESULT_PTR (SAADC[0x62C / 4])
#define SAADC_RESULT_MAXCNT (SAADC[0x630 / 4])
#define SAADC_TASKS_START (SAADC[0x000 / 4])
#define SAADC_TASKS_SAMPLE (SAADC[0x004 / 4])
#define SAADC_EVENTS_END (SAADC[0x104 / 4])

// Sensors
#define LIGHT_SENSOR_PIN 2 // P0.02 = AIN0
#define WATER_SENSOR_PIN 3 // P0.03 = AIN1

static int16_t adc_result;

// ========== UTILITIES ==========
// DWT for precise timing
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000)
#define DEMCR (*(volatile uint32_t *)0xE000EDFC)

void delay_us(uint32_t us) {
  uint32_t start = DWT_CYCCNT;
  uint32_t cycles = us * 64; // 64MHz clock
  while ((DWT_CYCCNT - start) < cycles)
    ;
}

// Simple pseudo-random number generator for simulation
static uint32_t seed = 123456789;
uint32_t rand_sim(void) {
  seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
  return seed;
}

// ========== DHT22 DRIVER ==========
#define DHT_PIN 4 // P0.04

// Helper to set pin mode
void pin_cfg_output(int pin) {
  G_PIN_CNF(pin) = 1; // Output
}

void pin_cfg_input(int pin) {
  G_PIN_CNF(pin) = 0; // Input, No pull
}

void pin_set(int pin, int val) {
  if (val)
    G_OUTSET = (1 << pin);
  else
    G_OUTCLR = (1 << pin);
}

int pin_read(int pin) {
  return (GPIO[0x510 / 4] & (1 << pin)) ? 1 : 0; // IN register
}

// Returns 1 on success, 0 on timeout/error
int dht_read(int16_t *temp, uint16_t *hum) {
  uint8_t data[5] = {0};

  // Enable DWT
  if (!(DEMCR & 0x01000000)) {
    DEMCR |= 0x01000000;
    DWT_CTRL |= 1;
  }

  // Start signal
  pin_cfg_output(DHT_PIN);
  pin_set(DHT_PIN, 0);
  delay_ms(20); // > 18ms
  pin_set(DHT_PIN, 1);
  delay_us(30);
  pin_cfg_input(DHT_PIN);

  // Wait for response (Low 80us, High 80us)
  // Timeout counter
  uint32_t timeout = 10000;
  while (pin_read(DHT_PIN) == 1 && --timeout)
    ;
  if (timeout == 0)
    return 0;

  timeout = 10000;
  while (pin_read(DHT_PIN) == 0 && --timeout)
    ;
  if (timeout == 0)
    return 0;

  timeout = 10000;
  while (pin_read(DHT_PIN) == 1 && --timeout)
    ;
  if (timeout == 0)
    return 0;

  // Read 40 bits
  for (int i = 0; i < 40; i++) {
    timeout = 10000;
    while (pin_read(DHT_PIN) == 0 && --timeout)
      ;
    if (timeout == 0)
      return 0;

    uint32_t t = DWT_CYCCNT;
    timeout = 10000;
    while (pin_read(DHT_PIN) == 1 && --timeout)
      ;
    if (timeout == 0)
      return 0;

    uint32_t duration = DWT_CYCCNT - t;
    // 0 is ~26-28us (26*64 = 1664 cycles)
    // 1 is ~70us (70*64 = 4480 cycles)
    // Threshold ~50us = 3200 cycles

    if (duration > 3200) {
      data[i / 8] |= (1 << (7 - (i % 8)));
    }
  }

  // Verify Checksum
  if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    return 0; // Checksum error
  }

  // Parse Data (DHT22)
  // Hum: data[0]<<8 | data[1] (x10)
  // Temp: data[2]<<8 | data[3] (x10, MSB is sign)

  *hum = (data[0] << 8) | data[1];

  int16_t t_raw = (data[2] << 8) | data[3];
  if (t_raw & 0x8000) {
    t_raw &= 0x7FFF;
    t_raw = -t_raw;
  }
  *temp = t_raw;

  return 1;
}

// ========== RTT DEBUG OUTPUT ==========
#define LOG(msg) SEGGER_RTT_WriteString(0, msg)
#define LOG_INT(val) SEGGER_RTT_printf(0, "%d", val)
#define LOG_UINT(val) SEGGER_RTT_printf(0, "%u", val)

// ========== GPIO ==========
void gpio_init(void) {
  G_PIN_CNF(RGB_RED) = 1;   // Output
  G_PIN_CNF(RGB_GREEN) = 1; // Output
  G_PIN_CNF(RGB_BLUE) = 1;  // Output
  G_PIN_CNF(LED4) = 1;      // Output

  // Turn off all LEDs
  G_OUTCLR = (1 << RGB_RED) | (1 << RGB_GREEN) | (1 << RGB_BLUE) | (1 << LED4);
}

void rgb_set(uint8_t red, uint8_t green, uint8_t blue) {
  if (red)
    G_OUTSET = (1 << RGB_RED);
  else
    G_OUTCLR = (1 << RGB_RED);

  if (green)
    G_OUTSET = (1 << RGB_GREEN);
  else
    G_OUTCLR = (1 << RGB_GREEN);

  if (blue)
    G_OUTSET = (1 << RGB_BLUE);
  else
    G_OUTCLR = (1 << RGB_BLUE);
}

// ========== ADC ==========
void adc_init(void) {
  // Enable SAADC
  SAADC_ENABLE = 1;

  // Configure Channel 0: AIN0 (P0.02)
  // PSELP: 1 = AIN0
  SAADC_CH0_PSELP = 1;

  // CONFIG: RESP=0 (Bypass), RESN=0 (Bypass), GAIN=1/6, REFSEL=0 (Internal
  // 0.6V), TACQ=10us, MODE=SE Gain 1/6 gives range 0-3.6V
  SAADC_CH0_CONFIG =
      (0 << 0) | (0 << 4) | (1 << 8) | (0 << 12) | (3 << 16) | (0 << 20);

  // Resolution: 12-bit
  SAADC_RESOLUTION = 2;

  // Result buffer
  SAADC_RESULT_PTR = (uint32_t)&adc_result;
  SAADC_RESULT_MAXCNT = 1;
}

uint16_t adc_read(uint8_t pin) {
  // Configure Channel 0 to the requested pin
  // PSELP: 1=AIN0(P0.02), 2=AIN1(P0.03)
  uint32_t pselp = 0;
  if (pin == 2)
    pselp = 1; // AIN0
  else if (pin == 3)
    pselp = 2; // AIN1
  else
    return 0;

  SAADC_CH0_PSELP = pselp;
  SAADC_CH0_CONFIG = (0 << 0) | (0 << 4) | (1 << 8) | (0 << 12) | (3 << 16) |
                     (0 << 20); // Gain 1/6, Internal 0.6V ref

  // Clear END event
  SAADC_EVENTS_END = 0;

  // Start ADC
  SAADC_TASKS_START = 1;

  // Trigger sample
  SAADC_TASKS_SAMPLE = 1;

  // Wait for conversion
  while (SAADC_EVENTS_END == 0)
    ;

  // Clear event
  SAADC_EVENTS_END = 0;

  // Return result (12-bit: 0-4095)
  if (adc_result < 0)
    return 0;
  return (uint16_t)adc_result;
}

// ========== RADIO ==========
void radio_init(void) {
  R_MODE = 1;
  R_FREQ = RADIO_CHANNEL;
  R_TXPOWER = 0;

  // Packet configuration: 8-bit length field
  R_PCNF0 = (8 << 0); // LFLEN=8 bits (was incorrectly 1 bit!)
  R_PCNF1 = (sizeof(packet_t) << 0) | (3 << 16);

  // Address configuration
  R_BASE0 = RADIO_ADDR;
  R_PREFIX0 = 0;

  // CRC configuration: 16-bit CRC
  R_CRCCNF = 2;       // CRC length: 2 bytes (16-bit)
  R_CRCINIT = 0xFFFF; // Initial value

  // TX: Set which logical address to use (0-7)
  R_TXADDRESS = 0; // Use address 0

  R_SHORTS = (1 << 0) | (1 << 1);
}

void radio_send(packet_t *pkt) {
  LOG("[RADIO] Starting TX...\n");

  // Point to packet
  R_PACKETPTR = (uint32_t)pkt;

  // Clear events
  R_EVENT_END = 0;
  R_EVENT_READY = 0;
  R_EVENT_DISABLED = 0;

  // Enable TX
  R_TXEN = 1;
  LOG("[RADIO] TX enabled, waiting for READY...\n");

  // Wait for END (Transmission complete)
  // Because of SHORTS, it will go READY->START->TX->END->DISABLE
  while (R_EVENT_END == 0)
    ;
  LOG("[RADIO] END event received\n");

  // Clear END
  R_EVENT_END = 0;

  // Wait for DISABLED (Radio off)
  while (R_EVENT_DISABLED == 0)
    ;
  LOG("[RADIO] Radio disabled\n");

  R_EVENT_DISABLED = 0;
}

// ========== SENSOR READING ==========
void update_sensors(void) {
  // 1. Light (Photoresistor) - AIN0
  tx_packet.light = adc_read(LIGHT_SENSOR_PIN);

  // 2. Water (Soil Moisture) - AIN1
  tx_packet.water = adc_read(WATER_SENSOR_PIN);

  // 3. Temperature (Substrate) - Simulated for now
  tx_packet.temp_substrate = 200 + (rand_sim() % 100); // 20.0 - 30.0 C

  // 4. Temperature (Air) - Simulated for now
  tx_packet.temp_air = 220 + (rand_sim() % 50); // 22.0 - 27.0 C

  // 5. Humidity - Simulated for now
  tx_packet.humidity = 400 + (rand_sim() % 400); // 40.0 - 80.0 %

  tx_packet.len = sizeof(packet_t);
}

// ========== MAIN ==========
int main(void) {
  // Step 1: GPIO init
  gpio_init();

  // Step 2: UART init
  SEGGER_RTT_Init();

  // Send initial message immediately
  LOG("\n\n================================\n");
  LOG("  TRANSMITTER STARTED\n");
  LOG("  Photoresistor: P0.02 (AIN0)\n");
  LOG("================================\n\n");

  // Step 3: ADC init
  adc_init();

  // Step 4: Radio init
  radio_init();

  uint32_t packet_count = 0;

  while (1) {
    // 1. Update sensor data
    update_sensors();

    packet_count++;
    LOG("\n[TX #");
    LOG_UINT(packet_count);
    LOG("]\n");

    LOG("Light (ADC): ");
    LOG_UINT(tx_packet.light);
    LOG("\n");

    LOG("Water: ");
    LOG_UINT(tx_packet.water);
    LOG("\n");

    LOG("Temp Sub: ");
    LOG_INT(tx_packet.temp_substrate / 10);
    LOG(".");
    LOG_UINT(tx_packet.temp_substrate % 10);
    LOG(" C\n");

    LOG("Temp Air: ");
    LOG_INT(tx_packet.temp_air / 10);
    LOG(".");
    LOG_UINT(tx_packet.temp_air % 10);
    LOG(" C\n");

    LOG("Humidity: ");
    LOG_UINT(tx_packet.humidity / 10);
    LOG(".");
    LOG_UINT(tx_packet.humidity % 10);
    LOG(" %\n");

    // 2. Send data
    LOG("Sending...\n");
    radio_send(&tx_packet);
    LOG("Sent!\n");

    // 3. Wait before next transmission
    delay_ms(5000);
  }
}
