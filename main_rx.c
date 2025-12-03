#include "radio_config.h"
#include "third_party/external/segger_rtt/SEGGER_RTT.h"

// ========== RTT DEBUG OUTPUT ==========
#define LOG(msg) SEGGER_RTT_WriteString(0, msg)
#define LOG_INT(val) SEGGER_RTT_printf(0, "%d", val)
#define LOG_UINT(val) SEGGER_RTT_printf(0, "%u", val)

static packet_t rx_packet;

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

// ========== RADIO ==========
void radio_init(void) {
  R_MODE = 1;
  R_FREQ = RADIO_CHANNEL;
  R_TXPOWER = 0;
  R_PCNF0 = (8 << 0); // LFLEN=8 bits (must match TX!)
  R_PCNF1 = (sizeof(packet_t) << 0) | (3 << 16);
  R_BASE0 = RADIO_ADDR;
  R_PREFIX0 = 0;

  // CRC configuration: 16-bit CRC (must match TX!)
  R_CRCCNF = 2;       // CRC length: 2 bytes (16-bit)
  R_CRCINIT = 0xFFFF; // Initial value

  R_RXADDRESSES = 1; // Enable logical address 0 for RX
  R_SHORTS = (1 << 0) | (1 << 1);
}

void radio_rx_start(void) {
  R_PACKETPTR = (uint32_t)&rx_packet;
  R_EVENT_END = 0;
  R_RXEN = 1; // Trigger RXEN task
}

uint8_t radio_rx_check(void) {
  if (R_EVENT_END) {
    LOG("[DEBUG] Packet received! ");
    R_EVENT_END = 0;
    if (R_CRCSTATUS == 1) {
      LOG("CRC OK\n");
      return 1;
    }
    LOG("CRC FAILED\n");
    // If CRC failed, restart immediately
    R_TASKS_START = 1; // Or just re-trigger RXEN if disabled?
    // If SHORTS END_DISABLE is set, radio is now DISABLED.
    // We need to enable it again.
    radio_rx_start();
  }
  return 0;
}

// ========== ANALYSIS ==========
void analyze_data(packet_t *pkt) {
  int16_t temp_sub = pkt->temp_substrate / 10;
  int16_t temp_air = pkt->temp_air / 10;
  uint16_t hum = pkt->humidity / 10;

  uint8_t is_day = (pkt->light > 2000);
  uint8_t is_night = (pkt->light < 500);

  LOG("\n=== GECKO MONITOR ===\n");

  LOG("Light: ");
  LOG_UINT(pkt->light);
  LOG("\n");

  LOG("Water: ");
  LOG_UINT(pkt->water);
  LOG("\n");

  LOG("Temp Substrate: ");
  LOG_INT(temp_sub);
  LOG(".");
  LOG_UINT(pkt->temp_substrate % 10);
  LOG(" C\n");

  LOG("Temp Air: ");
  LOG_INT(temp_air);
  LOG(".");
  LOG_UINT(pkt->temp_air % 10);
  LOG(" C\n");

  LOG("Humidity: ");
  LOG_UINT(hum);
  LOG(".");
  LOG_UINT(pkt->humidity % 10);
  LOG(" %\n");

  LOG("Period: ");
  if (is_day)
    LOG("DAY\n");
  else if (is_night)
    LOG("NIGHT\n");
  else
    LOG("TWILIGHT\n");

  // Anomaly detection
  uint8_t critical = 0;
  uint8_t warning = 0;

  if (temp_air < 15 || temp_air > 28)
    critical = 1;
  if (temp_sub < 16 || temp_sub > 28)
    critical = 1;
  if (hum < 40 || hum > 85)
    critical = 1;
  if (pkt->water < 100)
    critical = 1;

  if (!critical) {
    if (is_day && (temp_air < 22 || temp_air > 26))
      warning = 1;
    if (is_night && (temp_air < 18 || temp_air > 22))
      warning = 1;
    if (temp_sub < 20 || temp_sub > 25)
      warning = 1;
    if (is_day && (hum < 50 || hum > 70))
      warning = 1;
    if (is_night && (hum < 65 || hum > 80))
      warning = 1;
    if (pkt->water < 1200)
      warning = 1;
  }

  // Set RGB LED and LED4
  if (critical) {
    rgb_set(1, 0, 0);       // RED
    G_OUTSET = (1 << LED4); // LED4 ON (error)
    LOG("STATUS: CRITICAL\n");
  } else if (warning) {
    rgb_set(1, 1, 0);       // YELLOW
    G_OUTCLR = (1 << LED4); // LED4 OFF
    LOG("STATUS: WARNING\n");
  } else {
    rgb_set(0, 1, 0);       // GREEN
    G_OUTCLR = (1 << LED4); // LED4 OFF
    LOG("STATUS: OK\n");
  }
  LOG("=====================\n");
}

// ========== MAIN ==========
int main(void) {
  gpio_init();
  SEGGER_RTT_Init();
  radio_init();

  LOG("\n\n================================\n");
  LOG("  BARE METAL GECKO RECEIVER\n");
  LOG("  RGB: P0.13/14/15 | LED4: Error\n");
  LOG("  Waiting for sensor data...\n");
  LOG("================================\n\n");

  // Startup RGB test
  rgb_set(1, 0, 0); // RED
  delay_ms(300);
  rgb_set(0, 1, 0); // GREEN
  delay_ms(300);
  rgb_set(0, 0, 1); // BLUE
  delay_ms(300);
  rgb_set(0, 0, 0); // OFF

  // LED4 test
  G_OUTSET = (1 << LED4);
  delay_ms(300);
  G_OUTCLR = (1 << LED4);

  radio_rx_start();

  uint32_t packet_count = 0;
  uint32_t heartbeat_counter = 0;

  // Set Blue ON (Waiting state)
  rgb_set(0, 0, 1);

  while (1) {
    if (radio_rx_check()) {
      packet_count++;

      // Blink Green on receive
      rgb_set(0, 1, 0); // Green
      delay_ms(100);    // Short blink

      LOG("\n[Packet #");
      LOG_UINT(packet_count);
      LOG(" received]\n");

      analyze_data(&rx_packet);

      // Restart RX after processing
      radio_rx_start();

      // Return to Blue (Waiting state)
      rgb_set(0, 0, 1);
    }

    delay_ms(10);

    // Heartbeat every 10 seconds (1000 * 10ms)
    heartbeat_counter++;
    if (heartbeat_counter >= 1000) {
      LOG(".");
      heartbeat_counter = 0;
    }
  }
}
