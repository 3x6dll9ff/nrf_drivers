#include "ble.h"
#include "board.h"
#include <string.h>
#include <stdio.h>

//=========================== variables =======================================

static ble_connection_cb_t connection_callback = NULL;
static ble_state_t ble_state = BLE_STATE_DISCONNECTED;
static bool advertising_active = false;
static uint32_t advertising_counter = 0;
static uint8_t advertising_channel = 0; // 0=37, 1=38, 2=39
static uint32_t advertising_delay_counter = 0;

// BLE Advertising packet structure
// PDU Header: 2 bytes (Type + Length)
// AdvA: 6 bytes (MAC address)
// AdvData: variable length
// Total packet length: 2 (PDU) + 6 (AdvA) + 3 (Flags) + 12 (Name) = 23 bytes
// PDU Length field: 6 (AdvA) + 15 (AdvData) = 21 bytes
// AdvData: 3 (Flags) + 12 (Name) = 15 bytes total
static uint8_t ble_advertising_packet[] = {
    // PDU Header: Type (4 bits) + RFU (1 bit) + TxAdd (1 bit) + RxAdd (1 bit) + Length (6 bits)
    // Type = ADV_IND (0x00). TxAdd=1 (Random Address).
    // Header Byte 0: 0x40 (TxAdd=1, RxAdd=0, Type=0)
    // Header Byte 1: 0x15 (Length=21: 6 AdvA + 15 AdvData)
    0x40, 0x15,  // PDU Header
    
    // AdvA (Advertising Address) - 6 bytes (little endian, random address)
    // Use a new address to force cache refresh on phone: C0:00:00:00:00:01
    0x01, 0x00, 0x00, 0x00, 0x00, 0xC0,
    
    // AdvData: 15 bytes
    // Flags (3 bytes)
    0x02, 0x01, 0x06,  // Length=2, Type=Flags (0x01), Value=0x06 (LE General Discoverable + BR/EDR not supported)
    
    // Complete Local Name (12 bytes: Length(1) + Type(1) + "Game_Score"(10))
    0x0A, 0x09,  // Length=10 (0x0A), Type=Complete Local Name (0x09)
    'G', 'a', 'm', 'e', '_', 'S', 'c', 'o', 'r', 'e'
};

// Channel frequencies (MHz)
static const uint32_t ble_adv_channels[] = {
    BLE_ADV_CHANNEL_37,
    BLE_ADV_CHANNEL_38,
    BLE_ADV_CHANNEL_39
};

//=========================== prototypes ======================================

static void _ble_set_state(ble_state_t new_state);
static void _ble_setup_radio(void);
static void _ble_setup_advertising_channel(uint8_t channel_idx);

//=========================== public ==========================================

void ble_init(ble_connection_cb_t callback) {
    connection_callback = callback;
    ble_state = BLE_STATE_DISCONNECTED;
    advertising_active = false;
    
    _ble_setup_radio();
}

void ble_advertising_start(void) {
    if (advertising_active) {
        return;
    }
    
    advertising_active = true;
    advertising_counter = 0;
    advertising_channel = 0;
    advertising_delay_counter = 0;
    
    // Print BLE address
    // Address is at bytes [2:7] in packet (after PDU header bytes 0-1)
    printf("BLE Advertising started\n");
    printf("Device Address: %02lX:%02lX:%02lX:%02lX:%02lX:%02lX\n",
           (unsigned long)ble_advertising_packet[7], (unsigned long)ble_advertising_packet[6],
           (unsigned long)ble_advertising_packet[5], (unsigned long)ble_advertising_packet[4],
           (unsigned long)ble_advertising_packet[3], (unsigned long)ble_advertising_packet[2]);
    printf("Device Name: Game_Score\n");
    
    // Start advertising on channel 37
    _ble_setup_advertising_channel(0);
    
    // Clear events
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    
    // Enable radio in TX mode
    NRF_RADIO->TASKS_TXEN = 1;
    
    // Wait for radio to be ready and start transmission
    while (NRF_RADIO->EVENTS_READY == 0);
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_START = 1;
    
    printf("Radio TX started on channel 37\n");
}

void ble_advertising_stop(void) {
    advertising_active = false;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
}

bool ble_is_connected(void) {
    return (ble_state == BLE_STATE_CONNECTED);
}

void ble_process(void) {
    // BLE advertising is handled by interrupt handler
    // This function is kept for future use
    (void)advertising_active;
    (void)advertising_delay_counter;
}

//=========================== private ==========================================

static void _ble_set_state(ble_state_t new_state) {
    if (ble_state != new_state) {
        ble_state = new_state;
        if (connection_callback) {
            connection_callback(ble_state);
        }
    }
}

static void _ble_setup_radio(void) {
    // Disable radio first
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;
    
    // Configure for BLE 1Mbit mode
    NRF_RADIO->MODE = 3UL;  // RADIO_MODE_MODE_Ble_1Mbit
    
    // Packet configuration PCNF0
    // For BLE: S0LEN=0 (no S0 field), LFLEN=8 (8 bits for Length field), S1LEN=0
    // BLE doesn't use S0 field - PDU Type is in the packet data itself
    NRF_RADIO->PCNF0 = (8UL << 0) |   // LFLEN: 8 bits (1 byte)
                       (0UL << 8) |   // S0LEN: 0 bytes (no S0 field for BLE)
                       (0UL << 16);   // S1LEN: 0 bits
    
    // Packet configuration PCNF1
    // MAXLEN=32 (plenty for 21), STATLEN=0, BALEN=3 (3 bytes Base Address), ENDIAN=Little, WHITEEN=Enabled
    NRF_RADIO->PCNF1 = (32UL << 0) |  // MAXLEN: max packet length
                       (0UL << 8) |   // STATLEN: 0
                       (3UL << 16) |  // BALEN: 3 bytes
                       (0UL << 24) |  // ENDIAN: Little (0)
                       (1UL << 25);   // WHITEEN: Enabled
    
    // Set access address (BASE0 + PREFIX0)
    // BLE Access Address: 0x8E89BED6
    // BASE0 register: bits [31:8] = base address (3 bytes), bits [7:0] = reserved
    // For 0x8E89BED6: base = 0x89BED6 (bits 23:0), prefix = 0x8E
    // BASE0 format: [31:8] = base address, so shift left by 8
    NRF_RADIO->BASE0 = (0x89BED6UL << 8);
    NRF_RADIO->PREFIX0 = 0x8E;
    
    // Verify access address setup
    printf("Access Address: BASE0=0x%08lX, PREFIX0=0x%02lX\n", (unsigned long)NRF_RADIO->BASE0, (unsigned long)NRF_RADIO->PREFIX0);
    
    // Set packet pointer
    NRF_RADIO->PACKETPTR = (uint32_t)ble_advertising_packet;
    
    // CRC configuration
    // LEN=Three (3 bytes), SKIPADDR=Skip (skip Access Address)
    NRF_RADIO->CRCCNF = (3UL << 0) |  // LEN: Three (3 bytes)
                        (1UL << 8);   // SKIPADDR: Skip
    
    NRF_RADIO->CRCINIT = 0x555555;  // BLE CRC initial value
    NRF_RADIO->CRCPOLY = 0x00065B;  // BLE CRC polynomial
    
    // TX power: 0 dBm
    NRF_RADIO->TXPOWER = 0x0UL;
    
    // Shortcuts: READY -> START, END -> DISABLE
    NRF_RADIO->SHORTS = (1UL << 0) |   // READY_START: Enabled
                        (1UL << 1);    // END_DISABLE: Enabled
    
    // Enable interrupts
    NRF_RADIO->INTENSET = (1UL << 0) |   // READY: Enabled
                          (1UL << 3) |   // END: Enabled
                          (1UL << 4);    // DISABLED: Enabled
    
    // Enable RADIO interrupt in NVIC
    NVIC_EnableIRQ(RADIO_IRQn);
}

static void _ble_setup_advertising_channel(uint8_t channel_idx) {
    if (channel_idx >= 3) {
        channel_idx = 0;
    }
    
    uint32_t frequency = ble_adv_channels[channel_idx];
    
    // Set frequency (2400 MHz + channel frequency offset)
    // FREQUENCY register: frequency - 2400 MHz
    // Channel 37 (2402): 2
    // Channel 38 (2426): 26
    // Channel 39 (2480): 80
    NRF_RADIO->FREQUENCY = frequency - 2400;
    
    // Set whitening initial value based on channel
    // For BLE advertising: DATAWHITEIV = channel index | 0x40
    // Channel 37 (index 37) -> 37 | 0x40 = 0x65
    // Channel 38 (index 38) -> 38 | 0x40 = 0x66
    // Channel 39 (index 39) -> 39 | 0x40 = 0x67
    uint8_t ble_channel_num = 37 + channel_idx;
    NRF_RADIO->DATAWHITEIV = ble_channel_num | 0x40;
}

//=========================== interrupt handlers ==============================

void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_READY) {
        NRF_RADIO->EVENTS_READY = 0;
        // Start transmission (shortcut will handle this, but we clear event)
    }
    
    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;
        // Packet sent, will be disabled by shortcut
        advertising_counter++;
        if (advertising_counter % 100 == 0) {
            printf("Packets sent: %lu\n", advertising_counter);
        }
    }
    
    if (NRF_RADIO->EVENTS_DISABLED) {
        NRF_RADIO->EVENTS_DISABLED = 0;
        
        if (advertising_active) {
            // Switch to next advertising channel
            advertising_channel = (advertising_channel + 1) % 3;
            uint8_t channel_num = 37 + advertising_channel;
            _ble_setup_advertising_channel(advertising_channel);
            
            if (advertising_counter % 50 == 0) {
                printf("Switching to channel %d (freq: %d MHz)\n", channel_num, ble_adv_channels[advertising_channel]);
            }
            
            // Add delay between packets
            // Approx 20ms delay
            for (volatile int i = 0; i < 300000; i++);
            
            // Re-enable TX for next packet
            NRF_RADIO->TASKS_TXEN = 1;
        }
    }
}

