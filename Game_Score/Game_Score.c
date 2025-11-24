#include "board.h"
#include "ble.h"
#include <stdio.h>

//=========================== defines ==========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

static void _ble_connection_cb(ble_state_t state);

//=========================== main ============================================

int main(void) {
   
    // bsp
    board_init();
    
    printf("Game_Score starting...\n");
    printf("nRF52840 BLE Device\n");
    
    // Initialize BLE
    ble_init(_ble_connection_cb);
    
    // Start BLE advertising
    ble_advertising_start();
    
    printf("Entering main loop\n");

    // main loop
    while(1) {
        
        // Process BLE events
        ble_process();
        
        // sleep while waiting for an event
        board_sleep();
    }
}

//=========================== private ==========================================

static void _ble_connection_cb(ble_state_t state) {
    if (state == BLE_STATE_CONNECTED) {
        // Device connected
        ble_advertising_stop();
    } else if (state == BLE_STATE_DISCONNECTED) {
        // Device disconnected
        ble_advertising_start();
    }
}
