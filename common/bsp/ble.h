#ifndef __BLE_H
#define __BLE_H

#include "nrf.h"
#include <stdbool.h>
#include <stdint.h>

//=========================== define ==========================================

#define BLE_ADV_INTERVAL_MS    100  // Advertising interval in milliseconds
#define BLE_DEVICE_NAME        "Game_Score"

// BLE Advertising Access Address (fixed for advertising)
#define BLE_ADV_ACCESS_ADDRESS 0x8E89BED6UL

// BLE Advertising channels
#define BLE_ADV_CHANNEL_37     2402  // MHz
#define BLE_ADV_CHANNEL_38     2426  // MHz
#define BLE_ADV_CHANNEL_39     2480  // MHz

// BLE PDU types
#define BLE_PDU_TYPE_ADV_IND   0x00
#define BLE_PDU_TYPE_ADV_DIRECT_IND 0x01
#define BLE_PDU_TYPE_ADV_NONCONN_IND 0x02
#define BLE_PDU_TYPE_SCAN_REQ  0x03
#define BLE_PDU_TYPE_SCAN_RSP  0x04
#define BLE_PDU_TYPE_CONNECT_REQ 0x05
#define BLE_PDU_TYPE_ADV_SCAN_IND 0x06

//=========================== typedef ==========================================

typedef enum {
    BLE_STATE_DISCONNECTED,
    BLE_STATE_CONNECTED
} ble_state_t;

typedef void (*ble_connection_cb_t)(ble_state_t state);

//=========================== prototypes ==========================================

void ble_init(ble_connection_cb_t callback);
void ble_advertising_start(void);
void ble_advertising_stop(void);
bool ble_is_connected(void);
void ble_process(void);

#endif

