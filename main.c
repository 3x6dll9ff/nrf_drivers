/**
 * Copyright (c) 2015 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_error.h"
#include "nrf_strerror.h"

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#endif


#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

// RGB LED пины
#define RGB_RED_PIN     13
#define RGB_GREEN_PIN   14
#define RGB_BLUE_PIN    15

#define DEVICE_NAME                     "Game_Score"                            /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

// Простой 16-битный кастомный сервис (если понадобится GATT в будущем)
#define SENSOR_SERVICE_UUID             0xFFF0
#define SENSOR_CHAR_UUID                0xFFF1
#define SENSOR_PAYLOAD_LEN              10u
#define SENSOR_PAYLOAD_MAX_LEN          20u
#define BLE_CONN_MONITOR_DISABLED       1

// Производитель и сканер для приема данных с ESP32 по рекламе
#define ESP_MANUF_ID                    0x1234
#define SCAN_INTERVAL                   0x00A0  /**< 100 ms. */
#define SCAN_WINDOW                     0x0050  /**< 50 ms. */
#define SCAN_TIMEOUT                    0       /**< No timeout. */

typedef struct __attribute__((packed))
{
    uint16_t light_raw;
    uint16_t water_raw;
    int16_t  temp_inside_cx100;
    int16_t  temp_outside_cx100;
    uint16_t hum_outside_pctx100;
} sensor_payload_t;

typedef struct
{
    uint16_t                  service_handle;
    ble_gatts_char_handles_t  sensor_char_handles;
    uint8_t                   uuid_type;
} ble_sensor_service_t;

static ble_sensor_service_t   m_sensor_service;

BLE_LBS_DEF(m_lbs);                                                             /**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */
static uint32_t m_conn_no_activity_counter = 0;                                  /**< Counter for connection inactivity check. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

// Буфер для сканирования рекламных пакетов ESP32
static ble_gap_scan_params_t m_scan_params =
{
    .active        = 0x00,                       // пассивное сканирование
    .interval      = SCAN_INTERVAL,
    .window        = SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .timeout       = SCAN_TIMEOUT,
    .scan_phys     = BLE_GAP_PHY_1MBPS
#if defined(S140) || defined(S132) || defined(S112)
    ,
    .extended      = 0
#endif
};

static uint8_t m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_MIN];
static ble_data_t m_scan_buffer =
{
    .p_data = m_scan_buffer_data,
    .len    = sizeof(m_scan_buffer_data)
};

static void sensor_service_init(void);
static void sensor_payload_process(sensor_payload_t const * p_payload);
static void scan_start(void);
static void rgb_init(void);
static void rgb_set_color(bool red, bool green, bool blue);

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

/**@brief Custom error fault handler without breakpoint.
 */
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    __disable_irq();
    NRF_LOG_FINAL_FLUSH();

#ifndef DEBUG
    NRF_LOG_ERROR("Fatal error");
#else
    switch (id)
    {
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
        case NRF_FAULT_ID_SD_ASSERT:
            NRF_LOG_ERROR("SOFTDEVICE: ASSERTION FAILED");
            break;
        case NRF_FAULT_ID_APP_MEMACC:
            NRF_LOG_ERROR("SOFTDEVICE: INVALID MEMORY ACCESS");
            break;
#endif
        case NRF_FAULT_ID_SDK_ASSERT:
        {
            assert_info_t * p_info = (assert_info_t *)info;
            NRF_LOG_ERROR("ASSERTION FAILED at %s:%u",
                          p_info->p_file_name,
                          p_info->line_num);
            break;
        }
        case NRF_FAULT_ID_SDK_ERROR:
        {
            error_info_t * p_info = (error_info_t *)info;
            NRF_LOG_ERROR("ERROR %u [%s] at %s:%u\r\nPC at: 0x%08x",
                          p_info->err_code,
                          nrf_strerror_get(p_info->err_code),
                          p_info->p_file_name,
                          p_info->line_num,
                          pc);
            NRF_LOG_ERROR("End of error report");
            break;
        }
        default:
            NRF_LOG_ERROR("UNKNOWN FAULT at 0x%08X", pc);
            break;
    }
#endif

    // No breakpoint - just log and continue
    // NRF_BREAKPOINT_COND; // Removed to prevent debugger breakpoint

#ifndef DEBUG
    NRF_LOG_WARNING("System reset");
    NVIC_SystemReset();
#else
    app_error_save_and_stop(id, pc, info);
#endif
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}

/**@brief Инициализация RGB светодиода. */
static void rgb_init(void)
{
    nrf_gpio_cfg_output(RGB_RED_PIN);
    nrf_gpio_cfg_output(RGB_GREEN_PIN);
    nrf_gpio_cfg_output(RGB_BLUE_PIN);
    
    // Выключаем все цвета (active high: 0 = выключен)
    nrf_gpio_pin_clear(RGB_RED_PIN);
    nrf_gpio_pin_clear(RGB_GREEN_PIN);
    nrf_gpio_pin_clear(RGB_BLUE_PIN);
}

/**@brief Установка цвета RGB светодиода.
 * @param red   true = красный включён
 * @param green true = зелёный включён
 * @param blue  true = синий включён
 */
static void rgb_set_color(bool red, bool green, bool blue)
{
    // HW-478: общий катод, active high (1 = включён, 0 = выключен)
    nrf_gpio_pin_write(RGB_RED_PIN, red ? 1 : 0);
    nrf_gpio_pin_write(RGB_GREEN_PIN, green ? 1 : 0);
    nrf_gpio_pin_write(RGB_BLUE_PIN, blue ? 1 : 0);
}

// Define timer using APP_TIMER_DEF macro for app_timer v2
APP_TIMER_DEF(m_conn_check_timer_id);
static uint32_t m_conn_start_tick = 0;

/**@brief Timer handler for connection status check.
 */
static void conn_check_timer_handler(void * p_context)
{
#if BLE_CONN_MONITOR_DISABLED
    UNUSED_PARAMETER(p_context);
    return;
#endif
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        m_conn_no_activity_counter++;
        
        // Log every 2 seconds (4 checks)
        if (m_conn_no_activity_counter % 4 == 0)
        {
            NRF_LOG_INFO("Connection check: counter=%d (handle=%d)", 
                         m_conn_no_activity_counter, m_conn_handle);
            NRF_LOG_FLUSH();
        }
        
        // If no activity for 5 seconds (10 checks at 500ms each), force disconnect
        if (m_conn_no_activity_counter >= 10)
        {
            NRF_LOG_WARNING("=== No connection activity for 5 seconds, forcing disconnect ===");
            NRF_LOG_FLUSH();
            ret_code_t err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
            {
                NRF_LOG_ERROR("Failed to disconnect: 0x%08X", err_code);
                NRF_LOG_FLUSH();
            }
            else
            {
                NRF_LOG_INFO("Disconnect command sent, waiting for DISCONNECTED event");
                NRF_LOG_FLUSH();
            }
            // Don't reset handle here - let DISCONNECTED event handle it
            m_conn_no_activity_counter = 0;
        }
    }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module, making it use the scheduler
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timer for connection status check
    err_code = app_timer_create(&m_conn_check_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                conn_check_timer_handler);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Connection check timer created successfully");
    NRF_LOG_FLUSH();
}

/**@brief Запуск пассивного сканирования рекламных пакетов (ESP32). */
static void scan_start(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Starting BLE scan for ESP32 advertising...");
    NRF_LOG_FLUSH();

    err_code = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("scan_start error: 0x%08X", err_code);
        NRF_LOG_FLUSH();
    }
    else
    {
        NRF_LOG_INFO("BLE scan started successfully");
        NRF_LOG_FLUSH();
    }
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] =
    {
        {SENSOR_SERVICE_UUID, BLE_UUID_TYPE_BLE}
    };

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
    if (led_state)
    {
        bsp_board_led_on(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED ON!");
    }
    else
    {
        bsp_board_led_off(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED OFF!");
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_lbs_init_t     init     = {0};
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize LBS.
    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);

    sensor_service_init();
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void sensor_service_init(void)
{
    ret_code_t          err_code;
    ble_uuid_t          service_uuid;
    ble_uuid_t          char_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    uint8_t             initial_value[SENSOR_PAYLOAD_LEN] = {0};

    // 16-битный кастомный сервис 0xFFF0
    service_uuid.type = BLE_UUID_TYPE_BLE;
    service_uuid.uuid = SENSOR_SERVICE_UUID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &m_sensor_service.service_handle);
    APP_ERROR_CHECK(err_code);

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;

    memset(&attr_md, 0, sizeof(attr_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.vlen = 1;

    char_uuid.type = BLE_UUID_TYPE_BLE;
    char_uuid.uuid = SENSOR_CHAR_UUID;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(initial_value);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = SENSOR_PAYLOAD_MAX_LEN;
    attr_char_value.p_value   = initial_value;

    err_code = sd_ble_gatts_characteristic_add(m_sensor_service.service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &m_sensor_service.sensor_char_handles);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Sensor service ready (service_handle=%d, char_handle=%d)",
                 m_sensor_service.service_handle,
                 m_sensor_service.sensor_char_handles.value_handle);
    NRF_LOG_FLUSH();
}

static void sensor_payload_process(sensor_payload_t const * p_payload)
{
    if (p_payload == NULL)
    {
        return;
    }

    // Дедупликация: выводим только если данные изменились
    static sensor_payload_t last_payload = {0};
    if (memcmp(&last_payload, p_payload, sizeof(sensor_payload_t)) == 0)
    {
        return; // Данные не изменились, не логируем
    }
    memcpy(&last_payload, p_payload, sizeof(sensor_payload_t));

    // Преобразуем в читаемый формат (деление на 100 для температур и влажности)
    int16_t ti_int = p_payload->temp_inside_cx100 / 100;
    uint8_t ti_frac = abs(p_payload->temp_inside_cx100 % 100);
    
    int16_t to_int = p_payload->temp_outside_cx100 / 100;
    uint8_t to_frac = abs(p_payload->temp_outside_cx100 % 100);
    
    uint16_t h_int = p_payload->hum_outside_pctx100 / 100;
    uint8_t h_frac = p_payload->hum_outside_pctx100 % 100;

    // Определяем день/ночь по освещённости
    bool is_day = (p_payload->light_raw > 2000);
    bool is_night = (p_payload->light_raw < 500);
    const char* period = is_day ? "DAY" : (is_night ? "NIGHT" : "TWILIGHT");

    NRF_LOG_INFO("=== Crested Gecko Monitor [%s] ===", period);
    NRF_LOG_INFO("Light: %u", p_payload->light_raw);
    NRF_LOG_INFO("Water: %u", p_payload->water_raw);
    NRF_LOG_INFO("Temp Substrate: %d.%02u C", ti_int, ti_frac);
    NRF_LOG_INFO("Temp Air: %d.%02u C", to_int, to_frac);
    NRF_LOG_INFO("Humidity: %u.%02u %%", h_int, h_frac);

    // Анализ данных и установка цвета RGB для Crested Gecko
    bool critical = false;
    bool warning = false;

    // Критические условия (красный):
    // 1. Температура воздуха (outside) критична
    if (to_int < 15 || to_int > 28)
    {
        critical = true;
    }
    
    // 2. Температура субстрата (inside) критична
    if (ti_int < 16 || ti_int > 28)
    {
        critical = true;
    }
    
    // 3. Влажность критична
    if (h_int < 40 || h_int > 85)
    {
        critical = true;
    }
    
    // 4. Очень сухо (нужно срочно прскать)
    if (p_payload->water_raw < 100)  // Примерно < 30% от макс значения ~4095
    {
        critical = true;
    }

    // Предупреждение (жёлтый):
    if (!critical)
    {
        // День: температура воздуха вне оптимума 22-26°C
        if (is_day && (to_int < 22 || to_int > 26))
        {
            warning = true;
        }
        
        // Ночь: температура воздуха вне оптимума 18-22°C
        if (is_night && (to_int < 18 || to_int > 22))
        {
            warning = true;
        }
        
        // Температура субстрата вне идеала 20-25°C
        if (ti_int < 20 || ti_int > 25)
        {
            warning = true;
        }
        
        // День: влажность вне оптимума 50-70%
        if (is_day && (h_int < 50 || h_int > 70))
        {
            warning = true;
        }
        
        // Ночь: влажность вне оптимума 65-80%
        if (is_night && (h_int < 65 || h_int > 80))
        {
            warning = true;
        }
        
        // Сухо, нужно прскать (water < 30%)
        if (p_payload->water_raw < 1200)  // Примерно < 30% от макс ~4095
        {
            warning = true;
        }
    }

    // Установка цвета
    if (critical)
    {
        rgb_set_color(true, false, false);  // Красный
        NRF_LOG_WARNING("Status: CRITICAL");
    }
    else if (warning)
    {
        rgb_set_color(true, true, false);   // Жёлтый (красный + зелёный)
        NRF_LOG_WARNING("Status: WARNING");
    }
    else
    {
        rgb_set_color(false, true, false);  // Зелёный
        NRF_LOG_INFO("Status: OK");
    }
}


/**@brief Function for printing BLE address.
 */
static void ble_address_print(void)
{
    ret_code_t err_code;
    ble_gap_addr_t addr;

    NRF_LOG_INFO("Getting BLE address...");
    NRF_LOG_FLUSH();
    NRF_LOG_PROCESS();

    // Try to get identity address first
    err_code = sd_ble_gap_addr_get(&addr);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_INFO("BLE Identity Address: %02X:%02X:%02X",
                     addr.addr[5], addr.addr[4], addr.addr[3]);
        NRF_LOG_INFO("                      %02X:%02X:%02X",
                     addr.addr[2], addr.addr[1], addr.addr[0]);
        NRF_LOG_INFO("Address type: %d", addr.addr_type);
        NRF_LOG_FLUSH();
        NRF_LOG_PROCESS();
    }
    else
    {
        NRF_LOG_WARNING("Identity address not available (err: 0x%08X)", err_code);
        NRF_LOG_FLUSH();
        NRF_LOG_PROCESS();
        
        // Try to get advertising address as fallback
        err_code = sd_ble_gap_adv_addr_get(m_adv_handle, &addr);
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_INFO("BLE Advertising Address: %02X:%02X:%02X",
                         addr.addr[5], addr.addr[4], addr.addr[3]);
            NRF_LOG_INFO("                        %02X:%02X:%02X",
                         addr.addr[2], addr.addr[1], addr.addr[0]);
            NRF_LOG_INFO("Address type: %d", addr.addr_type);
            NRF_LOG_FLUSH();
            NRF_LOG_PROCESS();
        }
        else
        {
            NRF_LOG_WARNING("Advertising address not available (err: 0x%08X)", err_code);
            NRF_LOG_FLUSH();
            NRF_LOG_PROCESS();
        }
    }
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;
    uint32_t             adv_interval_ms;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    adv_interval_ms = (APP_ADV_INTERVAL * 625) / 1000; // Convert to milliseconds

    // Print BLE address after stack is fully initialized
    ble_address_print();

    NRF_LOG_INFO("BLE Advertising started");
    NRF_LOG_INFO("Advertising interval: %d ms (slots: %d)", adv_interval_ms, APP_ADV_INTERVAL);
    NRF_LOG_INFO("Advertising channels: 37 (2402 MHz), 38 (2426 MHz), 39 (2480 MHz)");
    NRF_LOG_INFO("Sending advertising packets...");
    NRF_LOG_FLUSH();

    bsp_board_led_on(ADVERTISING_LED);

    // Параллельно запускаем сканирование для приема данных от ESP32
    scan_start();
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // Reset activity counter on any BLE event for our connection
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID && 
        p_ble_evt->evt.gap_evt.conn_handle == m_conn_handle)
    {
        if (m_conn_no_activity_counter > 0)
        {
            NRF_LOG_DEBUG("BLE event received, resetting activity counter (was %d)", m_conn_no_activity_counter);
        }
        m_conn_no_activity_counter = 0;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            ble_gap_evt_connected_t const * p_connected_evt = &p_ble_evt->evt.gap_evt.params.connected;
            NRF_LOG_INFO("=== Device connected ===");
            NRF_LOG_INFO("Peer address: %02X:%02X:%02X:%02X:%02X:%02X",
                         p_connected_evt->peer_addr.addr[5],
                         p_connected_evt->peer_addr.addr[4],
                         p_connected_evt->peer_addr.addr[3],
                         p_connected_evt->peer_addr.addr[2],
                         p_connected_evt->peer_addr.addr[1],
                         p_connected_evt->peer_addr.addr[0]);
            NRF_LOG_INFO("Connection handle: %d", p_ble_evt->evt.gap_evt.conn_handle);
            NRF_LOG_FLUSH();
            bsp_board_led_on(CONNECTED_LED);
            bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            // Reset activity counter and start connection check timer (check every 500ms)
            m_conn_no_activity_counter = 0;
            NRF_LOG_INFO("Starting connection activity monitor (500ms interval)");
            NRF_LOG_FLUSH();
            err_code = app_timer_start(m_conn_check_timer_id, APP_TIMER_TICKS(500), NULL);
            APP_ERROR_CHECK(err_code);
            m_conn_start_tick = app_timer_cnt_get();
            break;
        }

        case BLE_GAP_EVT_DISCONNECTED:
        {
            ble_gap_evt_disconnected_t const * p_disconnected_evt = &p_ble_evt->evt.gap_evt.params.disconnected;
            NRF_LOG_INFO("=== Device disconnected ===");
            NRF_LOG_INFO("Reason: 0x%02X (%s)", 
                         p_disconnected_evt->reason,
                         (p_disconnected_evt->reason == BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION) ? "Remote terminated" :
                         (p_disconnected_evt->reason == BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION) ? "Local terminated" :
                         "Other");
            NRF_LOG_FLUSH();
            bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            m_conn_no_activity_counter = 0;
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            // Stop connection check timer
            err_code = app_timer_stop(m_conn_check_timer_id);
            APP_ERROR_CHECK(err_code);
            uint32_t end_tick = app_timer_cnt_get();
            uint32_t ticks = app_timer_cnt_diff_compute(end_tick, m_conn_start_tick);
            uint32_t duration_ms = ROUNDED_DIV((uint64_t)ticks * 1000, APP_TIMER_CLOCK_FREQ);
            NRF_LOG_INFO("Session duration: %lu ms", duration_ms);
            advertising_start();
            break;
        }

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_WARNING("Connection timeout detected, forcing disconnect");
                NRF_LOG_FLUSH();
                // Force disconnect on timeout
                if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
                {
                    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_WRITE:
        {
            ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
            if (p_evt_write->handle == m_sensor_service.sensor_char_handles.value_handle)
            {
                if (p_evt_write->len == sizeof(sensor_payload_t))
                {
                    sensor_payload_t payload;
                    memcpy(&payload, p_evt_write->data, sizeof(sensor_payload_t));
                    sensor_payload_process(&payload);
                }
                else
                {
                    NRF_LOG_WARNING("Sensor payload size mismatch: %u (expected %u)",
                                    p_evt_write->len,
                                    sizeof(sensor_payload_t));
                    NRF_LOG_FLUSH();
                }
            }
            else
            {
                NRF_LOG_DEBUG("Write to handle 0x%04X len=%u", p_evt_write->handle, p_evt_write->len);
            }
        } break;

        case BLE_GAP_EVT_ADV_REPORT:
        {
            ble_gap_evt_adv_report_t const * p_adv = &p_ble_evt->evt.gap_evt.params.adv_report;
            uint16_t len   = p_adv->data.len;
            uint8_t const *data = p_adv->data.p_data;

            // Фильтр: игнорируем слишком короткие пакеты (не содержат manufacturer data)
            if (len < 10 || data == NULL)
            {
                // Продолжаем сканирование без логирования
                ret_code_t err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
                (void)err_code;
                break;
            }

            uint16_t index = 0;
            while (index < len)
            {
                uint8_t field_len = data[index];
                if (field_len == 0)
                {
                    break;
                }
                if (index + field_len >= len + 1)
                {
                    break;
                }

                uint8_t ad_type = data[index + 1];

                if (ad_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA)
                {
                    // [len][type][company_id_lo][company_id_hi][ascii_payload...]
                    uint8_t payload_len = field_len - 1;
                    if (payload_len > 2)
                    {
                        uint8_t const *mdata = &data[index + 2];
                        uint16_t company_id = mdata[0] | (mdata[1] << 8);

                        if (company_id == ESP_MANUF_ID)
                        {
                            uint8_t txt_len = payload_len - 2;
                            
                            if (txt_len >= 5 && txt_len < 40)
                            {
                                char buf[40] = {0};
                                memcpy(buf, &mdata[2], txt_len);
                                buf[txt_len] = '\0';

                                sensor_payload_t payload;
                                memset(&payload, 0, sizeof(payload));

                                int ti10 = 0, to10 = 0;
                                int hum10 = 0;
                                unsigned int light = 0, water = 0;

                                int parsed = sscanf(buf, "%u,%u,%d,%d,%d",
                                                    &light, &water, &ti10, &to10, &hum10);
                                
                                if (parsed == 5)
                                {
                                    payload.light_raw            = (uint16_t)light;
                                    payload.water_raw            = (uint16_t)water;
                                    payload.temp_inside_cx100    = (int16_t)(ti10 * 10);
                                    payload.temp_outside_cx100   = (int16_t)(to10 * 10);
                                    payload.hum_outside_pctx100  = (uint16_t)(hum10 * 10);
                                    sensor_payload_process(&payload);
                                }
                            }
                        }
                    }
                }

                index += field_len + 1;
            }

            // Продолжаем сканирование (SoftDevice ставит его на паузу)
            if (p_adv->type.status != BLE_GAP_ADV_DATA_STATUS_INCOMPLETE_MORE_DATA)
            {
                err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
                if (err_code != NRF_SUCCESS &&
                    err_code != NRF_ERROR_INVALID_STATE)
                {
                    NRF_LOG_ERROR("scan_continue error: 0x%08X", err_code);
                    NRF_LOG_FLUSH();
                }
            }
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON:
            NRF_LOG_INFO("Send button state change.");
            err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for application main entry.
 */
int main(void)

{
    // Initialize.
    log_init();
    leds_init();
    rgb_init();
    timers_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    NRF_LOG_INFO("Game_Score BLE device started.");
    
    // Тест RGB: красный -> зелёный -> синий -> выключен
    NRF_LOG_INFO("RGB test: RED");
    rgb_set_color(true, false, false);
    nrf_delay_ms(500);
    
    NRF_LOG_INFO("RGB test: GREEN");
    rgb_set_color(false, true, false);
    nrf_delay_ms(500);
    
    NRF_LOG_INFO("RGB test: BLUE");
    rgb_set_color(false, false, true);
    nrf_delay_ms(500);
    
    NRF_LOG_INFO("RGB test: OFF");
    rgb_set_color(false, false, false);
    nrf_delay_ms(500);
    
    // Синий цвет = система запущена и готова
    rgb_set_color(false, false, true);
    
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
