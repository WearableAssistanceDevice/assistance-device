/**
 * @brief BLE Assistance Request Service central and client application main file.
 *
 * This file contains the source code for a sample client application using the Assistance Request service.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"

#include "board_service/board_service.h"
#include "ble_service/ble_services.h"
#include "ble_service/ble_ars/ble_ars.h"
#include "ble_service/ble_dls_c/ble_dls_c.h"

#include "bsp.h"

#include "ble.h"
#include "ble_advertising.h"  
#include "ble_db_discovery.h"

#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_gq.h"
#include "nrf_ble_qwr.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_pwr_mgmt.h"


NRF_BLE_SCAN_DEF(m_scan);                         /**< Scanning module instance */
NRF_BLE_GATT_DEF(m_gatt);                         /**< GATT module instance */
BLE_DB_DISCOVERY_DEF(m_db_disc);                  /**< DB discovery module instance */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                  /**< BLE GATT Queue instance */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

BLE_ARS_DEF(m_ble_ars);      /**< Assistance Request server module */
BLE_DLS_C_DEF(m_ble_dls_c);  /**< Door Lock client module */


static char const m_server_periph_name[] = SERVER_PERIPH_NAME;
static char const m_door_periph_name[] = DOOR_PERIPH_NAME;


/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t* p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling the Door Lock client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void dls_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Set up the filter to scan for the assistance server
 */
static void server_scan_filters_enable(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_server_periph_name);
    APP_ERROR_CHECK(err_code);
}


/**@brief Set up the filter to scan for a door lock
 */
static void door_scan_filters_enable(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_door_periph_name);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_off(CENTRAL_CONNECTED_LED);
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}


/**@brief Function to start scanning.
 */
static void scan_stop(void)
{
    nrf_ble_scan_stop();
    bsp_board_led_off(CENTRAL_SCANNING_LED);
}


/**@brief Handles events coming from the Assistance Request central module.
 *
 * @param[in] p_ars      The ARS module instance
 * @param[in] p_ars_evt  The ARS event structure
 */
static void ars_evt_handler(ble_ars_t* p_ars, ble_ars_evt_t* p_ars_evt)
{
    uint32_t err_code;
    uint8_t assist_requested;

    switch(p_ars_evt->evt_type) {
        case BLE_ARS_EVT_NOTIFICATION_ENABLED:
            break;

        case BLE_ARS_EVT_NOTIFICATION_DISABLED:
            break;

        case BLE_ARS_EVT_CONNECTED:
            break;

        case BLE_ARS_EVT_DISCONNECTED:
            break;

        case BLE_ARS_EVT_WRITE: {
            err_code = ble_ars_assist_req_get(p_ars, &assist_requested);
            APP_ERROR_CHECK(err_code);
            if (assist_requested) {
                bsp_board_led_on(ASSISTANCE_REQUEST_LED);
            }
            else {
                bsp_board_led_off(ASSISTANCE_REQUEST_LED);
                sd_ble_gap_disconnect(p_ars->conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            }
            break;
        }

        default: break;
    }
}


/**@brief Handles events coming from the Door Lock client module.
 */
static void dls_c_evt_handler(ble_dls_c_t* p_dls_c, ble_dls_c_evt_t* p_dls_c_evt)
{
    ret_code_t err_code;

    switch (p_dls_c_evt->evt_type)
    {
        case BLE_DLS_C_EVT_DISCOVERY_COMPLETE:
        {
            // Assign handles
            err_code = ble_dls_c_handles_assign(&m_ble_dls_c,
                                                p_dls_c_evt->conn_handle,
                                                &p_dls_c_evt->params.peer_db);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Door Lock service discovered on conn_handle 0x%x.", p_dls_c_evt->conn_handle);

            // Door Lock service discovered. Enable notification of door lock state.
            //err_code = ble_dls_c_lock_req_notif_enable(p_dls_c);
            //APP_ERROR_CHECK(err_code);

            // Send unlock request
            err_code = ble_dls_c_lock_state_send(&m_ble_dls_c, 0);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Door unlock request sent");
        } break; // BLE_DLS_C_EVT_DISCOVERY_COMPLETE

        case BLE_DLS_C_EVT_LOCK_NOTIFICATION:
        {
            NRF_LOG_INFO("Door lock state changed on peer to 0x%x.", p_dls_c_evt->params.lock_state.state);
        } break; // BLE_DLS_C_EVT_LOCK_NOTIFICATION

        default: break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(const ble_evt_t* p_ble_evt, void* p_context)
{
    const ble_gap_evt_t* p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED: {
            bsp_board_led_off(CENTRAL_SCANNING_LED);
            bsp_board_led_on(CENTRAL_CONNECTED_LED);
        } break;

        case BLE_GAP_EVT_DISCONNECTED: {
            bsp_board_led_off(CENTRAL_CONNECTED_LED);
        } break;

        default: break;
    }
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] event  BSP event
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event) {
        case ASSIST_REQ_KEY_EVENT: {
            err_code = ble_ars_assist_req_set(&m_ble_ars, true);
            APP_ERROR_CHECK(err_code);

            bsp_board_led_on(ASSISTANCE_REQUEST_LED);

            nrf_ble_scan_stop();
            server_scan_filters_enable();
            scan_start();

            NRF_LOG_INFO("ARS initiate assistance request");
        }
        break;

        case DOOR_UNLOCK_KEY_EVENT: {
            nrf_ble_scan_stop();
            door_scan_filters_enable();
            scan_start();

            NRF_LOG_INFO("DLS_C initiate door unlock request");
        }
        break;

        default: break;
    }
}


/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(const scan_evt_t* p_scan_evt)
{
    return;
}


/**@brief Function for handling database discovery events.
 *
 * @param[in] p_evt  The database discover event
 */
static void db_disc_handler(ble_db_discovery_evt_t* p_evt)
{
    return;
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();
}


/**@brief Assistance Request server initialization.
 */
static void ars_init(void)
{
    ble_ars_init_t assist_init = {0};
    
    assist_init.evt_handler = ars_evt_handler;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&assist_init.assist_req_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&assist_init.assist_req_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&assist_init.assist_req_char_attr_md.write_perm);

    const ret_code_t err_code = ble_ars_init(&m_ble_ars, &assist_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Door Lock client initialization.
 */
static void dls_c_init(nrf_ble_gq_t* p_gatt_queue)
{
    ret_code_t       err_code;
    ble_dls_c_init_t dls_c_init_obj;

    dls_c_init_obj.evt_handler   = dls_c_evt_handler;
    dls_c_init_obj.p_gatt_queue  = p_gatt_queue;
    dls_c_init_obj.error_handler = dls_error_handler;

    err_code = ble_dls_c_init(&m_ble_dls_c, &dls_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    // Board services configuration
    board_services_init_t board_init = {0};
    board_init.bsp_evt_handler = bsp_event_handler;

    // BLE services configuration
    ble_services_init_t ble_init = {0};

    ble_gatts_service_init_func_t gatts_init_funcs[] = {
        ars_init
    };
    ble_gattc_service_init_func_t gattc_init_funcs[] = {
        dls_c_init
    };

    ble_init.p_ble_db_discovery  = &m_db_disc;
    ble_init.p_ble_gatt          = &m_gatt;
    ble_init.p_ble_qatt_queue    = &m_ble_gatt_queue;
    ble_init.p_ble_scan          = &m_scan;
    ble_init.ble_evt_handler     = ble_evt_handler;
    ble_init.scan_evt_handler    = scan_evt_handler;
    ble_init.db_disc_evt_handler = db_disc_handler;

    ble_init.gatts_init_funcs      = gatts_init_funcs;
    ble_init.gatts_init_func_count = sizeof(gatts_init_funcs) / sizeof(gatts_init_funcs[0]);

    ble_init.gattc_init_funcs      = gattc_init_funcs;
    ble_init.gattc_init_func_count = sizeof(gattc_init_funcs) / sizeof(gattc_init_funcs[0]);

    // Initialize services
    board_services_init(&board_init);
    ble_services_init(&ble_init);

    // Start execution
    NRF_LOG_INFO("Assitance Device started.");

    // Enter main loop
    for (;;)
    {
        idle_state_handle();
    }
}
