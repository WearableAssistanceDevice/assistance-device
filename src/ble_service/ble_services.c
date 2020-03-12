#include "ble_services.h"

#include "config.h"

#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_ble_scan.h"

#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_db_discovery.h"
#include "nrf_ble_gatt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


static struct {
    nrf_ble_scan_t*            p_ble_scan;
    nrf_ble_gatt_t*            p_ble_gatt;
    ble_db_discovery_t*        p_ble_db_discovery;
    nrf_ble_gq_t*              p_ble_qatt_queue;

    ble_scan_evt_handler_t     scan_evt_handler;
    ble_evt_handler_t          ble_evt_handler;
    db_discovery_evt_handler_t db_disc_evt_handler;
} ble_services_config;


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(const ble_evt_t* p_ble_evt, void* p_context)
{
    ret_code_t err_code;

    // For readability.
    const ble_gap_evt_t* p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
            const uint8_t* addr = p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr;
            NRF_LOG_INFO("Connected to %02x:%02x:%02x:%02x:%02x:%02x", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

            err_code = ble_db_discovery_start(ble_services_config.p_ble_db_discovery, p_gap_evt->conn_handle);
            //APP_ERROR_CHECK(err_code);
        } break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, and update the LEDs status.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected.");
        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
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

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }

    if (ble_services_config.ble_evt_handler != NULL) {
        ble_services_config.ble_evt_handler(p_ble_evt, p_context);
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
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


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t* p_evt)
{
    if (ble_services_config.db_disc_evt_handler != NULL) {
        ble_services_config.db_disc_evt_handler(p_evt);
    }
}


/**@brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(db_init));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = ble_services_config.p_ble_qatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(const scan_evt_t* p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    if (ble_services_config.scan_evt_handler != NULL) {
        ble_services_config.scan_evt_handler(p_scan_evt);
    }
}


static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(ble_services_config.p_ble_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(ble_services_config.p_ble_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


void ble_services_init(const ble_services_init_t* p_init) {
    if (p_init == NULL) {
        APP_ERROR_CHECK(NRF_ERROR_NULL);
        return;
    }
    if (p_init->p_ble_db_discovery == NULL ||
        p_init->p_ble_gatt         == NULL ||
        p_init->p_ble_qatt_queue   == NULL ||
        p_init->p_ble_scan         == NULL) {
        APP_ERROR_CHECK(NRF_ERROR_NULL);
        return;
    }

    // Copy event handler function pointers
    ble_services_config.p_ble_db_discovery  = p_init->p_ble_db_discovery;
    ble_services_config.p_ble_gatt          = p_init->p_ble_gatt;
    ble_services_config.p_ble_qatt_queue    = p_init->p_ble_qatt_queue;
    ble_services_config.p_ble_scan          = p_init->p_ble_scan;
    ble_services_config.ble_evt_handler     = p_init->ble_evt_handler;
    ble_services_config.scan_evt_handler    = p_init->scan_evt_handler;
    ble_services_config.db_disc_evt_handler = p_init->db_disc_evt_handler;

    // Initialize core BLE services
    ble_stack_init();
    scan_init();
    gatt_init();
    db_discovery_init();

    // Initialize GATT server and client services
    for (int i = 0; i < p_init->gatts_init_func_count; ++i) {
        p_init->gatts_init_funcs[i]();
    }
    for (int i = 0; i < p_init->gattc_init_func_count; ++i) {
        p_init->gattc_init_funcs[i](p_init->p_ble_qatt_queue);
    }
}