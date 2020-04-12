#include "ble_services.h"

#include "config.h"

#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_bms.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "fds.h"

#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_db_discovery.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_srv_common.h"

#include "peer_manager.h"
#include "peer_manager_handler.h"




/**< Storage for module and event handler pointers */
static struct {
    nrf_ble_bms_t*             p_ble_bms;
    ble_db_discovery_t*        p_ble_db_discovery;
    nrf_ble_gatt_t*            p_ble_gatt;
    nrf_ble_gq_t*              p_ble_qatt_queue;
    nrf_ble_qwr_t*             p_ble_qwr;
    nrf_ble_scan_t*            p_ble_scan;

    ble_scan_evt_handler_t     scan_evt_handler;
    ble_evt_handler_t          ble_evt_handler;
    db_discovery_evt_handler_t db_disc_evt_handler;
} ble_services_config;


#define MEM_BUFF_SIZE 512
static uint8_t m_qwr_mem[MEM_BUFF_SIZE]; /**< Write buffer for the Queued Write module. */


/**< Flags used to identify bonds that should be deleted */
static ble_conn_state_user_flag_id_t m_bms_bonds_to_delete;




/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}




/**@brief Function for handling Queued Write module events
 */
uint16_t qwr_evt_handler(nrf_ble_qwr_t* p_qwr, nrf_ble_qwr_evt_t* p_evt)
{
    return nrf_ble_bms_on_qwr_evt(ble_services_config.p_ble_bms, p_qwr, p_evt);
}

/**@brief Function for initializing the Queued Write module
 */
static void qwr_init(void)
{
    ret_code_t err_code;

    nrf_ble_qwr_init_t qwr_init;
    memset(&qwr_init, 0, sizeof(qwr_init));

    qwr_init.mem_buffer.len   = MEM_BUFF_SIZE;
    qwr_init.mem_buffer.p_mem = m_qwr_mem;
    qwr_init.callback         = qwr_evt_handler;
    qwr_init.error_handler    = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(ble_services_config.p_ble_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
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




/**@brief Function for deleting a single bond if it does not belong to a connected peer.
 *
 * This will mark the bond for deferred deletion if the peer is connected.
 */
static void bond_delete(uint16_t conn_handle, void* p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t   err_code;
    pm_peer_id_t peer_id;

    if (ble_conn_state_status(conn_handle) == BLE_CONN_STATUS_CONNECTED)
    {
        ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, true);
    }
    else
    {
        NRF_LOG_DEBUG("Attempting to delete bond.");
        err_code = pm_peer_id_get(conn_handle, &peer_id);
        APP_ERROR_CHECK(err_code);
        if (peer_id != PM_PEER_ID_INVALID)
        {
            err_code = pm_peer_delete(peer_id);
            APP_ERROR_CHECK(err_code);
            ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, false);
        }
    }
}

/**@brief Function for performing deferred deletions.
*/
static void delete_disconnected_bonds(void)
{
    uint32_t n_calls = ble_conn_state_for_each_set_user_flag(m_bms_bonds_to_delete, bond_delete, NULL);
    UNUSED_RETURN_VALUE(n_calls);
}

/**@brief Function for handling events from bond management service.
 */
void bms_evt_handler(nrf_ble_bms_t* p_ess, nrf_ble_bms_evt_t* p_evt)
{
    ret_code_t err_code;
    bool is_authorized = true;

    switch (p_evt->evt_type)
    {
        case NRF_BLE_BMS_EVT_AUTH:
            NRF_LOG_DEBUG("Authorization request");
#if USE_AUTHORIZATION_CODE
            if ((p_evt->auth_code.len != m_auth_code_len) ||
                (memcmp(m_auth_code, p_evt->auth_code.code, m_auth_code_len) != 0))
            {
                is_authorized = false;
            }
#endif
            err_code = nrf_ble_bms_auth_response(ble_services_config.p_ble_bms, is_authorized);
            APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for marking the requester's bond for deletion.
*/
static void delete_requesting_bond(const nrf_ble_bms_t* p_bms)
{
    NRF_LOG_INFO("Client requested that bond to current device deleted");
    ble_conn_state_user_flag_set(p_bms->conn_handle, m_bms_bonds_to_delete, true);
}

/**@brief Function for deleting all bonds
*/
static void delete_all_bonds(const nrf_ble_bms_t* p_bms)
{
    ret_code_t err_code;
    uint16_t conn_handle;

    NRF_LOG_INFO("Client requested that all bonds be deleted");

    pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    while (peer_id != PM_PEER_ID_INVALID)
    {
        err_code = pm_conn_handle_get(peer_id, &conn_handle);
        APP_ERROR_CHECK(err_code);

        bond_delete(conn_handle, NULL);

        peer_id = pm_next_peer_id_get(peer_id);
    }
}

/**@brief Function for deleting all bet requesting device bonds
*/
static void delete_all_except_requesting_bond(const nrf_ble_bms_t* p_bms)
{
    ret_code_t err_code;
    uint16_t conn_handle;

    NRF_LOG_INFO("Client requested that all bonds except current bond be deleted");

    pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    while (peer_id != PM_PEER_ID_INVALID)
    {
        err_code = pm_conn_handle_get(peer_id, &conn_handle);
        APP_ERROR_CHECK(err_code);

        /* Do nothing if this is our own bond. */
        if (conn_handle != p_bms->conn_handle)
        {
            bond_delete(conn_handle, NULL);
        }

        peer_id = pm_next_peer_id_get(peer_id);
    }
}

/**@brief Function for initializing the Bond Management Service
 */
static void bms_init(void)
{
    ret_code_t err_code;

#ifdef USE_AUTHORIZATION_CODE
    static const uint8_t m_auth_code[] = {'A', 'B', 'C', 'D'}; //0x41, 0x42, 0x43, 0x44
    static const int m_auth_code_len = sizeof(m_auth_code);
#endif

    nrf_ble_bms_init_t bms_init;
    memset(&bms_init, 0, sizeof(bms_init));

    m_bms_bonds_to_delete  = ble_conn_state_user_flag_acquire();
    bms_init.evt_handler   = bms_evt_handler;
    bms_init.error_handler = service_error_handler;
#if USE_AUTHORIZATION_CODE
    bms_init.feature.delete_requesting_auth         = true;
    bms_init.feature.delete_all_auth                = true;
    bms_init.feature.delete_all_but_requesting_auth = true;
#else
    bms_init.feature.delete_requesting              = true;
    bms_init.feature.delete_all                     = true;
    bms_init.feature.delete_all_but_requesting      = true;
#endif
    bms_init.bms_feature_sec_req = SEC_JUST_WORKS;
    bms_init.bms_ctrlpt_sec_req  = SEC_JUST_WORKS;

    bms_init.p_qwr                                       = ble_services_config.p_ble_qwr;
    bms_init.bond_callbacks.delete_requesting            = delete_requesting_bond;
    bms_init.bond_callbacks.delete_all                   = delete_all_bonds;
    bms_init.bond_callbacks.delete_all_except_requesting = delete_all_except_requesting_bond;

    err_code = nrf_ble_bms_init(ble_services_config.p_ble_bms, &bms_init);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(const pm_evt_t* p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
            break;

        case PM_EVT_CONN_SEC_FAILED:
            NRF_LOG_INFO("Connection security failed for connection handle %d. Deleting bond info...", p_evt->conn_handle);
            bond_delete(p_evt->conn_handle, NULL);
            sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_AUTHENTICATION_FAILURE);
            break;

        default:
            break;
    }
}

/**@brief Function for the Peer Manager initialization
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
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

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONNECTION_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONNECTION_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = SUPERVISION_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(ble_services_config.p_ble_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}




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

            err_code = nrf_ble_bms_set_conn_handle(ble_services_config.p_ble_bms, p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
            err_code = nrf_ble_qwr_conn_handle_assign(ble_services_config.p_ble_qwr, p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(ble_services_config.p_ble_db_discovery, p_gap_evt->conn_handle);
            //APP_ERROR_CHECK(err_code);
        } break;

        // Upon disconnection, reset the connection handle of the peer which disconnected, and update the LEDs status.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected.");
            delete_disconnected_bonds();
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

    // Copy configuration parameters
    ble_services_config.p_ble_bms           = p_init->p_ble_bms;
    ble_services_config.p_ble_db_discovery  = p_init->p_ble_db_discovery;
    ble_services_config.p_ble_gatt          = p_init->p_ble_gatt;
    ble_services_config.p_ble_qatt_queue    = p_init->p_ble_qatt_queue;
    ble_services_config.p_ble_qwr           = p_init->p_ble_qwr;
    ble_services_config.p_ble_scan          = p_init->p_ble_scan;
    ble_services_config.ble_evt_handler     = p_init->ble_evt_handler;
    ble_services_config.scan_evt_handler    = p_init->scan_evt_handler;
    ble_services_config.db_disc_evt_handler = p_init->db_disc_evt_handler;

    // Initialize core BLE services
    ble_stack_init();
    qwr_init();
    gap_params_init();
    gatt_init();
    scan_init();
    db_discovery_init();
    bms_init();
    peer_manager_init();

    // Initialize GATT server and client services
    for (int i = 0; i < p_init->gatts_init_func_count; ++i) {
        p_init->gatts_init_funcs[i]();
    }
    for (int i = 0; i < p_init->gattc_init_func_count; ++i) {
        p_init->gattc_init_funcs[i](p_init->p_ble_qatt_queue);
    }
}