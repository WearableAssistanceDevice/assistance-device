#include "ble_dls_c.h"

#include "sdk_common.h"

#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_gattc.h"

#define NRF_LOG_MODULE_NAME ble_dls_c

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */


/**@brief Function for intercepting the errors of GATTC and the BLE GATT Queue.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
                               void*      p_ctx,
                               uint16_t   conn_handle)
{
    ble_dls_c_t * p_ble_dls_c = (ble_dls_c_t *)p_ctx;

    NRF_LOG_DEBUG("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);

    if (p_ble_dls_c->error_handler != NULL)
    {
        p_ble_dls_c->error_handler(nrf_error);
    }
}


/**@brief Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details This function uses the Handle Value Notification received from the SoftDevice
 *          and checks whether it is a notification of Button state from the peer. If
 *          it is, this function decodes the state of the button and sends it to the
 *          application.
 *
 * @param[in] p_ble_dls_c Pointer to the Led Button Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_dls_c_t* p_ble_dls_c, const ble_evt_t* p_ble_evt)
{
    // Check if the event is on the link for this instance.
    if (p_ble_dls_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    // Check if this is a Button notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_dls_c->peer_dls_db.lock_state_handle)
    {
        if (p_ble_evt->evt.gattc_evt.params.hvx.len == 1)
        {
            ble_dls_c_evt_t ble_dls_c_evt;

            ble_dls_c_evt.evt_type                = BLE_DLS_C_EVT_LOCK_NOTIFICATION;
            ble_dls_c_evt.conn_handle             = p_ble_dls_c->conn_handle;
            ble_dls_c_evt.params.lock_state.state = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
            p_ble_dls_c->evt_handler(p_ble_dls_c, &ble_dls_c_evt);
        }
    }
}


/**@brief Function for handling the Disconnected event received from the SoftDevice.
 *
 * @details This function checks whether the disconnect event is happening on the link
 *          associated with the current instance of the module. If the event is happening, the function sets the instance's
 *          conn_handle to invalid.
 *
 * @param[in] p_ble_dls_c Pointer to the Led Button Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_dls_c_t* p_ble_dls_c, const ble_evt_t* p_ble_evt)
{
    if (p_ble_dls_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ble_dls_c->conn_handle                        = BLE_CONN_HANDLE_INVALID;
        p_ble_dls_c->peer_dls_db.lock_state_cccd_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_dls_c->peer_dls_db.lock_state_handle      = BLE_GATT_HANDLE_INVALID;
    }
}


void ble_dls_on_db_disc_evt(ble_dls_c_t* p_ble_dls_c, const ble_db_discovery_evt_t* p_evt)
{
    // Check if the LED Button Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == DLS_UUID_SERVICE &&
        p_evt->params.discovered_db.srv_uuid.type == p_ble_dls_c->uuid_type)
    {
        ble_dls_c_evt_t evt;

        evt.evt_type    = BLE_DLS_C_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;

        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            const ble_gatt_db_char_t * p_char = &(p_evt->params.discovered_db.charateristics[i]);
            switch (p_char->characteristic.uuid.uuid)
            {
                case DLS_UUID_LOCK_STATE_CHAR:
                    evt.params.peer_db.lock_state_handle      = p_char->characteristic.handle_value;
                    evt.params.peer_db.lock_state_cccd_handle = p_char->cccd_handle;
                    break;

                default:
                    break;
            }
        }

        NRF_LOG_DEBUG("Door Lock Service discovered at peer.");
        //If the instance was assigned prior to db_discovery, assign the db_handles
        if (p_ble_dls_c->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if ((p_ble_dls_c->peer_dls_db.lock_state_handle      == BLE_GATT_HANDLE_INVALID) &&
                (p_ble_dls_c->peer_dls_db.lock_state_cccd_handle == BLE_GATT_HANDLE_INVALID))
            {
                p_ble_dls_c->peer_dls_db = evt.params.peer_db;
            }
        }

        p_ble_dls_c->evt_handler(p_ble_dls_c, &evt);

    }
}


uint32_t ble_dls_c_init(ble_dls_c_t* p_ble_dls_c, ble_dls_c_init_t* p_ble_dls_c_init)
{
    uint32_t      err_code;
    ble_uuid_t    dls_uuid;
    ble_uuid128_t dls_base_uuid = {DLS_UUID_BASE};

    VERIFY_PARAM_NOT_NULL(p_ble_dls_c);
    VERIFY_PARAM_NOT_NULL(p_ble_dls_c_init);
    VERIFY_PARAM_NOT_NULL(p_ble_dls_c_init->evt_handler);
    VERIFY_PARAM_NOT_NULL(p_ble_dls_c_init->p_gatt_queue);

    p_ble_dls_c->peer_dls_db.lock_state_cccd_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_dls_c->peer_dls_db.lock_state_handle      = BLE_GATT_HANDLE_INVALID;
    p_ble_dls_c->conn_handle                        = BLE_CONN_HANDLE_INVALID;
    p_ble_dls_c->evt_handler                        = p_ble_dls_c_init->evt_handler;
    p_ble_dls_c->p_gatt_queue                       = p_ble_dls_c_init->p_gatt_queue;
    p_ble_dls_c->error_handler                      = p_ble_dls_c_init->error_handler;

    err_code = sd_ble_uuid_vs_add(&dls_base_uuid, &p_ble_dls_c->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    VERIFY_SUCCESS(err_code);

    dls_uuid.type = p_ble_dls_c->uuid_type;
    dls_uuid.uuid = DLS_UUID_SERVICE;

    return ble_db_discovery_evt_register(&dls_uuid);
}

void ble_dls_c_on_ble_evt(const ble_evt_t* p_ble_evt, void* p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_dls_c_t* p_ble_dls_c = (ble_dls_c_t*)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_dls_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_dls_c, p_ble_evt);
            break;

        default:
            break;
    }
}


/**@brief Function for configuring the CCCD.
 *
 * @param[in] p_ble_dls_c Pointer to the LED Button Client structure.
 * @param[in] enable      Whether to enable or disable the CCCD.
 *
 * @return NRF_SUCCESS if the CCCD configure was successfully sent to the peer.
 */
static uint32_t cccd_configure(ble_dls_c_t* p_ble_dls_c, bool enable)
{
    NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
                  p_ble_dls_c->peer_dls_db.lock_state_cccd_handle,
                  p_ble_dls_c->conn_handle);

    nrf_ble_gq_req_t cccd_req;
    uint16_t         cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    uint8_t          cccd[WRITE_MESSAGE_LENGTH];

    cccd[0] = LSB_16(cccd_val);
    cccd[1] = MSB_16(cccd_val);

    cccd_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    cccd_req.error_handler.cb            = gatt_error_handler;
    cccd_req.error_handler.p_ctx         = p_ble_dls_c;
    cccd_req.params.gattc_write.handle   = p_ble_dls_c->peer_dls_db.lock_state_cccd_handle;
    cccd_req.params.gattc_write.len      = WRITE_MESSAGE_LENGTH;
    cccd_req.params.gattc_write.offset   = 0;
    cccd_req.params.gattc_write.p_value  = cccd;
    cccd_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;

    return nrf_ble_gq_item_add(p_ble_dls_c->p_gatt_queue, &cccd_req, p_ble_dls_c->conn_handle);
}


uint32_t ble_dls_c_lock_state_notif_enable(ble_dls_c_t* p_ble_dls_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_dls_c);

    if (p_ble_dls_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return cccd_configure(p_ble_dls_c,
                          true);
}


uint32_t ble_dls_c_lock_state_get(ble_dls_c_t* p_ble_dls_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_dls_c);

    if (p_ble_dls_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    NRF_LOG_DEBUG("Reading Door Lock status");

    nrf_ble_gq_req_t read_req;

    memset(&read_req, 0, sizeof(nrf_ble_gq_req_t));

    read_req.type                        = NRF_BLE_GQ_REQ_GATTC_READ;
    read_req.error_handler.cb            = gatt_error_handler;
    read_req.error_handler.p_ctx         = p_ble_dls_c;
    read_req.params.gattc_read.handle    = p_ble_dls_c->peer_dls_db.lock_state_handle;
    read_req.params.gattc_read.offset    = 0;

    return nrf_ble_gq_item_add(p_ble_dls_c->p_gatt_queue, &read_req, p_ble_dls_c->conn_handle);
}


uint32_t ble_dls_c_lock_state_send(ble_dls_c_t* p_ble_dls_c, uint8_t status)
{
    VERIFY_PARAM_NOT_NULL(p_ble_dls_c);

    if (p_ble_dls_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    NRF_LOG_DEBUG("Writing Door Lock status 0x%x", status);

    nrf_ble_gq_req_t write_req;

    memset(&write_req, 0, sizeof(nrf_ble_gq_req_t));

    write_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    write_req.error_handler.cb            = gatt_error_handler;
    write_req.error_handler.p_ctx         = p_ble_dls_c;
    write_req.params.gattc_write.handle   = p_ble_dls_c->peer_dls_db.lock_state_handle;
    write_req.params.gattc_write.len      = sizeof(status);
    write_req.params.gattc_write.p_value  = &status;
    write_req.params.gattc_write.offset   = 0;
    write_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_CMD; 

    return nrf_ble_gq_item_add(p_ble_dls_c->p_gatt_queue, &write_req, p_ble_dls_c->conn_handle);
}

uint32_t ble_dls_c_handles_assign(ble_dls_c_t*    p_ble_dls_c,
                                  uint16_t        conn_handle,
                                  const dls_db_t* p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_dls_c);

    p_ble_dls_c->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_ble_dls_c->peer_dls_db = *p_peer_handles;
    }
    return nrf_ble_gq_conn_handle_register(p_ble_dls_c->p_gatt_queue, conn_handle);
}

