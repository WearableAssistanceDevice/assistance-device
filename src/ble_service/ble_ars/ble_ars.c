#include "ble_ars.h"

#include <string.h>
#include "sdk_common.h"
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "bsp_config.h"
#include "boards.h"
#include "nrf_log.h"


/**@brief Function for adding the Assistance Requested characteristic.
 *
 * @param[in]   p_ars        Assistance Service structure.
 * @param[in]   p_ars_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t assistance_request_char_add(ble_ars_t* p_ars, const ble_ars_init_t* p_ars_init) {
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.write_perm = p_ars_init->assist_req_char_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;
		
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.read_perm  = p_ars_init->assist_req_char_attr_md.read_perm;
    attr_md.write_perm = p_ars_init->assist_req_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    ble_uuid.type = p_ars->uuid_type;
    ble_uuid.uuid = ARS_UUID_ASSIST_REQ_CHAR;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = &p_ars_init->initial_assist_req_value;

    return sd_ble_gatts_characteristic_add(p_ars->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_ars->assist_req_handles);
}


uint32_t ble_ars_init(ble_ars_t* p_ars, const ble_ars_init_t* p_ars_init) {
    if (p_ars == NULL || p_ars_init == NULL) {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service struct
    p_ars->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_ars->evt_handler = p_ars_init->evt_handler;

    // Add Assistance Service UUID
    ble_uuid128_t base_uuid = {ARS_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_ars->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_ars->uuid_type;
    ble_uuid.uuid = ARS_UUID_SERVICE;

    // Add the Assistance Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ars->service_handle);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    return assistance_request_char_add(p_ars, p_ars_init);
}





uint32_t ble_ars_assist_req_set(ble_ars_t* p_ars, uint8_t assist_req_value) {
    if (p_ars == NULL) {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct
    memset(&gatts_value, 0, sizeof(gatts_value));
    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &assist_req_value;

    // Update database
    err_code = sd_ble_gatts_value_set(p_ars->conn_handle,
                                      p_ars->assist_req_handles.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // Send value if connected and notifying
    if (p_ars->conn_handle != BLE_CONN_HANDLE_INVALID) {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ars->assist_req_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset  = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_ars->conn_handle, &hvx_params);
    }
/*
    // Send write signal
    if (p_ars->evt_handler != NULL) {
        ble_ars_evt_t evt;
        evt.evt_type = BLE_ARS_EVT_WRITE;
        p_ars->evt_handler(p_ars, &evt);
    }
*/
    return err_code;
}

uint32_t ble_ars_assist_req_get(ble_ars_t* p_ars, uint8_t* p_assist_req_value) {
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct
    memset(&gatts_value, 0, sizeof(gatts_value));
    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset   = 0;
    gatts_value.p_value = p_assist_req_value;

    // Retrieve the value
    err_code = sd_ble_gatts_value_get(p_ars->conn_handle,
                                      p_ars->assist_req_handles.value_handle,
                                      &gatts_value);

    return err_code;
}


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_ars    Assistance Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_ars_t* p_ars, const ble_evt_t* p_ble_evt) {
    p_ars->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    if (p_ars->evt_handler != NULL) {
        ble_ars_evt_t evt;
        evt.evt_type = BLE_ARS_EVT_CONNECTED;
        p_ars->evt_handler(p_ars, &evt);
    }
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_ars    Assistance Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_ars_t* p_ars, const ble_evt_t* p_ble_evt) {
    UNUSED_PARAMETER(p_ble_evt);
    p_ars->conn_handle = BLE_CONN_HANDLE_INVALID;

    if (p_ars->evt_handler != NULL) {
        ble_ars_evt_t evt;
        evt.evt_type = BLE_ARS_EVT_DISCONNECTED;
        p_ars->evt_handler(p_ars, &evt);
    }
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ars    Assistance Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_ars_t* p_ars, const ble_evt_t* p_ble_evt) {
    const ble_gatts_evt_write_t* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    // Check if the handle passed with the event matches the Assistance Requested Characteristic handle
    if (p_evt_write->handle == p_ars->assist_req_handles.value_handle) {
        if (p_ars->evt_handler != NULL) {
            ble_ars_evt_t evt;
            evt.evt_type = BLE_ARS_EVT_WRITE;
            p_ars->evt_handler(p_ars, &evt);
        }
    }

    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_ars->assist_req_handles.cccd_handle) && (p_evt_write->len == 2)) {
        // CCCD written, call application event handler
        if (p_ars->evt_handler != NULL) {
            ble_ars_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data)) {
                evt.evt_type = BLE_ARS_EVT_NOTIFICATION_ENABLED;
            }
            else {
                evt.evt_type = BLE_ARS_EVT_NOTIFICATION_DISABLED;
            }

            // Call the application event handler
            p_ars->evt_handler(p_ars, &evt);
        }
    }
}

void ble_ars_on_ble_evt(const ble_evt_t* p_ble_evt, void* p_context) {
    ble_ars_t* p_ars = (ble_ars_t*)p_context;
    
    if (p_ars == NULL || p_ble_evt == NULL) {
        return;
    }

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ars, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ars, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ars, p_ble_evt);
            break;

        default:
            break;
    }
}