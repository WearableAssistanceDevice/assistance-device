#include "ble_assist_service.h"
#include "ble_service/ble_config.h"

#include "nrf_sdh_ble.h"

#include "ble_ars/ble_ars.h"


BLE_ARS_DEF(m_assist);  /**< Define the assistance service instance */


/**@brief Function for handling Assistance Request related BSP events.
 *
 * @param[in] event  BSP event.
 */
void ble_assist_bsp_evt_handler(bsp_event_t event) {
    switch (event) {
        case ASSISTANCE_REQUEST_BUTTON_EVT:
            ble_ars_assist_req_set(&m_assist, true);
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Assistance Service Service events.
 *
 * @details This function will be called for all Assistance Service events which are passed to
 *          the application.
 *
 * @param[in]   p_assist       Assistance Service structure.
 * @param[in]   p_evt          Event received from the Custom Service.
 *
 */
static void on_assist_evt(ble_ars_t* p_assist, ble_ars_evt_t* p_evt) {
    uint32_t err_code;
    uint8_t assist_requested;

    switch(p_evt->evt_type) {
        case BLE_ARS_EVT_NOTIFICATION_ENABLED:
            break;

        case BLE_ARS_EVT_NOTIFICATION_DISABLED:
            break;

        case BLE_ARS_EVT_CONNECTED:
            break;

        case BLE_ARS_EVT_DISCONNECTED:
            break;

        case BLE_ARS_EVT_WRITE: {
            err_code = ble_ars_assist_req_get(p_assist, &assist_requested);
            APP_ERROR_CHECK(err_code);
            if (assist_requested) {
                bsp_board_led_on(ASSISTANCE_REQUEST_LED);
            }
            else {
                bsp_board_led_off(ASSISTANCE_REQUEST_LED);
            }
            break;
        }

        default:
            break;
    }
}


static void assist_service_init(void) {
    ble_ars_init_t assist_init = {0};
    
    assist_init.evt_handler = on_assist_evt;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&assist_init.assist_req_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&assist_init.assist_req_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&assist_init.assist_req_char_attr_md.write_perm);

    const ret_code_t err_code = ble_ars_init(&m_assist, &assist_init);
    APP_ERROR_CHECK(err_code);
}


void ble_assist_service_init(void) {
    assist_service_init();
}