#pragma once

#include <stdbool.h>

#include "bsp.h"
#include "ble.h"
#include "ble_advertising.h"



/**@brief BLE advertisement event handler type */
typedef void (*ble_adv_evt_handler_t)(ble_adv_evt_t ble_adv_evt);

/**@brief BLE event handler type */
typedef void (*ble_evt_handler_t)(const ble_evt_t* p_ble_evt, void* p_context);

/**@brief BLE service initialization function type */
typedef void (*ble_service_init_func_t)(void);

/**@brief BLE services init structure */
typedef struct {
    ble_adv_evt_handler_t    adv_evt_handler;
    ble_evt_handler_t        ble_evt_handler;
    ble_service_init_func_t* service_init_funcs;
    unsigned int             service_init_func_count;
    ble_uuid_t*              adv_uuids;
    unsigned int             adv_uuid_count;
} ble_services_init_t;



/**@brief Function for initializing BLE services.
 *
 * @param[in] p_init  BLE service initialization config.
 */
void ble_services_init(const ble_services_init_t* p_init);


/**@brief Function for starting BLE advertising.
 *
 * @param[in] erase_bonds  True if existing bonds should be erased.
 */
void advertising_start(bool erase_bonds);


/**@brief Function for handling BLE-related BSP events.
 *
 * @param[in] event  BSP event.
 */
void ble_bsp_evt_handler(bsp_event_t event);