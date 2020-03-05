#pragma once

#include "bsp.h"


/**@brief Function for initializing the BLE Assistance Request service.
 */
void ble_assist_service_init(void);


/**@brief Function for handling Assistance Request related BSP events.
 *
 * @param[in] event  BSP event.
 */
void ble_assist_bsp_evt_handler(bsp_event_t event);