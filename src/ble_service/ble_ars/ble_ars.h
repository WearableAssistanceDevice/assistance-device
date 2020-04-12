#ifndef BLE_ARS_H
#define BLE_ARS_H

#include <stdint.h>
#include <stdbool.h>

#include "ble.h"
#include "ble_srv_common.h"

// Base 128-bit UUID for the assistance service
#define ARS_UUID_BASE {0xD2, 0x5F, 0xC5, 0xB3, 0xD6, 0xBA, 0xCF, 0x84, \
                      0x1E, 0x45, 0x13, 0x14, 0x6A, 0x66, 0xD7, 0xBA}

// 16-bit UUID for the service and its characteristics
#define ARS_UUID_SERVICE         0x1000
#define ARS_UUID_ASSIST_REQ_CHAR 0x1001


/**@brief   Macro for defining an assistance service instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_ARS_DEF(_name)                        \
static ble_ars_t _name;                          \
NRF_SDH_BLE_OBSERVER(_name ## _obs,              \
                     BLE_HRS_BLE_OBSERVER_PRIO,  \
                     ble_ars_on_ble_evt, &_name)


typedef enum {
    BLE_ARS_EVT_NOTIFICATION_ENABLED,   /**< Assistance Request notification enabled event. */
    BLE_ARS_EVT_NOTIFICATION_DISABLED,  /**< Assistance Request notification disabled event. */
    BLE_ARS_EVT_DISCONNECTED,
    BLE_ARS_EVT_CONNECTED,
    BLE_ARS_EVT_WRITE
} ble_ars_evt_type_t;

/**@brief Assistance Service event. */
typedef struct {
    ble_ars_evt_type_t evt_type;  /**< Type of event. */
} ble_ars_evt_t;


// Forward declaration of the ble_ars_t type.
typedef struct ble_ars_s ble_ars_t;


/**@brief Assistance Service event handler type. */
typedef void (*ble_ars_evt_handler_t)(ble_ars_t* p_ars, ble_ars_evt_t* p_evt);


/**@brief Assistance Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct {
    ble_ars_evt_handler_t        evt_handler;                   /**< Event handler to be called for handling events in the Assistance Service */
    uint8_t                      initial_assist_req_value;      /**< Initial value for the assistance request */
    security_req_t               assist_req_char_read_access;   /**< Security level for reading the Assistance Request characteristic */
    security_req_t               assist_req_char_write_access;  /**< Security level for writing the Assistance characteristics */
    security_req_t               assist_req_char_cccd_access;   /**< Security level for writing the Assistance characteristics's CCCD */
} ble_ars_init_t;


/**@brief Assistance Service structure. This contains various status information for the service. */
struct ble_ars_s {
    ble_ars_evt_handler_t    evt_handler;         /**< Event handler to be called for handling events in the Assistance Service */
    uint16_t                 service_handle;      /**< Handle of Assistance Service (as provided by the BLE stack) */
    ble_gatts_char_handles_t assist_req_handles;  /**< Handles related to the Assistance Requested characteristic */
    uint16_t                 conn_handle;         /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection) */
    uint8_t                  uuid_type; 
};


/**@brief Function for initializing the Assistance Service.
 *
 * @param[out]  p_ars       Assistance Service structure. This structure will have to be supplied by
 *                             the application. It will be initialized by this function, and will later
 *                             be used to identify this particular service instance.
 * @param[in]   p_ars_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_ars_init(ble_ars_t* p_ars, const ble_ars_init_t* p_ars_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Assistance Service.
 *
 * @note 
 *
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 * @param[in]   p_context  Assistance Service structure.
 */
void ble_ars_on_ble_evt(const ble_evt_t* p_ble_evt, void* p_context);



/**@brief Function for updating the assistance request value.
 *
 * @details The application calls this function when the cutom value should be updated.
 *
 * @note 
 *       
 * @param[in]   p_ars             Assistance Service structure
 * @param[in]   assist_req_value  Assistance Request value
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ars_assist_req_set(ble_ars_t* p_ars, uint8_t assist_req_value);



/**@brief Function for retrieving the assistance request value.
 *
 * @details The application calls this function when it wants to retrieve the value of
 *          the assistance request characteristic.
 *
 * @note 
 *       
 * @param[in]   p_ars               Assistance Service structure
 * @param[in]   p_assist_req_value  Pointer to the location to store the Assistance Request value
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ars_assist_req_get(ble_ars_t* p_ars, uint8_t* p_assist_req_value);

#endif //BLE_ARS_H