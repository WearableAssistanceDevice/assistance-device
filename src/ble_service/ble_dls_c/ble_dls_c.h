/**@file
 *
 * @defgroup ble_dls_c Door Lock Service Client
 * @{
 * @brief    The Door Lock Service client is used to interact with a Bluetooth door lock.
 *
 * @details  This module contains the APIs and types exposed by the Door Lock Service Client
 *           module. The application can use these APIs and types to perform the discovery of
 *           Door Lock Service at the peer and to interact with it.
 *
 * @note    The application must register this module as the BLE event observer by using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_dls_c_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_DLS_C_BLE_OBSERVER_PRIO,
 *                                   ble_dls_c_on_ble_evt, &instance);
 *          @endcode
 */

#ifndef BLE_DLS_C_H__
#define BLE_DLS_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "nrf_ble_gq.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_DLS_C_BLE_OBSERVER_PRIO 2

/**@brief   Macro for defining a ble_dls_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_DLS_C_DEF(_name)                         \
static ble_dls_c_t _name;                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                 \
                     BLE_DLS_C_BLE_OBSERVER_PRIO,   \
                     ble_dls_c_on_ble_evt, &_name)

/**@brief   Macro for defining multiple ble_dls_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 */
#define BLE_DLS_C_ARRAY_DEF(_name, _cnt)                    \
static ble_dls_c_t _name[_cnt];                            \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                       \
                      BLE_DLS_C_BLE_OBSERVER_PRIO,         \
                      ble_dls_c_on_ble_evt, &_name, _cnt)


// Base 128-bit UUID for the door lock service
#define DLS_UUID_BASE { 0x7D, 0x94, 0xA7, 0x59, 0x10, 0xD8, 0x0B, 0xAB, \
                        0x4D, 0x4E, 0xA9, 0x10, 0x0D, 0xA3, 0xE1, 0xD1 }

// 16-bit UUID for the service and its characteristics
#define DLS_UUID_SERVICE        0x2000
#define DLS_UUID_LOCK_STATE_CHAR 0x2001


/**@brief DLS Client event type. */
typedef enum
{
    BLE_DLS_C_EVT_DISCOVERY_COMPLETE = 1,  /**< Event indicating that the Door Lock Service was discovered at the peer. */
    BLE_DLS_C_EVT_LOCK_NOTIFICATION        /**< Event indicating that a notification of the Door Lock characteristic was received from the peer. */
} ble_dls_c_evt_type_t;

/**@brief Structure containing the Door Lock state received from the peer. */
typedef struct
{
    uint8_t state;  /**< Door Lock Value. */
} ble_lock_state_t;

/**@brief Structure containing the handles related to the Door Lock Service found on the peer. */
typedef struct
{
    uint16_t lock_state_cccd_handle;  /**< Handle of the CCCD of the Door Lock characteristic. */
    uint16_t lock_state_handle;       /**< Handle of the Door Lock characteristic as provided by the SoftDevice. */
} dls_db_t;

/**@brief Door Lock Event structure. */
typedef struct
{
    ble_dls_c_evt_type_t evt_type;     /**< Type of the event. */
    uint16_t             conn_handle;  /**< Connection handle on which the event occured.*/
    union
    {
        ble_lock_state_t lock_state;   /**< Door Lock value received. This is filled if the evt_type is @ref BLE_DLS_C_EVT_BUTTON_NOTIFICATION. */
        dls_db_t         peer_db;      /**< Handles related to the Door Lock Service found on the peer device. This is filled if the evt_type is @ref BLE_DLS_C_EVT_DISCOVERY_COMPLETE.*/
    } params;
} ble_dls_c_evt_t;

// Forward declaration of the ble_dls_c_t type.
typedef struct ble_dls_c_s ble_dls_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that is to be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_dls_c_evt_handler_t)(ble_dls_c_t* p_ble_dls_c, ble_dls_c_evt_t* p_evt);

/**@brief Door Lock Client structure. */
struct ble_dls_c_s
{
    uint16_t                  conn_handle;   /**< Connection handle as provided by the SoftDevice. */
    dls_db_t                  peer_dls_db;   /**< Handles related to DLS on the peer. */
    ble_dls_c_evt_handler_t   evt_handler;   /**< Application event handler to be called when there is an event related to the Door Lock service. */
    ble_srv_error_handler_t   error_handler; /**< Function to be called in case of an error. */
    uint8_t                   uuid_type;     /**< UUID type. */
    nrf_ble_gq_t*             p_gatt_queue;  /**< Pointer to the BLE GATT Queue instance. */
};

/**@brief Door Lock Client initialization structure. */
typedef struct
{
    ble_dls_c_evt_handler_t   evt_handler;   /**< Event handler to be called by the Door Lock Client module when there is an event related to the Door Lock Service. */
    nrf_ble_gq_t*             p_gatt_queue;  /**< Pointer to the BLE GATT Queue instance. */
    ble_srv_error_handler_t   error_handler; /**< Function to be called in case of an error. */
} ble_dls_c_init_t;


/**@brief Function for initializing the Door Lock client module.
 *
 * @details This function registers with the Database Discovery module for the
 *          Door Lock Service. The module looks for the presence of a Door Lock Service instance
 *          at the peer when a discovery is started.
 *
 * @param[in] p_ble_dls_c      Pointer to the Door Lock client structure.
 * @param[in] p_ble_dls_c_init Pointer to the Door Lock initialization structure that contains the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS On successful initialization. 
 * @retval    err_code    Otherwise, this function propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_dls_c_init(ble_dls_c_t* p_ble_dls_c, ble_dls_c_init_t* p_ble_dls_c_init);


/**@brief Function for handling BLE events from the SoftDevice.
 *
 * @details This function handles the BLE events received from the SoftDevice. If a BLE event
 *          is relevant to the Door Lock Client module, the function uses the event's data to update interval
 *          variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the door lock client structure.
 */
void ble_dls_c_on_ble_evt(const ble_evt_t* p_ble_evt, void* p_context);


/**@brief Function for requesting the peer to start sending notification of the Button
 *        Characteristic.
 *
 * @details This function enables to notification of the Button at the peer
 *          by writing to the CCCD of the Button characteristic.
 *
 * @param[in] p_ble_dls_c Pointer to the Door Lock Client structure.
 *
 * @retval  NRF_SUCCESS 			If the SoftDevice has been requested to write to the CCCD of the peer.
 * @retval  NRF_ERROR_INVALID_STATE If no connection handle has been assigned (@ref ble_dls_c_handles_assign).
 * @retval  NRF_ERROR_NULL 			If the given parameter is NULL.
 * @retval  err_code				Otherwise, this API propagates the error code returned by function
 *          						@ref nrf_ble_gq_item_add.
 */
uint32_t ble_dls_c_lock_req_notif_enable(ble_dls_c_t* p_ble_dls_c);


/**@brief Function for handling events from the Database Discovery module.
 *
 * @details Call this function when you get a callback event from the Database Discovery module. This
 *          function handles an event from the Database Discovery module, and determines whether it
 *          relates to the discovery of Door Lock service at the peer. If it does, this function calls the
 *          application's event handler to indicate that the Door Lock service was discovered
 *          at the peer. The function also populates the event with service-related information before
 *          providing it to the application.
 *
 * @param[in] p_ble_dls_c Pointer to the Door Lock client structure.
 * @param[in] p_evt Pointer to the event received from the Database Discovery module.
 */
void ble_dls_on_db_disc_evt(ble_dls_c_t* p_ble_dls_c, const ble_db_discovery_evt_t* p_evt);


/**@brief     Function for assigning handles to this instance of dls_c.
 *
 * @details Call this function when a link has been established with a peer to associate the link
 *          to this instance of the module. This makes it possible to handle several links and
 *          associate each link to a particular instance of this module.
 *
 * @param[in] p_ble_dls_c    Pointer to the Door Lock client structure instance for associating the link.
 * @param[in] conn_handle    Connection handle to associate with the given Door Lock Client Instance.
 * @param[in] p_peer_handles Door Lock Service handles found on the peer (from @ref BLE_DLS_C_EVT_DISCOVERY_COMPLETE event).
 *
 * @retval NRF_SUCCESS If the status was sent successfully.
 * @retval err_code    Otherwise, this API propagates the error code returned by function
 *                     @ref nrf_ble_gq_item_add.
 *
 */
uint32_t ble_dls_c_handles_assign(ble_dls_c_t*    p_ble_dls_c,
                                  uint16_t        conn_handle,
                                  const dls_db_t* p_peer_handles);


/**@brief Function for writing the Door Lock status to the connected server.
 *
 * @param[in] p_ble_dls_c Pointer to the Door Lock client structure.
 * @param[in] status      Door Lock status to send.
 *
 * @retval NRF_ERROR_INVALID_STATE  If the connection handle is invalid.
 * @retval err_code                 Otherwise, this API propagates the error code returned by function
 *                                  @ref nrf_ble_gq_item_add.
 */
uint32_t ble_dls_c_lock_state_send(ble_dls_c_t* p_ble_dls_c, uint8_t status);


/**@brief Function for reading the Door Lock status from the connected server.
 *
 * @param[in] p_ble_dls_c Pointer to the Door Lock client structure.
 *
 * @retval NRF_ERROR_INVALID_STATE  If the connection handle is invalid.
 * @retval err_code                 Otherwise, this API propagates the error code returned by function
 *                                  @ref nrf_ble_gq_item_add.
 */
uint32_t ble_dls_c_lock_state_get(ble_dls_c_t* p_ble_dls_c);


#ifdef __cplusplus
}
#endif

#endif // BLE_DLS_C_H__

/** @} */
