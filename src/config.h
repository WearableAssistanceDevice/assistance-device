#pragma once

#include "app_util.h"
#include "bsp.h"

// Device Config
//--------------------------------------------------------------------------------
#define DEVICE_NAME                     "Assistance Wearable"               /**< The advertised name of the device */

#define SCAN_INTERVAL                   0x00A0                              /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     0x0050                              /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION                   0x0000                              /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define SEC_PARAM_BOND                  1                                   /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                   /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                   /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                   /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                   /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                   /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                  /**< Maximum encryption key size. */

#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                 /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */


// App Config
//--------------------------------------------------------------------------------
#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0                     /**< Scanning LED will be on when the device is scanning. */
#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_1                     /**< Connected LED will be on when the device is connected. */
#define ASSISTANCE_REQUEST_LED          BSP_BOARD_LED_3                     /**< Assistance Request LED will be on when the assistance request button is pressed, until the server acknowledges the request */

#define ASSIST_REQ_KEY_EVENT            BSP_EVENT_KEY_0                     /**< The button to press in order to make an assistance request */
#define DOOR_UNLOCK_KEY_EVENT           BSP_EVENT_KEY_1                     /**< The button to press in order to unlock a door */
#define BOND_KEY_EVENT                  BSP_EVENT_KEY_3                     /**< The button to press in order to bond to the currently connected device */

#define SERVER_PERIPH_NAME              "Assistance_Server"                 /**< Name of the assistance server. This name is searched in the scan report data*/
#define DOOR_PERIPH_NAME                "BLE_Door"                          /**< Name of the door lock. This name is searched in the scan report data*/