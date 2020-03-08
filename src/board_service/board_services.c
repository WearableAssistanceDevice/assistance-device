#include "board_services.h"

#include "nordic_common.h"

#include "nrf.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "bsp.h"
#include "bsp_btn_ble.h"
#include "app_timer.h"
#include "fds.h"


// Board services config storage
static struct {
    bsp_evt_handler_t bsp_evt_handler;
} board_services_config;


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event) {
        default:
            break;
    }

    if (board_services_config.bsp_evt_handler != NULL) {
        board_services_config.bsp_evt_handler(event);
    }
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool* p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    if (p_erase_bonds != NULL)  {
        *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
    }
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing the board services.
 *
 * @param[in] p_init  Board services initialization config.
 */
void board_services_init(board_services_init_t* p_init) {
    if (p_init == NULL) {
        return;
    }

    board_services_config.bsp_evt_handler = p_init->bsp_evt_handler;

    log_init();
    timers_init();
    buttons_leds_init(p_init->erase_bonds);
    power_management_init();
}