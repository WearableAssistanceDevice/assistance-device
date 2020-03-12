#include "board_service.h"

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


static struct {
    bsp_evt_handler_t bsp_evt_handler;
} board_services_config;


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    if (board_services_config.bsp_evt_handler != NULL) {
        board_services_config.bsp_evt_handler(event);
    }
}


/**@brief Function for initializing the LED and button handler module.
 */
static void leds_buttons_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the log.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Power manager. */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


void board_services_init(const board_services_init_t* p_init) {
    if (p_init == NULL) {
        return;
    }

    board_services_config.bsp_evt_handler = p_init->bsp_evt_handler;

    log_init();
    timer_init();
    leds_buttons_init();
    power_management_init();
}