/** @file
 *
 * @defgroup ble_wearable_assist_device main.c
 * @{
 * @ingroup ble_wearable_assist_device
 * @brief Wearable Assistance Device main file.
 *
 * DESCRIPTION HERE
 */

#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "board_service/board_services.h"
#include "ble_service/ble_services.h"
#include "ble_service/ble_assist_service.h"


#define DEAD_BEEF 0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}



/**@brief Function for creating timers.
 */
static void user_timers_init(void) {
    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       ret_code_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */

}


/**@brief User function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
void bsp_event_handler(bsp_event_t event) {
    ble_bsp_evt_handler(event);
    ble_assist_bsp_evt_handler(event);
}


/**@brief User function for handling BLE advertising events.
 *
 * @param[in]   ble_adv_evt   Advertising event.
 */
void ble_adv_evt_handler(ble_adv_evt_t ble_adv_evt) {
    switch (ble_adv_evt) {
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief User function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
void ble_evt_handler(const ble_evt_t* p_ble_evt, void* p_context) {
    return;
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Board services config
    board_services_init_t board_init = {0};
    board_init.bsp_evt_handler = bsp_event_handler;
    board_init.erase_bonds = &erase_bonds;

    // BLE services config
    ble_service_init_func_t init_funcs[] = { ble_assist_service_init };

    ble_services_init_t ble_init = {0};
    ble_init.adv_evt_handler = ble_adv_evt_handler;
    ble_init.ble_evt_handler = ble_evt_handler;
    ble_init.service_init_funcs = init_funcs;
    ble_init.service_init_func_count = sizeof(init_funcs) / sizeof(init_funcs[0]);

    // Initialize
    board_services_init(&board_init);
    ble_services_init(&ble_init);
    user_timers_init();

    // Start execution
    NRF_LOG_INFO("Template example started.");
    application_timers_start();

    advertising_start(erase_bonds);

    // Enter main loop
    for (;;) {
        idle_state_handle();
    }
}


/**
 * @}
 */
