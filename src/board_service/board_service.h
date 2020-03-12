#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief BSP event handler type */
typedef void (*bsp_evt_handler_t)(bsp_event_t event);

/**@brief Board services init structure */
typedef struct {
    bsp_evt_handler_t bsp_evt_handler;
} board_services_init_t;



/**@brief Function for initializing the board services.
 *
 * @param[in] p_init  Board services initialization config.
 */
void board_services_init(const board_services_init_t* p_init);


#ifdef __cplusplus
}
#endif
