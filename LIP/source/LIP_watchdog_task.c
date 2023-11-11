/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides task that is used for protection functionality for cart max/min 
 * positions, it is also the default entry point for LIP controller application and is 
 * always in the running state
 * 
 * Priority : 3 
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "LIP_tasks_common.h"

void watchdogTask( void * pvParameters )
{
    /* For RTOS vTaskDelayUntil() */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* Action for "zero" cli command.
    If cart is not in min position, "zero" command moves it there. */
    // if ( !READ_ZERO_POSITION_REACHED )
    // {
    //     /* Start moving cart to the left if it's not already there.
    //     Left position has to be reached to reset cart position to zero. */
    //     dcm_set_output_volatage( -2.0f );
    // }

    for ( ;; )
    {
        /* Cart position protection functionality. */

        /* Task delay */
        vTaskDelayUntil( &xLastWakeTime, dt_watchdog );
    }
}
