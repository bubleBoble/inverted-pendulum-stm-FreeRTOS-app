/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides task that is used for protection functionality for cart max/min 
 * positions, it is also the default entry point for LIP controller application and is 
 * always in the running state
 * 
 * Priority : 3 
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "LIP_tasks_common.h"

extern enum lip_app_states app_current_state;

/* from main_LIP.h, toggled in limit switch ISR. */
extern uint8_t MAX_POSITION_REACHED;
extern uint8_t ZERO_POSITION_REACHED;

void watchdogTask( void * pvParameters )
{
    /* For RTOS vTaskDelayUntil() */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* Set start app state to uninitialized. */
    app_current_state = UNINITIALIZED;

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

        if( MAX_POSITION_REACHED || ZERO_POSITION_REACHED )
        {
            dcm_set_output_volatage( 0.0f );
        }


        MAX_POSITION_REACHED = 0;
        ZERO_POSITION_REACHED = 0;

        /* Task delay */
        vTaskDelayUntil( &xLastWakeTime, dt_watchdog );
    }
}
