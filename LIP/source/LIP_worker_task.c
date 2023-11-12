/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides task that is only active when command "zero" or "home" is called.
 * Its purpose is to move the cart without any controller.
 * 
 * Priority : 3 
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "LIP_tasks_common.h"
#include "limits.h"
// extern enum lip_app_states app_current_state;

void workerTask( void * pvParameters )
{
    /* For RTOS vTaskDelayUntil() */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    uint32_t notif_value_recived;

    for ( ;; )
    {
        /* Wait for notification at index 0. */
        xTaskNotifyWait( 0,                     /* Bits to clear on entry (before save to third arg). */ 
                         ULONG_MAX,             /* Bits to clear on exit (after save to third arg). */
                         &notif_value_recived,  /* Value of received notification. Can be set to NULL if not used. */
                         portMAX_DELAY );       /* Block time (while waiting for notification). */

        if( ( notif_value_recived & 0x01 ) != 0 )
        {
            /* Bit 0 was set (first bit)- process whichever event is represented by bit 0. */
        }
        if ( ( notif_value_recived & 0x02 ) != 0 )
        {
            /* Bit 1 was set (second bit) - process whichever event is represented by bit 1. */
        }
        if ( ( notif_value_recived & 0x04 ) != 0 )
        {
            /* Bit 2 was set (third bit) - process whichever event is represented by bit 2. */
        }

        /* Task delay */
        vTaskDelayUntil( &xLastWakeTime, dt_worker );
    }
}
