/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides task that is only active when command "zero" or "home" is called.
 * Its purpose is to move the cart without any controller.
 * 
 * Priority : 3 
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "LIP_tasks_common.h"

// extern enum lip_app_states app_current_state;

void workerTask( void * pvParameters )
{
    /* For RTOS vTaskDelayUntil() */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for ( ;; )
    {

        /* Task delay */
        vTaskDelayUntil( &xLastWakeTime, dt_worker );
    }
}
