/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides task that pendulum swingup routine. This task should run 
 * with sampling period of 10ms.
 * 
 * Lookup table for swingup input voltage is written 
 * in swingup_input_voltage_lookup_table.c.
 * 
 * Note: To start swingup routine cart should be in position of about 0.2 meters. Task
 * should be run every 10ms and swingup routine shouldn't take longer than 3 seconds.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include "LIP_tasks_common.h"

/* Voltage lookup table for swingup,
defined in swingup_input_voltage_lookup_table.c. */
extern float swingup_control[ 301 ]; 

/* These are defined in LIP_tasks_common.c */
extern float pend_angle[ 2 ];
extern float pend_speed[ 2 ];
extern float cart_position[ 2 ];
extern float cart_speed[ 2 ];
extern float *cart_position_setpoint_cm;
extern float pendulum_arm_angle_setpoint_rad;
extern enum cart_position_zones cart_current_zone;

/* Keeps track of current app state, defined in LIP_tasks_common.c */
extern enum lip_app_states app_current_state;

/* Defined in LIP_tasks_common.c. This flag indicates that the swingup task is running. */
extern uint32_t swingup_task_resumed;

void swingup_task( void *pvParameters )
{
    /* For RTOS vTaskDelayUntil(). */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* Index for swingup_control lookup table. */
    uint32_t lookup_index = 0;

    for( ;; )
    {
        if( lookup_index < 301 )
        {
            xLastWakeTime = xTaskGetTickCount();
            /* Use the values from swingup_control lookup table to set
            dc motor voltage. */
            dcm_set_output_volatage( swingup_control[ lookup_index ] );
            lookup_index++;

            /* 10ms task delay when swingup is running. */
            vTaskDelayUntil( &xLastWakeTime, dt_swingup );
        }
        else
        {
            /* No more data in lookup table.
            set zero voltage. */
            dcm_set_output_volatage( 0.0f );
            
            /* Reset lookup_index so that it can be used later again. */
            lookup_index = 0;

            /* If this task wasn't suspended ealier it means that up position controller didn't take over control 
            and app is in DEFAULT state. */
            app_current_state = DEFAULT;

            /* Suspend this task. */
            vTaskSuspend( NULL );
        }
    }
}



