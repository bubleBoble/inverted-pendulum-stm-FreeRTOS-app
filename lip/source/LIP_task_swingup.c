/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides task that pendulum swingup routine. This task should run
 * with sampling period of 10ms.
 *
 * Lookup table for swingup input voltage is saved
 * in swingup_input_voltage_lookup_table.c.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include "LIP_tasks_common.h"
#include <math.h>

/* These are defined in LIP_tasks_common.c */
extern float pend_angle[ 2 ];
extern float pend_speed[ 2 ];
extern float cart_position[ 2 ];
extern float cart_speed[ 2 ];
extern float *cart_position_setpoint_cm;
extern float pendulum_arm_angle_setpoint_rad;
extern enum cart_position_zones cart_current_zone;
extern uint32_t reset_lookup_index;
extern float cart_position_setpoint_cm_cli_raw;

/* Keeps track of current app state, defined in LIP_tasks_common.c */
extern enum lip_app_states app_current_state;

/* Defined in LIP_tasks_common.c. This flag indicates that the swingup task is running. */
extern uint32_t swingup_task_resumed;

/* Down position controller stask handle. */
extern TaskHandle_t ctrl_downposition_task_handle;

/* Voltage lookup tables for swingup. Comment/uncomment one or the other. */
/* swingup_control_1 lookup table. */
// #define SWINGUP_START_POSITION 20.0f
// #define N_LOOKUP_SAMPLES 300
// #define LOOKUP_TABLE swingup_control_1
// extern float LOOKUP_TABLE[ 301 ];

/* swingup_control_2 lookup table. */
#define SWINGUP_START_POSITION 11.0f
#define N_LOOKUP_SAMPLES 220
// #define N_LOOKUP_SAMPLES 180
#define LOOKUP_TABLE swingup_control_2
extern float LOOKUP_TABLE[ 220 ];

void swingup_task( void *pvParameters )
{
    /* For RTOS vTaskDelayUntil(). */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* Index for swingup_control lookup table. */
    uint32_t lookup_index = 0;

    // char msg[128];

    for( ;; )
    {
        xLastWakeTime = xTaskGetTickCount();

        /* APP HAS TO BE IN DEFAULT STATE - Cart position already calibrated. */

        for( lookup_index = 0; lookup_index < N_LOOKUP_SAMPLES; lookup_index++ )
        {
            /* This task can be suspended any time while in this for loop. */

            if( reset_lookup_index )
            {
                /* This procedure runs on every call to "swingup" command. */

                /* Global reset lookup index variable was set to 1,
                this means that command "swingup" was called and lookup_index should be set to zero. */
                lookup_index = 0;
                reset_lookup_index = 0;

                /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
                /* Change to DPC state. */
                vTaskResume( ctrl_downposition_task_handle );
                // app_current_state = DPC;
                // com_send( "\r\nDPC ON\r\n", 10 );
                
                /* Change DPC setpoint to necessary swingup cart start position. */
                cart_position_setpoint_cm_cli_raw = SWINGUP_START_POSITION;
                // sprintf( msg, "\r\nSETPOINT CHANGED TO: %f\r\n", ( double ) SWINGUP_START_POSITION );
                // com_send( msg, strlen(msg) );

                /* Wait for 3 seconds - should be enough for cart to reach SWINGUP_START_POSITION. */
                vTaskDelay( 3000 );
                vTaskSuspend( ctrl_downposition_task_handle );
                app_current_state = SWINGUP;

                // com_send( "\r\nswingup in: 3.\r\n",  18 );
                // vTaskDelay( 1000 );
                // com_send(     "swingup in: 2.\r\n",  16 );
                // vTaskDelay( 1000 );
                // com_send(     "swingup in: 1.\r\n",  16 );
                // vTaskDelay( 1000 );
                // com_send(     "swingup in: 0.\r\n",  16 );
                /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

                /* Reset task last wake time, so that when this task resumed, timing works properly. */
                xLastWakeTime = xTaskGetTickCount();
            }

            /* Use the values from swingup_control lookup table to set
            dc motor voltage. */
            dcm_set_output_volatage( LOOKUP_TABLE[ lookup_index ] );

            /* Delay for 10ms exacly. */
            vTaskDelayUntil( &xLastWakeTime, dt_swingup );
            // vTaskDelay( dt_swingup );
        }

    /* No more data in the lookup table, set zero voltage. */
    dcm_set_output_volatage( 0.0f );

    /* If this task wasn't suspended ealier it means that up position controller didn't take over control,
    and app should remain in DEFAULT state. */
    app_current_state = DEFAULT;

    /* Suspend this task. */
    vTaskSuspend( NULL );
    } // for( ;; ){ ... }
}



