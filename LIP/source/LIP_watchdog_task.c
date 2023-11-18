/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides watchdog task which:
 *     - is always in the running state
 *     - is the default entry point for LIP controller application 
 *     - is used for protection functionality for cart max/min positions
 *           - set the cart zones based on current cart position
 *           - set dc motor voltage to zero when any track limit is reached (gpio pooling) 
 *     - is switching between controllers in swingup routine
 *       (swingup task - up position control task)
 * 
 * For safety reasons cart position zones were defined as:
 * FREEZING_ZONE_L    |      OK_ZONE             |      FREEZING_ZONE_R
 * (0cm - 4cm)	      |      (6cm - 34.7cm)      |      (36.7cm - 40.7cm)
 *
 * OK_ZONE           - Normal up/down controller working
 * DANGER_ZONE_L/R   - Control signal lowered
 * FREEZING_ZONE_L/R - Controller turned off
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "LIP_tasks_common.h"

/* Note: max cart run is 40.7cm */
#define FREEZING_ZONE_L_LOWER_LIMIT     0.0f
#define OK_ZONE_LOWER_LIMIT             6.0f
#define FREEZING_ZONE_R_LOWER_LIMIT     (40.07f - 6.0f)

/* Defined in LIP_tasks_common.c */
extern enum lip_app_states app_current_state;
extern float cart_position[ 2 ]; 
extern enum cart_position_zones cart_current_zone;
extern uint32_t bounceoff_resumed;

extern TaskHandle_t ctrl_3_FSF_downpos_task_handle;
extern TaskHandle_t bounceoff_task_handle;
extern TaskHandle_t swingup_task_handle;

extern uint32_t bounce_off_action_on;

/* Defined in LIP_tasks_common.c. This flag indicates that the swingup task is running. */
extern uint32_t swingup_task_resumed;

void watchdogTask( void * pvParameters )
{
    uint8_t MAX_POSITION_REACHED_h;
    uint8_t ZERO_POSITION_REACHED_h;

    /* For RTOS vTaskDelayUntil() */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* Set start app state to uninitialized. */
    app_current_state = UNINITIALIZED;

    for ( ;; )
    {
        /* Cart position protection functionality. */

        /* Set flags for cart position zones while in DPC or UPC states. */
        if( app_current_state == UPC || app_current_state == DPC || app_current_state == SWINGUP )
        {
            if( cart_position[ 0 ] < OK_ZONE_LOWER_LIMIT )
            {
                /* FREEZING_ZONE_L */
                cart_current_zone = FREEZING_ZONE_L;

                /* Turn off controllers tasks. */
                vTaskSuspend( ctrl_3_FSF_downpos_task_handle );
                // vTaskSuspend( #UPC );
                vTaskSuspend( swingup_task_handle );
                
                if( bounce_off_action_on )
                {
                    /* Resume bounce off task. */
                    if( ! bounceoff_resumed )
                    {
                        bounceoff_resumed = 1;
                        vTaskResume( bounceoff_task_handle );
                    }
                }
                else
                {
                    dcm_set_output_volatage( 0.0f );
                }

                /* Change app state back to DEFAULT. */
                app_current_state = DEFAULT;
            }
            else if( cart_position[ 0 ] > OK_ZONE_LOWER_LIMIT && cart_position[ 0 ] < FREEZING_ZONE_R_LOWER_LIMIT )
            {
                /* OK_ZONE */
                cart_current_zone = OK_ZONE;
            }
            else if( cart_position[ 0 ] > FREEZING_ZONE_R_LOWER_LIMIT )
            {
                /* FREEZING_ZONE_R */
                cart_current_zone = FREEZING_ZONE_R;

                /* Turn off controllers tasks. */
                vTaskSuspend( ctrl_3_FSF_downpos_task_handle );
                // vTaskSuspend( #UPC );
                vTaskSuspend( swingup_task_handle );

                if( bounce_off_action_on )
                {
                    /* Resume bounce off task. */
                    if( ! bounceoff_resumed )
                    {
                        bounceoff_resumed = 1;
                        vTaskResume( bounceoff_task_handle );
                    }
                }
                else
                {
                    dcm_set_output_volatage( 0.0f );
                }

                /* Change app state back to DEFAULT. */
                app_current_state = DEFAULT;
            }
        }
        
        /* Limit switches. */
        if( READ_ZERO_POSITION_REACHED )
        {
            /* Leftmost position reached (zero position). */

            ZERO_POSITION_REACHED_h = 1;
            MAX_POSITION_REACHED_h  = 0;

            /* Set output voltage to zero. */
            dcm_set_output_volatage( 0.0f );

            /* Leftmost switch was closed, zero cart position encoder. */
            dcm_enc_zero_counter();
        }
        else if( READ_MAX_POSITION_REACHED )
        {
            /* Rightmost position reached (max position). */

            MAX_POSITION_REACHED_h  = 1;
            ZERO_POSITION_REACHED_h = 0;
            
            /* Set output voltage to zero. */
            dcm_set_output_volatage( 0.0f );
        }


        MAX_POSITION_REACHED_h  = 0;
        ZERO_POSITION_REACHED_h = 0;

        // /* Switching from swingup to up position controller. */
        // if( swingup_task_resumed )
        // {

        // }

        /* Task delay */
        vTaskDelayUntil( &xLastWakeTime, dt_watchdog );
    }
}
