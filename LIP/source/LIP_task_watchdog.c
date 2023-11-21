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
#include <math.h>

/* Note: max cart run is 40.7cm */
#define FREEZING_ZONE_L_LOWER_LIMIT     0.0f
#define OK_ZONE_LOWER_LIMIT             3.0f
#define FREEZING_ZONE_R_LOWER_LIMIT     (40.07f - OK_ZONE_LOWER_LIMIT)

/* Globals defined in LIP_tasks_common.c */
extern float cart_position[ 2 ];
extern float pend_angle[ 2 ];
extern enum lip_app_states app_current_state;
extern float cart_position[ 2 ]; 
extern enum cart_position_zones cart_current_zone;
extern uint32_t bounceoff_resumed;
extern float number_of_pendulumarm_revolutions_dpc;
extern float pendulum_angle_in_base_range_dpc;
extern float number_of_pendulumarm_revolutions_upc;
extern float pendulum_angle_in_base_range_upc;
extern float pendulum_arm_angle_setpoint_rad_dpc;
extern float pendulum_arm_angle_setpoint_rad_upc;

extern uint32_t bounce_off_action_on;
extern uint32_t swingup_task_resumed;

extern TaskHandle_t ctrl_3_FSF_downpos_task_handle;
extern TaskHandle_t bounceoff_task_handle;
extern TaskHandle_t swingup_task_handle;
extern TaskHandle_t ctrl_5_FSF_uppos_task_handle;


void watchdogTask( void * pvParameters )
{
    // uint8_t MAX_POSITION_REACHED_h;
    // uint8_t ZERO_POSITION_REACHED_h;

    /* For RTOS vTaskDelayUntil() */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* Set start app state to uninitialized. */
    app_current_state = UNINITIALIZED;

    for ( ;; )
    {
        /* Cart position protection functionality. */

        /* Set flags for cart position zones while in DPC or UPC states.
        Perform cart bounceoff (if enabled) or freeze in danger zone (if bounceoff disabled). */
        if( app_current_state == UPC || app_current_state == DPC )
        {
            if( cart_position[ 0 ] < OK_ZONE_LOWER_LIMIT )
            {
                /* FREEZING_ZONE_L */
                cart_current_zone = FREEZING_ZONE_L;

                /* Suspend controller tasks. */
                vTaskSuspend( ctrl_3_FSF_downpos_task_handle );
                vTaskSuspend( ctrl_5_FSF_uppos_task_handle );
                // vTaskSuspend( swingup_task_handle );

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

                /* UPC or DPC controller was on, this means that app was already initialized (in default state).
                Change app state back to default. */
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

                /* Suspend controller tasks. */
                vTaskSuspend( ctrl_3_FSF_downpos_task_handle );
                vTaskSuspend( ctrl_5_FSF_uppos_task_handle );
                // vTaskSuspend( swingup_task_handle );

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

                /* UPC or DPC controller was on, this means that app was already initialized (in default state).
                Change app state back to default. */
                app_current_state = DEFAULT;
            }
        }
        
        /* Limit switches. */
        if( READ_ZERO_POSITION_REACHED )
        {
            /* Leftmost position reached (zero position). */

            // ZERO_POSITION_REACHED_h = 1;
            // MAX_POSITION_REACHED_h  = 0;

            /* Set output voltage to zero. */
            dcm_set_output_volatage( 0.0f );

            /* Suspend controller tasks. */
            vTaskSuspend( ctrl_3_FSF_downpos_task_handle );
            vTaskSuspend( ctrl_5_FSF_uppos_task_handle );

            /* Set output voltage to zero again in case any controller task 
            managed to set any output voltage. */
            dcm_set_output_volatage( 0.0f );

            /* Leftmost switch was closed, zero cart position encoder. */
            dcm_enc_zero_counter();

            /* UPC or DPC controller was on, this means that app was already initialized (in default state).
            Change app state back to default. */
            app_current_state = DEFAULT;
        }
        else if( READ_MAX_POSITION_REACHED )
        {
            /* Rightmost position reached (max position). */

            // MAX_POSITION_REACHED_h  = 1;
            // ZERO_POSITION_REACHED_h = 0;
            
            /* Set output voltage to zero. */
            dcm_set_output_volatage( 0.0f );

            /* Suspend controller tasks. */
            vTaskSuspend( ctrl_3_FSF_downpos_task_handle );
            vTaskSuspend( ctrl_5_FSF_uppos_task_handle );

            /* Set output voltage to zero again in case any controller task 
            managed to set any output voltage. */
            dcm_set_output_volatage( 0.0f );

            /* UPC or DPC controller was on, this means that app was already initialized (in default state).
            Change app state back to default. */
            if( app_current_state != UNINITIALIZED )
            {
                app_current_state = DEFAULT;
            }
        }

        // MAX_POSITION_REACHED_h  = 0;
        // ZERO_POSITION_REACHED_h = 0;

        /* SWINGUP to UPC hard switching. */
        if( app_current_state == SWINGUP )
        {
            /* App is in swingup state. */
            
            if( fabsf( pendulum_arm_angle_setpoint_rad_upc - pend_angle[ 0 ] ) < 40.0f*PI/180.0f ) 
            {
                /* Pendulum angle error is within pm. 25 degrees from up position. */

                /* Suspend swingup task. */
                vTaskSuspend( swingup_task_handle );
                // app_current_state = DEFAULT;
                /* Resume UPC. */
                vTaskResume( ctrl_5_FSF_uppos_task_handle );    
                
                /* Change app state to UPC. */
                app_current_state = UPC;
            }
        }

        /* Task delay */
        vTaskDelayUntil( &xLastWakeTime, dt_watchdog );
    }
}
