/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides task that is only active when command "zero" or "home" is called.
 * Its purpose is to move the cart without any controller.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "LIP_tasks_common.h"
#include "limits.h"

/* Voltage value used to move cart without any controller. */
#define IDLE_MOVEMENTS_VOLTAGE 2.0f

/* Keeps track of current app state. */
extern enum lip_app_states app_current_state;

/* Global cart position variable, defined in LIP_tasks_common.c */
extern float cart_position[ 2 ];

/* Cart position setpoint, independent of currenlty selected setpoint source. 
defined in LIP_tasks_common.c */
extern float *cart_position_setpoint_cm;
extern float cart_position_setpoint_cm_cli_raw;

void cartWorkerTask( void * pvParameters )
{
    /* Hold value retrieved from task notification. */
    uint32_t notif_value_received;

    for ( ;; )
    {   
        /* Wait for notification at index 0. */
        xTaskNotifyWaitIndexed( 0,                     /* Notification index */
                                0x00,                  /* Bits to clear on entry (before save to third arg). */ 
                                ULONG_MAX,             /* Bits to clear on exit (after save to). */
                                &notif_value_received, /* Value of received notification. Can be set to NULL if not used. */
                                portMAX_DELAY );       /* Block time (while waiting for notification). */
                
        /* notif_value_received possible values: GO_RIGHT, GO_RIGHT+GO_LEFT, GO_LEFT, SP_HOME
           cart_position_calibrated*/
        if( notif_value_received == GO_RIGHT )
        {
            /* App is uninitialized state and cart is already in zero position OR
            app is in default state (cart position calibrated). 
            Move cart to the right until track center reached. */
            dcm_set_output_volatage( IDLE_MOVEMENTS_VOLTAGE );
            while( cart_position[ 0 ] < TRACK_LEN_MAX_CM/2 * 0.98f )
            {
                vTaskDelay( dt_cartworker );
            }
            dcm_set_output_volatage( 0.0f );

            /* Change app state from UNINITIALIZED to DEFAULT right after zero position is reached. */
            app_current_state = DEFAULT;
        }
        else if ( notif_value_received == GO_LEFT )
        {
            /* App is in default state(cart position calibrated).
            Move cart to the left until center position reached. */
            dcm_set_output_volatage( -IDLE_MOVEMENTS_VOLTAGE );
            while( cart_position[ 0 ] > TRACK_LEN_MAX_CM/2 * 1.02f )
            {
                vTaskDelay( dt_cartworker );
            }
            dcm_set_output_volatage( 0.0f );
        }
        else if( notif_value_received == GO_LEFT+GO_RIGHT )
        {
            // com_send( "\r\ngowno nie dziala\r\n", 20 ); // message used for debugging, in polish it means: "this shit doesn't work"
            /* App is in uninitialized state and cart position has to be calibrated. */
            dcm_set_output_volatage( -IDLE_MOVEMENTS_VOLTAGE );
            while( ! READ_ZERO_POSITION_REACHED )
            {
                /* Go left until zero position reached. */
                vTaskDelay( dt_cartworker );
                // vTaskDelay( 10 );
            }
            app_current_state = DEFAULT;
            dcm_set_output_volatage( IDLE_MOVEMENTS_VOLTAGE );
            while( cart_position[ 0 ] < TRACK_LEN_MAX_CM/2.0f )
            {
                /* Go to track center. */
                vTaskDelay( dt_cartworker );
                // vTaskDelay( 10 );
            }
            dcm_set_output_volatage( 0.0f );
        }
        else if( notif_value_received == SP_HOME )
        {
            /* App is in UPC or DPC state. Change setpoint to home postion. Write new setpoint to
            _raw cli setpoint (unfiltered). cart_position_setpoint_cm_cli_raw acts as input to 
            cart_position_setpoint_cm_cli low-pass filter. Low-pass filter is used for both setpoint 
            sources to smooth out discontinous input. */
            cart_position_setpoint_cm_cli_raw = TRACK_LEN_MAX_CM/2.0f;
        }

        /* Task delay */
        /* This task wont run all the time, so vTaskDelayUntil can't be used. */
        vTaskDelay( dt_cartworker );
    }
}
