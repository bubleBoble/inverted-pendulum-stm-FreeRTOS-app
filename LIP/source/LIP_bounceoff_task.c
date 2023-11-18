/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Basically, when cart is in freezing zone (see watchdog task), app should 
 * instantly stop it. Setting motor voltage to zero is not enough becouse of cart 
 * inertia - it keeps moving for a little bit. This task is resumed when the cart 
 * is present in freezing zone and should perform hard break operation, which
 * means that cart should instantly stop. It can't be implemented directly in 
 * watchdog task where current cart zone is indicated because it have other
 * security related stuff to do. 
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include "LIP_tasks_common.h"
#include "math.h"

// extern float cart_position[ 2 ];
/* Defined in LIP_tasks_common.c, set in watchdog task. */
extern enum cart_position_zones cart_current_zone;

extern uint32_t bounceoff_resumed;

float voltage_step;

#define N_STEPS_UP   10

void bounceoff_task( void *pvParameters )
{
    for( ;; )
    {
        if( cart_current_zone == FREEZING_ZONE_R )
        {
            dcm_set_output_volatage( 0.0f );

            voltage_step = 12.0f / N_STEPS_UP;
            for( int32_t i=0; i<N_STEPS_UP; i++ )
            {
                dcm_set_output_volatage( -i*voltage_step );
                vTaskDelay( 30 );
            }
            dcm_set_output_volatage( 0.0f );
        }
        else if( cart_current_zone == FREEZING_ZONE_L )
        {
            dcm_set_output_volatage( 0.0f );

            voltage_step = 12.0f / N_STEPS_UP;
            for( int32_t i=0; i<N_STEPS_UP; i++ )
            {
                dcm_set_output_volatage( i*voltage_step );
                vTaskDelay( 30 );
            }
            dcm_set_output_volatage( 0.0f );
        }

        bounceoff_resumed = 0;
        /* Suspend this task. */
        vTaskSuspend( NULL );
    }
}



