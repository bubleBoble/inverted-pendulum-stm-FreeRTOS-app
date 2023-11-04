/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file contains task that implements full state feedback controller for
 * linear inverted pendulum. Controller keeps pendulum in down position  
 *
 * This task is used to:
 *     1. Read LIP state variables (defined as global), these are:
 *         state variable    |  variable name in prog  |  unit 
 *         ---------------------------------------------------------
 *         Cart position:    |  cart_position[ 0 ]     |  cm
 *         Cart speed:       |  cart_speed (filtered)  |  cm/sec
 *         Pendulum angle:   |  angle[ 0 ]             |  rad
 *         Pendulum speed:   |  pend_speed (filtered)  |  rad/sec
 * 
 *     2. Calculate control signal for inverted pendulum - dc motor voltage.
 *
 * This controller works with cart position and speed in meters and meters 
 * per second units, feedback gains are recalculated to work with these units 
 * 
 * This task runs every 10ms
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include "LIP_tasks_common.h"

/* These are defined in LIP_tasks_common.c */
extern volatile uint16_t adc_data_pot;
extern float pend_angle[ 2 ];
extern float pend_speed;
extern float cart_position[ 2 ];
extern float cart_speed;
extern float *cart_position_setpoint_cm;

void ctrlFSFTask( void *pvParameters )
{
    TickType_t xLastWakeTime = xTaskGetTickCount(); // For RTOS vTaskDelayUntil()

    /* Down position gains, u = F*(x_setpoint - x) 
    gains[0] - cart position error gain, units: V/cm
    gains[1] - pend angle error gain,    units: V/rad
    gains[2] - cart speed error gain,    units: Vs/cm 
    gains[3] - pend speed error gain,    units: Vs/rad */
    float gains[4] = {44.721360f, 20.131541f, 5.820552f, -0.529622f};
    gains[0] = gains[0] * 0.01f; // from V/m to V/cm
    gains[2] = gains[2] * 0.01f; // from V/m/s to V/cm/s
    
    /* Allowed error for cart position in centimeters (cm).
    There will always be some steady state error becouse of
    the presence of voltage deadzone in real pendulum. This
    values are set as they are set in matlab simulation. */
    float cart_position_allowed_error_cm = 1.0f;
    float pend_position_allowed_error    = 1.0f*PI/180.0f;
    float voltage_deadzone               = 1.0f;

    float ctrl_signal = 0;
    float pend_angle_setpoint = PI;

    float cart_position_error = 0;
    float cart_speed_error    = 0;
    float pend_position_error = 0;
    float pend_speed_error    = 0; 

    for( ;; )
    {
        cart_position_error = *cart_position_setpoint_cm - cart_position[0]; 
        cart_speed_error    = - cart_speed;
        pend_position_error = pend_angle_setpoint - pend_angle[ 0 ];
        pend_speed_error    = - pend_speed;

        /* controller update
        control signal = ( state_setpoint - state ) * ( F ) */
        ctrl_signal = ( cart_position_error )       * gains[0] +
                      ( pend_position_error )       * gains[1] +
                      ( cart_speed_error )          * gains[2] +
                      ( pend_speed_error )          * gains[3];

        /* Deadzone for control signal in terms of cart position error */
        if ( (cart_position_error < cart_position_allowed_error_cm)     && 
             (cart_position_error > - cart_position_allowed_error_cm)   && 
             (pend_position_error < pend_position_allowed_error)        &&
             (pend_position_error > - pend_position_allowed_error))
        {
            ctrl_signal = 0.0f;
        }

        /* Deadzone for control signal,
        when cart speed is zero, voltage in range [-a, a] doesnt make the cart move. */
        if( (ctrl_signal < voltage_deadzone) && (ctrl_signal > -voltage_deadzone) )
        {
            ctrl_signal = 0.0f;
        }
        
        /* Additional ctrl signal saturation - for testing */    
        else if ( ctrl_signal > 12.0f  )
        {
            ctrl_signal = 12.0f;
        }
        else if ( ctrl_signal < -12.0f )
        {
            ctrl_signal = -12.0f;
        }

        /* Set calculated output voltage */
        dcm_set_output_volatage( ctrl_signal );

        /* Task delay */
        vTaskDelayUntil( &xLastWakeTime, dt );
    }

}



