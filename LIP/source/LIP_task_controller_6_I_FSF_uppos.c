/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file contains task that implements full state feedback controller for
 * linear inverted pendulum. Controller tries to balance pendulum in 
 * down position, tanh switing for voltage deadzone compensation.
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
#include "math.h"

/* These are defined in LIP_tasks_common.c */
extern volatile uint16_t adc_data_pot;
extern float pend_angle[ 2 ];
extern float pend_speed[ 2 ];
extern float cart_position[ 2 ];
extern float cart_speed[ 2 ];
extern float *cart_position_setpoint_cm;
extern enum cart_position_zones cart_current_zone;
extern float number_of_pendulumarm_revolutions_upc;
extern float pendulum_angle_in_base_range_upc;
extern float pendulum_arm_angle_setpoint_rad_upc;

#ifdef COM_SEND_CTRL_DEBUG
    extern float ctrl_xw;
    extern float ctrl_th;
    extern float ctrl_Dx;
    extern float ctrl_Dt;
#endif

void ctrl_6_I_FSF_uppos_task( void *pvParameters )
{
    /* For RTOS vTaskDelayUntil() */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* Controller should turn on only if the angle is in range [switch_angle_low, switch_angle_high]. */
    float switch_angle_low  = -27.0f * PI / 180.0f;    // lower boundry in radians
    float switch_angle_high =  27.0f * PI / 180.0f;    // upper boundry in radians

    /* Down position gains, u = F*(x_setpoint - x) 
    gains[0] - cart position error gain, units: V/cm
    gains[1] - pend angle error gain,    units: V/rad
    gains[2] - cart speed error gain,    units: Vs/cm 
    gains[3] - pend speed error gain,    units: Vs/rad */

    /* Gains from first test iteration. There is about 2cm error in cart position. */
    float gains[ 5 ] = {-70.710678, -76.351277, -50.892920, -9.096002, -0.1f};
    gains[ 0 ] = gains[ 0 ] * 0.01f; // from V/m to V/cm
    gains[ 2 ] = gains[ 2 ] * 0.01f; // from V/m/s to V/cm/s
    gains[ 4 ] = gains[ 4 ] * 0.01f;

    /* Allowed error for cart position in centimeters (cm).
    There will always be some steady state error becouse of
    the presence of voltage deadzone in real pendulum. This
    values are set as they are set in matlab simulation. */
    // float cart_position_allowed_error_cm = 1.0f;
    // float pend_position_allowed_error    = 1.0f * PI/180.0f;
    float voltage_deadzone = 1.0f;

    float ctrl_signal = 0.0f;

    float cart_position_error[ 2 ]     = { 0.0f }; // This error will be integrated, need two samples
    float cart_speed_error             =   0.0f;
    float pend_position_error          =   0.0f;
    float pend_speed_error             =   0.0f; 
    float cart_position_error_int[ 2 ] = { 0.0f }; // Integral of cart_position_error

    /* These variables sum to final control signal. */
    float ctrl_cart_position_error = 0.0f;
    float ctrl_pend_angle_error    = 0.0f;
    float ctrl_cart_speed_error    = 0.0f;
    float ctrl_pend_speed_error    = 0.0f;
    float ctrl_cart_position_error_int =  0.0f;
    
    /* Will be set to sum of last two cart position error samples. */
    float sum_cart_position_error    =  0.0f; // for cart position error integaration

    for( ;; )
    {
        /* Note: this angle switching range is different from switching angle range from swingup to upc, set up
        if watchdog task. */
        if( switch_angle_low < pendulum_angle_in_base_range_upc && switch_angle_high > pendulum_angle_in_base_range_upc )
        {
            /* Controller should only work when pendulum arm angle is in range [switch_angle_low, switch_angle_high]. */

            /* Calculate state varialbes errors */
            cart_position_error[ 1 ] = cart_position_error[ 0 ];
            cart_position_error[ 0 ] =  *cart_position_setpoint_cm - cart_position[0]; 
            cart_speed_error         = - cart_speed[ 0 ];
            pend_position_error      =   pendulum_arm_angle_setpoint_rad_upc - pend_angle[ 0 ];
            pend_speed_error         = - pend_speed[ 0 ];
          
            /* Integrate cart position error using Tustin method:
            y[n] = y[n-1] + T/2 * ( u[n] + u[n-1] ),
            constant "dt" is in ms so multiply by 0.001 to convert units to seconds */
            sum_cart_position_error = cart_position_error[ 0 ] + cart_position_error[ 1 ];
            cart_position_error_int[ 1 ] = cart_position_error_int[ 0 ];
            cart_position_error_int[ 0 ] = cart_position_error_int[ 1 ] + dt*0.001/2 * ( sum_cart_position_error );

            /* Calculate control signal contribution of each state variable error */
            /* Deadzone protection nr. 3 "tanh switching" : non linear cart position gain.
            When cart postion error is >0 linear feedback with offset +1V is used
            to compensate for voltage deadzone, for <0 error, y-axis mirror is used.
            graph: https://www.desmos.com/calculator/ycgnqpyy9y */
            /* Default cart position error gain is gains[0] */
            if( cart_position_error > 0 )
            {
                ctrl_cart_position_error =   tanhf( 7.0f * cart_position_error[ 0 ] ) * ( gains[ 0 ] * cart_position_error[ 0 ] + voltage_deadzone );
                // if( ctrl_cart_position_error < voltage_deadzone )
                // {
                //     ctrl_cart_position_error = 0.0f;
                // }
            } 
            else if( cart_position_error < - 0 )
            {
                ctrl_cart_position_error = - tanhf( 7.0f * cart_position_error[ 0 ] ) * ( gains[ 0 ] * cart_position_error[ 0 ] - voltage_deadzone );
                // if( ctrl_cart_position_error > -voltage_deadzone )
                // {
                //     ctrl_cart_position_error = 0.0f;
                // }
            }
            // ctrl_cart_position_error = cart_position_error   * gains[ 0 ];
            ctrl_pend_angle_error           = pend_position_error           * gains[ 1 ];
            ctrl_cart_speed_error           = cart_speed_error              * gains[ 2 ];
            ctrl_pend_speed_error           = pend_speed_error              * gains[ 3 ];
            ctrl_cart_position_error_int    = cart_position_error_int[ 0 ]  * gains[ 4 ];

            #ifdef COM_SEND_CTRL_DEBUG
                ctrl_xw = ctrl_cart_position_error;
                ctrl_th = ctrl_pend_angle_error;
                ctrl_Dx = ctrl_cart_speed_error;
                ctrl_Dt = ctrl_pend_speed_error;
            #endif

            /* Regular controller update - with constant gains
            control signal = ( state_setpoint - state ) * ( F ) */
            ctrl_signal = ctrl_cart_position_error + 
                          ctrl_pend_angle_error    + 
                          ctrl_cart_speed_error    + 
                          ctrl_pend_speed_error    +
                          ctrl_cart_position_error_int;
        
            /* Set calculated output voltage / control signal. */
            dcm_set_output_volatage( ctrl_signal );
        }
        else
        {
            /* Angle not in specified range, output zero voltage. */
            dcm_set_output_volatage( 0.0f );
        }

        /* Task delay */
        vTaskDelayUntil( &xLastWakeTime, dt );
    }

}
