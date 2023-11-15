/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file contains task that implements full state feedback controller for
 * linear inverted pendulum. Controller keeps pendulum in down position.
 * 
 * This implementation tries to compensate for voltage deadzone with "tanh switching"
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
extern float pendulum_arm_angle_setpoint_rad;

/* Angle setpoint for pendulum arm. Down position corresponds to 180 degrees or pi radians.
Controller works with angle in radians. "BASE" postfix indicates that this setpoint is from
base angle range [0, 2PI]. Because pendulum arm can make many full revolutions, 
angles PI, 3PI, 5PI and so on, all correspond to the same down position, angle setpoint needs
to be changed accordingly. */
#define PENDULUM_ANGLE_DOWN_SETPOINT_BASE PI

void ctrl_3_FSF_downpos_task( void *pvParameters )
{
    /* For RTOS vTaskDelayUntil() */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* Holds number of pendulum full revolutions, negative number indicates
    full revolution in counter clockwise direction. */
    int32_t number_of_pendulumarm_revolutions = 0;

    /* Holds pendulum arm angle in base range [0 2pi]. */
    float pendulum_angle_in_base_range = 0.0f;

    /* Controller should turn on only if the angle is in range [switch_angle_low, switch_angle_high]. */
    float switch_angle_low  = 120.0f * PI / 180.0f;    // lower boundry in radians
    float switch_angle_high = 240.0f * PI / 180.0f;    // upper boundry in radians

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
    float cart_position_allowed_error_cm = 0.2f;
    float pend_position_allowed_error    = 3.0f * PI/180.0f;
    float voltage_deadzone = 0.75f;

    float ctrl_signal = 0.0f;
    float pend_angle_setpoint = PI;

    float cart_position_error = 0.0f;
    float cart_speed_error    = 0.0f;
    float pend_position_error = 0.0f;
    float pend_speed_error    = 0.0f; 

    /* These variables sum to final control signal */
    float ctrl_cart_position_error = 0.0f;
    float ctrl_pend_angle_error    = 0.0f;
    float ctrl_cart_speed_error    = 0.0f;
    float ctrl_pend_speed_error    = 0.0f;

    for( ;; )
    {
        /* Get number of revolutions. */
        number_of_pendulumarm_revolutions = get_num_of_revolutions();

        /* Calculate pendulum arm angle in base range [0 2pi]. */
        pendulum_angle_in_base_range = pend_angle[ 0 ] - ( float ) number_of_pendulumarm_revolutions * PI2;

        if( switch_angle_low < pendulum_angle_in_base_range && switch_angle_high > pendulum_angle_in_base_range )
        {
            /* Controller should only work when pendulum arm angle is in range [switch_angle_low, pendulum_angle_in_base_range]. */
         
            /* Get number of pendulum revolutions. */
            number_of_pendulumarm_revolutions = get_num_of_revolutions();

            /* Calculate real pendulum angle setpoint from setpoint in base range [0, 2PI]. */
            pend_angle_setpoint = PENDULUM_ANGLE_DOWN_SETPOINT_BASE + (float) number_of_pendulumarm_revolutions * PI2;
            pendulum_arm_angle_setpoint_rad = pend_angle_setpoint; // This variable is only for communication task.

            /* Calculate state varialbes errors */
            cart_position_error =  *cart_position_setpoint_cm - cart_position[0]; 
            cart_speed_error    = - cart_speed[ 0 ];
            pend_position_error =   pend_angle_setpoint - pend_angle[ 0 ];
            pend_speed_error    = - pend_speed[ 0 ];

            /* Calculate control signal contribution of each state variable error */
            /* Deadzone protection nr. 3 "tanh switching" : non linear cart position gain.
            When cart postion error is >0 linear feedback with offset +1V is used
            to compensate for voltage deadzone, for <0 error, y-axis mirror is used.
            graph: https://www.desmos.com/calculator/ycgnqpyy9y */
            
            /* Cart position error control signal component. */
            if( cart_position_error > cart_position_allowed_error_cm )
            {
                ctrl_cart_position_error =   tanhf( 8.0f * cart_position_error ) * ( gains[0] * cart_position_error + voltage_deadzone );
            } 
            else if( cart_position_error < -cart_position_allowed_error_cm )
            {
                ctrl_cart_position_error = - tanhf( 8.0f * cart_position_error ) * ( gains[0] * cart_position_error - voltage_deadzone );
            }
            else
            {
                ctrl_cart_position_error = 0;
            }
            
            /* Pendulum angle error control signal component. */
            if( pend_position_error < pend_position_allowed_error && pend_position_error > -pend_position_allowed_error)
            {
                ctrl_pend_angle_error = 0;
            }
            else
            {
                ctrl_pend_angle_error    = pend_position_error   * gains[1];
            }
            
            /* Cart speed error control signal component. */
            ctrl_cart_speed_error    = cart_speed_error      * gains[2];
            
            /* Pendulum speed error control signal component. */
            ctrl_pend_speed_error    = pend_speed_error      * gains[3];
            
            /* Regular controller update - with constant gains
            control signal = ( state_setpoint - state ) * ( F ) */
            ctrl_signal = ctrl_cart_position_error + 
                        ctrl_pend_angle_error    + 
                        ctrl_cart_speed_error    + 
                        ctrl_pend_speed_error;

            /* Safety to not hit cart max and min positions */
            if( ( cart_position[0] < 5.0f ) || ( cart_position[0] > (TRACK_LEN_MAX_CM-5.0f) ) )
            {
                /* divide control signal by const. to lower the voltage setting. */
                ctrl_signal /= 3;
            }

            /* Additional ctrl signal saturation - for testing.
            There is default saturaition pm12V */    
            // sat( &ctrl_signal, -5.0f, 5.0f );

            /* Set calculated output voltage */
            dcm_set_output_volatage( ctrl_signal );
        }
        else
        {
            dcm_set_output_volatage( 0.0f );
        }

        /* Task delay */
        vTaskDelayUntil( &xLastWakeTime, dt );
    }

}
