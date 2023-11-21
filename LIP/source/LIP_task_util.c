/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file contains task that implements full state feedback controller for
 * linear inverted pendulum. Controller keeps pendulum in down position
 *
 * This task is used to:
 *     1. Read dcm encoder,
 *     2. Read pendulum magnetic encoder
 *     3. Calculate derivatives of cart position and pend angular position
 *     4. Calculate cart position setpoint from adc potentiometer reading
 *     5. Calculate number of pendulum arm full revolutions
 *
 * Note about modulus:
 *     Calculate pendulum arm angle in base range [0 2pi]. This method uses modulus operation but implemented as
 *     "floored division" rather than "truncated division" implemented in math.h in mod, fmod, fmodf etc. 
 *     More info about modulus operation: https://en.wikipedia.org/wiki/Modulo#In_programming_languages 
 *     Graph: https://www.desmos.com/calculator/qaacl2m3cu 
 * 
 * This task writes to the following global variables:
 *     state variable              |  variable name in prog         |  unit
 *     --------------------------------------------------------------------------------
 *     Cart position:              |  cart_position[ 0 ]            |  cm
 *     Cart speed:                 |  cart_speed (filtered)         |  cm/sec
 *     Pendulum angle:             |  angle[ 0 ]                    |  rad
 *     Pendulum speed:             |  pend_speed (filtered)         |  rad/sec
 *     --------------------------------------------------------------------------------
 *     cart position setpoint pot  |  cart_position_cm_setpoint_pot |  cm
 *     cart position setpoint cli  |  cart_position_cm_setpoint_cli |  cm
 *
 * Poll the pnedulum encoder at least 3 times per full revolution
 *
 * This task runs every 10ms
 *
 * saving previous sample readings convention:
 *    reading [0] : reading [n]     : current sample (n)
 *    reading [1] : reading [n - 1] : previous sample (n-1)
 *    etc.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include "LIP_tasks_common.h"
#include <math.h>

/* These are defined in LIP_tasks_common.c */
extern volatile uint16_t adc_data_pot;
extern float pend_angle[ 2 ];
extern float pend_speed[ 2 ];
extern float cart_position[ 2 ];
extern float cart_speed[ 2 ];
extern IIR_filter low_pass_IIR_pend;
extern IIR_filter low_pass_IIR_cart;
extern float cart_position_setpoint_cm_pot_raw;
extern float cart_position_setpoint_cm_pot;
extern float cart_position_setpoint_cm_cli_raw;
extern float cart_position_setpoint_cm_cli;
extern float pend_init_angle_offset;
extern enum lip_app_states app_current_state;
extern float number_of_pendulumarm_revolutions_dpc;
extern float pendulum_angle_in_base_range_dpc;
extern float number_of_pendulumarm_revolutions_upc;
extern float pendulum_angle_in_base_range_upc;
extern float pendulum_arm_angle_setpoint_rad_upc;
extern float pendulum_arm_angle_setpoint_rad_dpc;

void utilTask( void *pvParameters )
{
    /* For RTOS vTaskDelayUntil() */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     * Low pass filters for derivatives. Pendulum and cart speed.
     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
    /* IIR for pendulum position */
    float alpha_pend = 0.65;
    IIR_init_fo( &low_pass_IIR_pend, alpha_pend );

    /* DCM encoder reading, IIR */
    float alpha_cart = 0.54;
    IIR_init_fo( &low_pass_IIR_cart, alpha_cart );

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     * Low pass filters for setpoints - cli and pot.
     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
    /* Low pass filter for cart position setpoint (pot and cli), 0.2sec time constant, 0dc gain. */
    LP_filter sp_filter_pot;
    LP_init( &sp_filter_pot, 0.2f, dt*0.001f );
    
    LP_filter sp_filter_cli;
    LP_init( &sp_filter_cli, 0.2f, dt*0.001f );

    for ( ;; )
    {
        /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
         * Pendulum angular position - magnetic encoder reading 
         * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
        pend_angle[ 1 ] = pend_angle[ 0 ];
        pend_angle[ 0 ] = ( float ) pend_enc_get_cumulative_count() / 4096.0f * PI2 - pend_init_angle_offset;
        
        /* ??? filter for pendulum angle ??? */
        // IIR_update_fo( &low_pass_IIR_pend, pend_angle[ 0 ] );
        // pend_angle[ 0 ] = low_pass_IIR_pend.out;

        /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
         * Pendulum angular speed calculation with Tustin method 
         * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
        pend_speed[ 1 ] = pend_speed[ 0 ];
        pend_speed[ 0 ] = ( pend_angle[0] - pend_angle[1] ) * 2 * dt_inv - pend_speed[ 1 ];

        /* ??? IIR filter for pendulum angle derivative ??? */
        IIR_update_fo( &low_pass_IIR_pend, pend_speed[ 0 ] );
        pend_speed[ 0 ] = low_pass_IIR_pend.out;

        /* Dead zone for calculated pendulum speed, about +-10 deg/sec. */ 
        if ( ( pend_speed[ 0 ] < 0.2 ) && ( pend_speed[ 0 ] > -0.2 ) )
        {
            pend_speed[ 0 ] = 0;
        }

        /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
         * Cart position - DCM encoder reading 
         * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
        cart_position[ 1 ] = cart_position[ 0 ];
        cart_position[ 0 ] = dcm_enc_get_cart_position_cm();

        /* ??? IIR filter for cart position ??? */
        // IIR_update_fo( &low_pass_IIR_cart, cart_position[ 0 ] );
        // cart_position[ 0 ] = low_pass_IIR_cart.out;

        /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
         * Cart speed with Tustin method
         * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
        cart_speed[ 1 ] = cart_speed[ 0 ];
        cart_speed[ 0 ] = ( cart_position[ 0 ] - cart_position[ 1 ] ) * 2 * dt_inv - cart_speed[ 1 ]; // 1st deriv

        /* ??? IIR filter for cart speed ??? */
        IIR_update_fo( &low_pass_IIR_cart, cart_speed[ 0 ] );
        cart_speed[ 0 ] = low_pass_IIR_cart.out;

        /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
         * Cart position setpoint from potentiometer adc reading, global variable. 
         * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
        cart_position_setpoint_cm_pot_raw = (float) adc_data_pot / 4096.0f * TRACK_LEN_MAX_CM;

        /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
         * Low pass filter for cart position setpoint (pot and cli), 0.2sec time constant, 0dc gain. 
         * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
        /* input is cart_position_setpoint_cm_pot_raw or cart_position_setpoint_cm_cli_raw, 
        output samples are stored internally in sp_filter struct.
        The latest sample is assiged to cart_position_setpoint_cm_pot or cart_position_setpoint_cm_cli_raw */
        LP_update( &sp_filter_pot, cart_position_setpoint_cm_pot_raw );
        cart_position_setpoint_cm_pot = sp_filter_pot.out[ 0 ];

        LP_update( &sp_filter_cli, cart_position_setpoint_cm_cli_raw );
        cart_position_setpoint_cm_cli = sp_filter_cli.out[ 0 ];

        if( app_current_state == DEFAULT )
        {
            /* While in DEFAULT state, cart setpoing is current position. 
            
            Value of cart_position_setpoint_cm_cli_raw should be constantly updated to
            the value of current cart position to avoid discontinuity in cart position setpoint. */
            cart_position_setpoint_cm_cli_raw = cart_position[ 0 ];
        }

        /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
         * For DPC - Angle switching in up position - switching in down position would generate
         * discontinuities in values of angle setpoint. 
         * 
         * This method uses modulus operation but implemented as
         * "floored division" rather than "truncated division" implemented in math.h in mod, fmod, fmodf etc. 
         * More info about modulus operation: https://en.wikipedia.org/wiki/Modulo#In_programming_languages 
         * Graph: https://www.desmos.com/calculator/qaacl2m3cu  
         * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

        /* Calculate number of revolutions using floored division. */
        number_of_pendulumarm_revolutions_dpc = floorf( pend_angle[ 0 ] / PI2 );
        
        /* Calculate pendulum angle in base range [0, 2PI].
        This range stays the same as original range in the model, so that pendulum angle of 180 degree
        or PI radians still corresponds to down position. */
        pendulum_angle_in_base_range_dpc = pend_angle[ 0 ] - PI2 * number_of_pendulumarm_revolutions_dpc;

        /* Angle setpoint for pendulum arm. Down position corresponds to 180 degrees or pi radians.
        Controller works with angle in radians. "BASE" postfix indicates that this setpoint is from
        base angle range [0, 2PI]. Because pendulum arm can make many full revolutions,
        angles PI, 3PI, 5PI and so on, all correspond to the same down position, angle setpoint needs
        to be changed accordingly. */
        #define PENDULUM_ANGLE_DOWN_SETPOINT_BASE PI

        /* Calculate real pendulum angle setpoint from setpoint in base range [0, 2PI] for DPC. */
        pendulum_arm_angle_setpoint_rad_dpc = PENDULUM_ANGLE_DOWN_SETPOINT_BASE + number_of_pendulumarm_revolutions_dpc * PI2;
        
        /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
         * For UPC - Angle switching in down position - switching on top would generate
         * discontinuities in values of angle setpoint. 
         * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
    
        /* Calculate number of revolutions using floored division. 
        Note: +1 because when pendulum starts in down position, transition to up position in CCW direction
        counts as negative revolution adding one compensates for that. */
        number_of_pendulumarm_revolutions_upc = floorf( ( pend_angle[ 0 ] - PI ) / PI2 ) + 1;
        
        /* Calculate pendulum angle in base range [-PI, PI]. 
        This range is changed to [-PI, PI], so that pendulum angle of zero degree corresponds to up position
        and there is no discontinuity around zero degree angle. */
        pendulum_angle_in_base_range_upc = pend_angle[ 0 ] - PI2 * number_of_pendulumarm_revolutions_upc;

        /* Angle setpoint for pendulum arm. Up position corresponds to 0 degrees or 0 radians.
        Controller works with angle in radians. "BASE" postfix indicates that this setpoint is from
        base angle range [0, 2PI]. Because pendulum arm can make many full revolutions,
        angles 0, 2PI, 4PI and so on, all correspond to the same up position, angle setpoint needs
        to be changed accordingly. */
        // non zero value because of pendulum encoder error
        // #define PENDULUM_ANGLE_UP_SETPOINT_BASE 0.0f
        #define PENDULUM_ANGLE_UP_SETPOINT_BASE -0.070563f

        /* Calculate real pendulum angle setpoint from setpoint in base range [-PI, PI] for UPC. */
        pendulum_arm_angle_setpoint_rad_upc = PENDULUM_ANGLE_UP_SETPOINT_BASE + number_of_pendulumarm_revolutions_upc * PI2;

        /* Task delay */
        vTaskDelayUntil( &xLastWakeTime, dt );
    } /* for ( ;; ) */
}