/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file contains task that implements full state feedback controller for
 * linear inverted pendulum. Controller keeps pendulum in down position
 *
 * This task is used to:
 *    1. Read dcm encoder,
 *    2. Read pendulum magnetic encoder
 *    3. Calculate derivatives of cart position and pend angular position
 *    4. Calculate cart position setpoint from adc potentiometer reading
 *
 * This task writes to the following global variables:
 *         state variable              |  variable name in prog         |  unit
 *         --------------------------------------------------------------------------------
 *         Cart position:              |  cart_position[ 0 ]            |  cm
 *         Cart speed:                 |  cart_speed (filtered)         |  cm/sec
 *         Pendulum angle:             |  angle[ 0 ]                    |  rad
 *         Pendulum speed:             |  pend_speed (filtered)         |  rad/sec
 *         --------------------------------------------------------------------------------
 *         cart position setpoint pot  |  cart_position_cm_setpoint_pot |  cm
 *         cart position setpoint cli  |  cart_position_cm_setpoint_cli |  cm
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

void utilTask( void *pvParameters )
{
    /* For RTOS vTaskDelayUntil() */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     * FILTERS
     * Pendulum magnetic encoder reading, IIR
     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
    float alpha_pend = 0.5;
    IIR_init_fo( &low_pass_IIR_pend, alpha_pend );

    /* DCM encoder reading, IIR */
    float alpha_cart = 0.75;
    IIR_init_fo( &low_pass_IIR_cart, alpha_cart );

    /* Low pass filter for cart position setpoint (pot and cli), 0.2sec time constant, 0dc gain. */
    LP_filter sp_filter_pot;
    LP_init( &sp_filter_pot, 0.2f, dt*0.001f );
    LP_filter sp_filter_cli;
    LP_init( &sp_filter_cli, 0.2f, dt*0.001f );

    /* Task main loop */
    for ( ;; )
    {
        /* Pendulum magnetic encoder reading */
        pend_angle[ 1 ] = pend_angle[ 0 ];
        pend_angle[ 0 ] = (float) pend_enc_get_cumulative_count() / 4096.0f * PI2 - pend_init_angle_offset;

        /* --------- Derivatives --------- */
        /* Pendulum angular speed calculation - backward Euler method */
        // pend_speed[0] = ( pend_angle[0] - pend_angle[1] ) * dt_inv;

        /* Pendulum angular speed calculation - Tusting method */
        pend_speed[ 1 ] = pend_speed[ 0 ];
        pend_speed[ 0 ] = ( pend_angle[0] - pend_angle[1] ) * 2 * dt_inv - pend_speed[ 1 ];
        /* ------------------------------- */

        /* IIR filter for pendulum angle derivative */
        IIR_update_fo( &low_pass_IIR_pend, pend_speed[ 0 ] );
        pend_speed[ 0 ] = low_pass_IIR_pend.out;

        /* dead zone for calculated pendulum speed, about +-10 deg/sec */
        if ( ( pend_speed[ 0 ] < 0.2 ) && ( pend_speed[ 0 ] > -0.2 ) )
        {
            pend_speed[ 0 ] = 0;
        }

        /* DCM encoder reading */
        cart_position[ 1 ] = cart_position[ 0 ];
        cart_position[ 0 ] = dcm_enc_get_cart_position_cm();

        /* --------- Derivatives --------- */
        /* Cart speed calculation - backward Euler method */
        // cart_speed[0] = ( cart_position[ 0 ] - cart_position[ 1 ] ) * dt_inv; // 1st deriv

        /* Cart speed calculation - Tustin method */
        cart_speed[ 1 ] = cart_speed[ 0 ];
        cart_speed[ 0 ] = ( cart_position[ 0 ] - cart_position[ 1 ] ) * 2 * dt_inv - cart_speed[ 1 ]; // 1st deriv
        /* ------------------------------- */

        /* IIR filter for cart speed */
        IIR_update_fo( &low_pass_IIR_cart, cart_speed[ 0 ] );
        cart_speed[ 0 ] = low_pass_IIR_cart.out;

        /* Dead zone for calculated cart speed */
        // if ( ( cart_speed < 0.2 ) && ( cart_speed > -0.2 ) )
        // {
        //     cart_speed = 0;
        // }

        /* Calculate cart position setpoint from potentiometer adc reading, global variable. */
        cart_position_setpoint_cm_pot_raw = (float) adc_data_pot / 4096.0f * TRACK_LEN_MAX_CM;

        /* Low pass filter for cart position setpoint (pot and cli), 0.2sec time constant, 0dc gain. */
        // input is cart_position_setpoint_cm_pot_raw or cart_position_setpoint_cm_cli_raw, 
        // output samples are stored internally in sp_filter struct.
        // The latest sample is assiged to cart_position_setpoint_cm_pot or cart_position_setpoint_cm_cli_raw
        LP_update( &sp_filter_pot, cart_position_setpoint_cm_pot_raw );
        cart_position_setpoint_cm_pot = sp_filter_pot.out[ 0 ];

        LP_update( &sp_filter_cli, cart_position_setpoint_cm_cli_raw );
        cart_position_setpoint_cm_cli = sp_filter_cli.out[ 0 ];

        if( app_current_state == DEFAULT )
        {
            /* While in DEFAULT state, value of cart_position_setpoint_cm_cli_raw should be constantly updated to
            the value of current cart position to avoid discontinuity in cart position setpoint while 
            turning on down position controller. */
            cart_position_setpoint_cm_cli_raw = cart_position[ 0 ];
        }

        /* Task delay */
        vTaskDelayUntil( &xLastWakeTime, dt );
    }
}