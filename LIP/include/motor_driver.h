/*
 * Description: Basic functionality for interfaceing DC motor
 * 		        through pwm signal.
 *
 * Created at: 20/09/2023
 *
 * Notes: Timer for PWM motor control is tim3 (htim3) (APB1@84MHz)
 * 	      on CH1 & CH2
 *
 * 	PSC set to 83 (84)
 * 	ARR set to 999 (1000)
 *	Auto-reload preload: enabled
 *	Counter mode: up 
 *  
 *  Pwm freq = 1kHz
 *
 *	GPIOs used: PA6 for CH1 (alias pwm1_dcmA1)
 *		        PA7 for CH2 (alias pwm2_dcmA2)
 *
 */

#ifndef DCM_DRIVER
#define DCM_DRIVER

#include "tim.h"

#define TIMER_HANDLE htim3

#define MAX_INPUT_VOLTAGE_POSITIVE 12.0f
#define MAX_INPUT_VOLTAGE_NEGATIVE -12.0f

#define DCM_PWM_FREQ 1000 // from precalculated value from PSC & ARR(autoreload register)

/*
 * Function to initialize pwm GPIOs and set initial pwm to 0 DC
 */
void dcm_init( void );
void dcm_zero_output_voltage( void );
void dcm_set_output_volatage( float inV );
float dcm_get_output_voltage( void );

// PRIVATE
void dcm_set_ch1_dutycycle( uint16_t dtc );
void dcm_set_ch2_dutycycle( uint16_t dtc );

#endif /* DCM_DRIVER */
