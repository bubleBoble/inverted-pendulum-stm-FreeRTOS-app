/*
 * Description: Basic functionality for interfaceing incremental encoder.
 *
 * Created at: 21/09/2023
 *
 * Notes:
 * 	Timer for encoder is tim4 (htim4) (APB1@84MHz)
 * 	on CH1 & CH2
 *
 *  Whole IP track generates about 6488 ticks
 *  So dcm encoder ARR register is set to 7000 so that it won't 
 *  randomly reset its value at limit max position.
 *  Track length is 47cm 
 *      6488 ticks -> 47cm
 * 
 * 	Encoder mode: Encoder Mode TI1&2 => ARR set to 7000 (final CPR)
 *
 *	Counter mode: up
 *
 *	GPIOs used: PD12 for CH1 (alias enc_A)
 *		    	PD13 for CH2 (alias enc_B)
 *
 */

#include "tim.h"

#ifndef DCM_ENC_DRIVER
#define DCM_ENC_DRIVER

/* handle to timer */
#define ENC_TIMER_HANDLE htim4

/* Max encoder timer count */
#define ENC_MAX_CNT 6488
#define TRACK_LEN_MAX_CM 47.0f

/* Start encoder timer in encoder mode */
void enc_init(void);

/* Return: raw encoder timer count (uint16_t) */
uint16_t enc_get_count(void);

/* Zero the encoder counter value */
void dcm_enc_zero_counter(void);

/* Returns cart position in cm */
float dcm_enc_get_cart_position_cm(void);

#endif /* DCM_ENC_DRIVER */
