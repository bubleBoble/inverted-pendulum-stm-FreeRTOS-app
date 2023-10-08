/*
 * Description: Basic functionality for interfaceing incremental encoder.
 *
 * Created at: 21/09/2023
 *
 * Notes:
 * 	Timer for encoder is tim4 (htim4) (APB1@84MHz)
 * 	on CH1 & CH2
 *
 *	For Encoder_mode : Encoder Mode TI1   => ARR set to 1103 (final CPR)
 *					   (= 23*48-1 = 1103 = gear_ration*base_CPR*1)
 * 	For Encoder_mode : Encoder Mode TI1&2 => ARR set to 2207 (final CPR)
 * 					   (= 23*48*2-1 = 2207 = gear_ration*base_CPR*2*1)
 *
 *	Counter mode: up
 *
 *	GPIOs used: PD12 for CH1 (alias enc_A)
 *		    	PD13 for CH2 (alias enc_B)
 *
 */

#ifndef DCM_ENC_DRIVER
#define DCM_ENC_DRIVER

#include "tim.h" // IDE AUTO GENERATED CODE

/* handle to timer */
#define ENC_TIMER_HANDLE htim4
/* Max encoder timer count */
#define ENC_MAX_CNT 2207
#define PI  3.1415926536
#define PI2 6.2831853072

/* Encoder timer count converted to degrees */
extern float enc_read_deg;
/* Encoder timer count converted to radians */
extern float enc_read_rad;

/* To start encoder timer in encoder mode */
void enc_init(void);
/* Return: raw encoder timer count (uint16_t) */
uint16_t enc_get_count(void);
/* Return: encoder timer count converted to degrees */
float enc_get_deg(void);
/* Return: encoder timer count converted to radians */
float enc_get_rad(void);

#endif /* DCM_ENC_DRIVER */
