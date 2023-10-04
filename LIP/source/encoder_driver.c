/*
 * Description: Basic functionality for interfaceing incremental encoder.
 *
 * Created at: 21/09/2023
 *
 * Notes:
 * 	Timer for encoder is tim4 (htim4) (APB1@84MHz)
 * 	on CH1 & CH2
 *
 *	For Encoder_mode : Encoder Mode TI1   => ARR set to 2255 (final CPR)
 *					   (= 47*48-1 = gear_ration*base_CPR*1)
 * 	For Encoder_mode : Encoder Mode TI1&2 => ARR set to 4511 (final CPR)
 * 					   (= 47*48*2-1 = gear_ration*base_CPR*2*1)
 *
 *	Counter mode: up
 *
 *	GPIOs used: PD12 for CH1 (alias enc_A)
 *		    	PD13 for CH2 (alias enc_B)
 *
 */

// #include "tim.h" // IDE AUTO GENERATED CODE
#include "encoder_driver.h"

float enc_read_deg;
float enc_read_rad;

void enc_init(void)
{
	// Alternative is to start this timer in interrupt or DMA mode
	HAL_TIM_Encoder_Start(&ENC_TIMER_HANDLE, TIM_CHANNEL_ALL);
}

uint16_t enc_get_count(void)
{
	return __HAL_TIM_GET_COUNTER(&htim4); // Reads raw count from encoder timer
}

float enc_get_deg(void)
{
	enc_read_deg = (float)enc_get_count(); 		// read raw encoder count, range [0, 4511]
	enc_read_deg = enc_read_deg / ENC_MAX_CNT;  // Normalize to [0, 1]
	enc_read_deg = enc_read_deg * 360;			// Map to [0, 360]
	return enc_read_deg;
} 

float enc_get_rad(void)
{
	enc_read_deg = (float)enc_get_count(); 		// Read raw encoder count, range [0, 4511]
	enc_read_deg = enc_read_deg / ENC_MAX_CNT;  // Normalize to [0, 1]
	enc_read_deg = enc_read_deg * PI2 ;			// Map to [0, 2PI] (0 is the same as 360)
	return enc_read_rad;
} 