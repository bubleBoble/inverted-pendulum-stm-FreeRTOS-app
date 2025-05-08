#include "dcm_encoder_driver.h"

static uint16_t enc_read_raw = 0;
static float cart_position = 0.0f;

/* Start encoder timer in encoder mode */
void enc_init(void)
{
	// Alternative is to start this timer in interrupt or DMA mode
	HAL_TIM_Encoder_Start( &ENC_TIMER_HANDLE, TIM_CHANNEL_ALL );

	// HAL_TIM_Encoder_Start_IT( &ENC_TIMER_HANDLE, TIM_CHANNEL_ALL );
}

/* Return: raw encoder timer count (uint16_t) */
uint16_t enc_get_count(void)
{
	return __HAL_TIM_GET_COUNTER( &ENC_TIMER_HANDLE ); // Reads raw count from encoder timer
}

/* Zero the encoder counter value */
void dcm_enc_zero_counter(void)
{
	__HAL_TIM_SetCounter( &ENC_TIMER_HANDLE, 0 ); // htim4.Instance->CNT = 0;
}

/* Returns cart position in cm */
float dcm_enc_get_cart_position_cm(void)
{
	enc_read_raw = enc_get_count();
	// cart_position = (float) enc_read_raw * 0.00724414303f; // div by (float) ENC_MAX_CNT mult by TRACK_LEN_MAX_CM;
	cart_position = (float) enc_read_raw * ENCODER_MULTIPLIER; // div by (float) ENC_MAX_CNT mult by TRACK_LEN_MAX_CM;
	return cart_position;
}