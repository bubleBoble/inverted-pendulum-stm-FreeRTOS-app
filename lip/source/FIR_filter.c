/*
 *  For FIR filter coefficients use https://www.youtube.com/watch?v=FaG5lPmBY_o
 *  Try to use at most 64 samples for filter coefficients
 *  Number of filter coefficients must the the same as FIR_BUFF_LEN
 */

#include "FIR_filter.h"

void FIR_init( FIR_filter *fir, float coeffs[FIR_BUFF_LEN])
{
    // Zero all samples in the filter buffer
    for( uint8_t i = 0; i < FIR_BUFF_LEN; i++)
    {
        fir->buf[i] = 0.0f;
    }

    // Assign filter coeffs
    for ( int8_t i = 0; i < FIR_BUFF_LEN; i++ )
    {
        fir->FIR_COEFFS[i] = coeffs[i];
    }

    fir->buf_index = 0;
    fir->out = 0.0f;
}

float FIR_update( FIR_filter *fir, float inp )
{
    /* inp is current input sample */
    
    // Store latest sample in buffer
    fir->buf[ fir->buf_index ] = inp;

    // Increment buff index and wrap around if necessay
    fir->buf_index++;
    if( fir->buf_index == FIR_BUFF_LEN )
    {
        fir->buf_index = 0;
    }

    // Convolution, compute new output sample
    // 1. iterate through FIR impulse response samples (FIR_COEFFS)
    // 2. mult. it by shifted samples inside circular buffer
    // iterate from the oldest to the most recent sample
    // can be the other way around, doesn't really matter for conv.
    fir->out = 0.0f;
    uint8_t sum_index = fir->buf_index;
    for( uint8_t i = 0; i<FIR_BUFF_LEN; i++ )
    {
        // inc. index and wrap around if necesay
        if (sum_index > 0)
        {
            sum_index--;
        }
        else
        {
            sum_index = FIR_BUFF_LEN - 1;
        }
        
        // Mult. impulse response with shifted sample and add to output
        fir->out += fir->FIR_COEFFS[ i ] * fir->buf[ sum_index ];
    }
    return fir->out;
}