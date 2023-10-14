/*
 *  For FIR filter coefficients use https://www.youtube.com/watch?v=FaG5lPmBY_o
 *  eg. use
 *      #include filter_coeffs.h                            // Defines for filter coeffs
 *      ...
 *      FIR_filter low_pass_FIR;                            // FIR filter struct
 *      float filter_coeffs[ FIR_BUFF_LEN ] = FIR_1;        // Filter coeffs array
 *      FIR_init( &low_pass_FIR, filter_coeffs );           // Init filter struct and assign coeffs
 */

#ifndef FIR_FILTER_H
#define FIR_FILTER_H

#include <stdint.h>

#define FIR_BUFF_LEN 16 // the same as FIR impulse response length

typedef struct 
{
    float buf[ FIR_BUFF_LEN ];
    uint8_t buf_index;
    float out;
    float FIR_COEFFS[ FIR_BUFF_LEN ];
} FIR_filter;

void FIR_init( FIR_filter *fir, float coeffs[FIR_BUFF_LEN]);
float FIR_update( FIR_filter *fir, float inp );

#endif // FIR_FILTER_H