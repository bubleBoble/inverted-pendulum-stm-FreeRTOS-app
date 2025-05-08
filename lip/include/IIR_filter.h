/*
 * 
 */

#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#include <stdint.h>

typedef struct
{
    float alpha;

    float out;
} IIR_filter;

// First order low pass IIR filter 
//     y[n] = (1-a)*x[n] + a*y[n-1]
void IIR_init_fo( IIR_filter *iir, float alpha );
float IIR_update_fo( IIR_filter *iir, float in );

#endif // FIR_FILTER_H