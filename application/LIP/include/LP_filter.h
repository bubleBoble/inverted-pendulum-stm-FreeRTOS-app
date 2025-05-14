/*
 * First order low pass filter with transfer function: 
 * 1/(T*s+1)
 * T - time constant
 * Discretized with Tustin (trapezoid) method.
 */
#ifndef LP_FILTER_H
#define LP_FILTER_H

#include <stdint.h>

typedef struct {
        float time_constant;
        float sampling_time;
        float out[2];
        float in[2];
} LP_filter;

void  LP_init(LP_filter *lp, float time_constant, float sampling_time);
float LP_update(LP_filter *lp, float in);
void  LP_update_time_Constant(LP_filter *lp, float new_time_constant);

#endif /* LP_FILTER_H */