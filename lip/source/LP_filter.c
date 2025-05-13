#include "LP_filter.h"

void LP_init(LP_filter *lp, float time_constant, float sampling_time)
{
        // This function initizalies lp low-pass filter
        if (time_constant < 0.0f) {
                lp->time_constant = 0.0f;
        } else {
                lp->time_constant = time_constant;
        }
        if (sampling_time < 0.0f) {
                lp->sampling_time = 0.0f;
        } else {
                lp->sampling_time = sampling_time;
        }

        lp->out[0] = 0.0f;
        lp->out[1] = 0.0f;
        lp->in[0]  = 0.0f;
        lp->in[1]  = 0.0f;
}

float LP_update(LP_filter *lp, float in)
{
        // This function calculates low-pass filter output
        lp->out[1] = lp->out[0];
        lp->in[1]  = lp->in[0];
        lp->in[0]  = in;

        float c1, c2;
        c1 = lp->sampling_time / (2 * lp->time_constant + lp->sampling_time);
        c2 = (2 * lp->time_constant - lp->sampling_time) /
             (2 * lp->time_constant + lp->sampling_time);
        lp->out[0] = c1 * (lp->in[0] + lp->in[1]) + c2 * lp->out[1];

        return lp->out[0];
}

void LP_update_time_Constant(LP_filter *lp, float new_time_constant)
{
        // Updates time constant of low-pass filter lp,
        // new_time_constant should be >0
        if (new_time_constant < 0.0f) {
                lp->time_constant = 0.0f;
        } else {
                lp->time_constant = new_time_constant;
        }
}