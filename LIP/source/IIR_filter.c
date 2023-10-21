#include "IIR_filter.h"

void IIR_init_fo( IIR_filter *iir, float alpha )
{
    if( alpha < 0.0f )
    {
        iir->alpha = 0.0f;
    }
    else if( alpha > 1.0f )
    {
        iir->alpha = 1.0f;
    }
    else
    {
        iir->alpha = alpha;
    }

    iir->out = 0.0f;
}

float IIR_update_fo( IIR_filter *iir, float in )
{
    iir->out = ( 1.0f - iir->alpha ) * in + iir->alpha * iir->out;

    return iir->out;
}