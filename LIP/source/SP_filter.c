#include "SP_filter.h"

void LP_init( LP_filter *lp, float timeConstant, float samplingTime )
{
    if( timeConstant < 0.0f )
    {
        lp->timeConstant = 0.0f;
    }
    else
    {
        lp->timeConstant = timeConstant;
    }
    if( samplingTime < 0.0f )
    {
        lp->samplingTime = 0.0f;
    }
    else
    {
        lp->samplingTime = samplingTime;
    }

    lp->out[ 0 ] = 0.0f;
    lp->out[ 1 ] = 0.0f;
    lp->in[ 0 ]  = 0.0f;
    lp->in[ 1 ]  = 0.0f;
}

float LP_update( LP_filter *lp, float in )
{
    lp->out[ 1 ] = lp->out[ 0 ];
    lp->in[ 1 ]  = lp->in[ 0 ];
    lp->in[ 0 ]  = in;

    float c1, c2;
    c1 = lp->samplingTime / ( 2*lp->timeConstant + lp->samplingTime );
    c2 = (2*lp->timeConstant - lp->samplingTime) / (2*lp->timeConstant + lp->samplingTime);
    lp->out[ 0 ] = c1 * ( lp->in[ 0 ] + lp->in[ 1 ] ) + c2 * lp->out[ 1 ] ;

    return lp->out[ 0 ];
}