/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides task that is the default entry point for LIP controller application 
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "LIP_tasks_common.h"

void startTask( void * pvParameters )
{
    // if ( !READ_ZERO_POSITION_REACHED )
    // {
    //     /* Start moving cart to the left if it's not already there.
    //     Left position has to be reached to reset cart position to zero. */
    //     dcm_set_output_volatage( -2.0f );
    // }

    for ( ;; )
    {
    }
}
