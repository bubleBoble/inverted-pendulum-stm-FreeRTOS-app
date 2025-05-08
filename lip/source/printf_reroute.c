/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * stdio reroute - use it only for testing
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include <stdint.h>
#include "main_LIP.h"

extern UART_HandleTypeDef huart3;

int _write( int file, char *ptr, int len )
{
    HAL_UART_Transmit( &huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY );
    // HAL_UART_Transmit_IT(&huart3, (uint8_t *)ptr, len);
    return len;
}
