#include "usart.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

void com_send(char* message)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)message, strlen(message), 500);
}
