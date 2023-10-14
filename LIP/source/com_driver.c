#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// #include "usart.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

extern UART_HandleTypeDef huart3;
void com_send(char* message, uint8_t len)
{
    HAL_UART_Transmit_IT(&huart3, (uint8_t *)message, len);
    // HAL_UART_Transmit(&huart3, (uint8_t *)message, len, 100);
}

/* 
    // sprintf( msg, "%f,%f\r\n", angle[0], D_angle );
    // com_send( msg, strlen( msg ) );
*/