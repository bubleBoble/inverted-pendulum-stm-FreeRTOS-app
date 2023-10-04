#include "usart.h"
#include <stdlib.h>
#include <stdio.h>
#include "encoder_driver.h"

struct message
{
    float enc_deg;      // 4 bytes
    uint32_t time_ms;   // 4 bytes
    float time_sec;     // 4 bytes
};
struct message msg;
char tx_buff[128] = {0};

void com_send_test()
{
    sprintf(tx_buff, "%.4f,%li\r\n", enc_get_deg(), HAL_GetTick());
    HAL_UART_Transmit_IT(&huart3, (uint8_t *)tx_buff, 128);
}

// /* UART Rx in interrupt mode */
// void epost_init()
// {
//     HAL_UART_Receive_IT(&huart3, &rx_buff, 1);
// }

// /* Callback function for usart3 data receive */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == USART3)
//     {
//         if (rx_buff == 'K')
//         {
//             msg.enc_deg = enc_get_deg();
//             msg.time_ms = HAL_GetTick(); 
//             msg.time_sec = (float)(msg.time_ms) / 1000;
//             memcpy(msg_buff, &msg, 12);
//             HAL_UART_Transmit(&huart3, (uint8_t *)msg_buff, 12, 10);
//         }
//         HAL_UART_Receive_IT(&huart3, &rx_buff, 1);
//     }
// }