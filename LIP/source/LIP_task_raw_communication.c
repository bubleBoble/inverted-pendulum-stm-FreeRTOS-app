/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides task that implements uC to PC communication over uart. Data is 
 * sent in form of raw bytes, HAL_UART_Transmit_IT (non-blocking mode) is used. 
 * 
 * For human readable com, comTask() is provided.
 * 
 * This task only reads global state and related variables defined in LIP_tasks_common.c.
 * This task shouldn't write to these variables.
 * 
 * Note:
 *     115200 baudrate is used by com_send() function
 *     so about 115200/8=14400 bytes/sec
 *     so 1/14400=0.00006944444 sec/byte
 *     so 0.00166666656 sec for 24 bytes
 *     which is 1.6(6) < 2 ms for one data packet
 *     Tx loop sample period is 10 ms so should be
 *     allright
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    * Com for matlab/simulink - raw bytes com
    * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    * Note:
    * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "LIP_tasks_common.h"

/* These are defined in LIP_tasks_common.c */
extern float pend_angle[ 2 ];
extern float pend_speed[ 2 ];
extern float cart_position[ 2 ];
extern float cart_speed[ 2 ];
extern float *cart_position_setpoint_cm;

void rawComTask( void *pvParameters )
{
    /* For RTOS vTaskDelayUntil() */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     * Raw data transmission for matlab/simulink com
     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
    /* Message structure */
    typedef struct
    {
        float cart_pos;     // float has 4 Bytes
        float cart_speed;   
        float angle;        
        float pend_speed;   
        float volt_setting; 
        float cart_setpoint;
    } tx_data;              // sum: 24 Bytes
    tx_data data;
    char tx_buff[ 24 ];

    /* Task mainloop */
    for (;;)
    {
        /* Message content */
        data.cart_pos       = cart_position[ 0 ];
        data.cart_speed     = cart_speed[ 0 ];
        data.angle          = pend_angle[ 0 ];
        data.pend_speed     = pend_speed[ 0 ];
        data.volt_setting   = dcm_get_output_voltage();
        data.cart_setpoint  = *cart_position_setpoint_cm;
        memcpy( tx_buff, &data, 24 );

        /* Serial send */
        com_send( tx_buff, 24 );

        /* Task delay */
        vTaskDelayUntil( &xLastWakeTime, dt );
    }
}