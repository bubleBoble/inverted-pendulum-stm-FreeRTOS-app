/*
*   siema
*/

#include "tim.h"
#include "usart.h"
#include "adc.h"

#include <stdio.h>
#include <string.h>
#include "motor_driver.h"
#include "encoder_driver.h"
#include "com_driver.h"
#include "main_LIP.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

void main_LIP_init(void)
{
    // Initialize PWM timer and zero its PWM output
    dcm_init_output_voltage();
    // Initialize encoder timer
    enc_init();
}

void main_LIP_run(void)
{
    vTaskStartScheduler();

    while (1)
    {
        // empty palce, so called void
    }
}
