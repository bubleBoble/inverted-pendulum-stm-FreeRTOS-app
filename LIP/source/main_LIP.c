/*
*   siema
*/

#include "tim.h"
#include "usart.h"
#include "adc.h"
#include "i2c.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "motor_driver.h"
#include "dcm_encoder_driver.h"
#include "com_driver.h"
#include "main_LIP.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pend_enc_driver.h"

TaskHandle_t mag_enc_task_h = NULL;
void mag_enc_task(void *p);

/*
 *    Initialization code
 */
void main_LIP_init(void)
{
    // Initialize PWM timer and zero its PWM output
    dcm_init_output_voltage();
    
    // Initialize encoder timer
    enc_init();

    // Initialize AS5600 encoder 
    pend_enc_init();
}

/*
 *    FreeRTOS main function to start scheduler and freeRTOS objects
 */
void main_LIP_run(void)
{
    xTaskCreate(mag_enc_task, "enctask mag", 500, (void *)0, tskIDLE_PRIORITY+1, &mag_enc_task_h);

    vTaskStartScheduler();

    for (;;) {}
}

/*
 *    Tasks main functions
 */
void mag_enc_task(void *p)
{
    float deg;

    for (;;)
    {
        /* read data */
        pend_enc_read_angle_deg(&deg);
        
        /* console output */
        as5600_interface_debug_print("%.2f\r\n", deg);

        /* delay 100ms */
        vTaskDelay(100);
    }
}
