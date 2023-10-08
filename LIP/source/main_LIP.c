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

uint8_t trolley_on_the_right = 0;
uint8_t trolley_on_the_left = 0;

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
    
    // dcm_set_output_volatage(0.0f); // start moving trolley to the left

    for (;;)
    {
        // /* read data */
        // pend_enc_read_angle_deg(&deg);
        
        // /* console output */
        // as5600_interface_debug_print("%.2f\r\n", deg);

        // /* delay 100ms */
        // vTaskDelay(100);
    }
}

/* 
 *    Interrupt callback fucntions
 *
 *    Limit switch left: PF12 EXTI12 alias limitSW_left
 *    Limit switch right: PF13 EXTI13 alias limitSW_right
 *
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == limitSW_left_Pin) // limit switch left
    {
        trolley_on_the_left = 1;
        trolley_on_the_right = 0;
        dcm_set_output_volatage(2.0f);
    }
    if (GPIO_Pin == limitSW_right_Pin) // limit switch right
    {
        trolley_on_the_right = 1;
        trolley_on_the_left = 0;
        dcm_set_output_volatage(-2.0f);
    }
    if (GPIO_Pin == blue_btn_Pin) /* built in blue button */
    {
        dcm_set_output_volatage(0.0f);
    }
    else
    {
        __NOP();
    }
}
