/*
*   Opis
*/

#include "main_LIP.h"


extern UART_HandleTypeDef huart3;
// stdio printf reroute
int _write(int file, char *ptr, int len) 
{
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

// ===============================================
// Helpers
// ===============================================
#define READ_ZERO_POSITION_REACHED HAL_GPIO_ReadPin(limitSW_left_GPIO_Port, limitSW_left_Pin)
#define READ_MAX_POSITION_REACHED HAL_GPIO_ReadPin(limitSW_right_GPIO_Port, limitSW_right_Pin)

// ===============================================
// Magnetic encoder task realted
// ===============================================
TaskHandle_t mag_enc_task_h = NULL;
void mag_enc_task(void *p);
#define MAGENC_STACKDEPTH 500
StackType_t enc_mag_STACKBUFFER [MAGENC_STACKDEPTH];
StaticTask_t enc_mag_TASKBUFFER_TCB; // structure for TCB of statically created task

// ===============================================
// Globals
// ===============================================
uint8_t ZERO_POSITION_REACHED = 0;  // 1 only if left limit switch activated 
uint8_t MAX_POSITION_REACHED = 0;   // 1 only if right limit switch activated

/*
 * ===============================================
 * Initialization code
 * ===============================================
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
 * =====================================================================
 * FreeRTOS main function to start scheduler and create freeRTOS objects
 * =====================================================================
 */
void main_LIP_run(void)
{
    // Tasks
    mag_enc_task_h = xTaskCreateStatic(
                        mag_enc_task,
                        "enctask mag", 
                        MAGENC_STACKDEPTH, 
                        (void *)0, 
                        tskIDLE_PRIORITY+1, 
                        enc_mag_STACKBUFFER,
                        &enc_mag_TASKBUFFER_TCB);

    // Start scheduler
    vTaskStartScheduler();

    for (;;) {}
}

/*
 * ===============================================
 * Tasks main functions
 * ===============================================
 */
void mag_enc_task(void * pvParameters)
{
    // float deg;
    
    if (!READ_ZERO_POSITION_REACHED)
    {
        dcm_set_output_volatage(-2.0f); // start moving trolley to the left
    }

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
 * ====================================================
 * Interrupt callback fucntions
 *
 * Limit switch left: PF12 EXTI12 alias limitSW_left
 * Limit switch right: PF13 EXTI13 alias limitSW_right
 * ====================================================
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == limitSW_left_Pin) // limit switch left
    {
        ZERO_POSITION_REACHED = 1;
        MAX_POSITION_REACHED = 0;
        dcm_set_output_volatage(0.0f);
    }
    if (GPIO_Pin == limitSW_right_Pin) // limit switch right
    {
        MAX_POSITION_REACHED = 1;
        ZERO_POSITION_REACHED = 0;
        dcm_set_output_volatage(0.0f);
    }
    if (GPIO_Pin == blue_btn_Pin) /* built in blue button */
    {
        if (ZERO_POSITION_REACHED)              // go to the max position if trolley on the zero 
        {
            dcm_set_output_volatage(2.0f); 
        } else if (MAX_POSITION_REACHED)        // go to the zero position if trolley on the max 
        { 
            dcm_set_output_volatage(-2.0f);
        } else {
            dcm_set_output_volatage(-2.0f);     // if trolley not on max or zero, go to the zero position  
        }
    }
    else
    {
        __NOP();
    }
}

/* Uart receive interrupt */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
}

/*   
 * ===============================================
 * Needed for freeeros objects static allocation 
 * ===============================================
 */ 
/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    /* If the buffers to be provided to the Idle task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
