/*
*   Opis
*/
#include "main_LIP.h"

// /* Used inside limit switch ISR */
// #define READ_ZERO_POSITION_REACHED HAL_GPIO_ReadPin( limitSW_left_GPIO_Port, limitSW_left_Pin )
// #define READ_MAX_POSITION_REACHED HAL_GPIO_ReadPin( limitSW_right_GPIO_Port, limitSW_right_Pin )

extern ADC_HandleTypeDef hadc3;
extern volatile uint16_t adc_data_pot;
extern float pend_init_angle_offset;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * LIP INIT & RUN
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
void main_LIP_init( void )
{
	SCB->CPACR |= ((3 << 10*2)|(3 << 11*2));     // FPU initialization
                                                 // FPU must be enabled before any FPU
                                                 // instruction is executed, otherwise
                                                 // hardware exception will be raised.
    HAL_TIM_Base_Start( &htim2 );                // Start timer for ADC3 pot read
    HAL_ADC_Start_DMA(
        &hadc3, (uint32_t *) &adc_data_pot, 1 ); // init dma for adc
    dcm_init();                                  // Initialize PWM timer and zero its PWM output
    enc_init();                                  // Initialize encoder timer
    pend_enc_init();                             // Initialize AS5600 encoder

    pend_init_angle_offset = (float) pend_enc_get_cumulative_count() / 4096.0f * PI2 - PI;
}
void main_LIP_run( void )
{
    LIPcreateTasks();
    vTaskStartScheduler();

    for (;;) { /* void */ }
}

/* Built in button interrupt callback function. */
uint8_t ZERO_POSITION_REACHED = 0;  // 1 only if left limit switch activated
uint8_t MAX_POSITION_REACHED = 0;   // 1 only if right limit switch activated
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
    if ( GPIO_Pin == limitSW_left_Pin ) // limit switch left
    {
        ZERO_POSITION_REACHED = 1;
        MAX_POSITION_REACHED = 0;
        // dcm_set_output_volatage( 0.0f ); // moved to watchdog task
        dcm_enc_zero_counter();
    }
    if ( GPIO_Pin == limitSW_right_Pin ) // limit switch right
    {
        MAX_POSITION_REACHED = 1;
        ZERO_POSITION_REACHED = 0;
        // dcm_set_output_volatage( 0.0f ); // moved to watchdog task
    }
    if ( GPIO_Pin == blue_btn_Pin ) // built in blue button
    {
        if ( ZERO_POSITION_REACHED )              // go to the max position if cart on the zero
        {
            dcm_set_output_volatage( 2.0f );
        }
        else if ( MAX_POSITION_REACHED )        // go to the zero position if cart on the max
        {
            dcm_set_output_volatage( -2.0f );
        }
        else
        {
            dcm_set_output_volatage( -2.0f );     // if cart not on max or zero, go to the zero position
        }
    }
    else
    {
        __NOP();
    }
}

/* Uart receive interrupt */
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
}

/* Needed for freeeros objects static allocation. */
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

