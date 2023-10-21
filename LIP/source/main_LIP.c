/*
*   Opis
*/
#include "main_LIP.h"

/* For limit switch interrupts */
#define READ_ZERO_POSITION_REACHED HAL_GPIO_ReadPin( limitSW_left_GPIO_Port, limitSW_left_Pin )
#define READ_MAX_POSITION_REACHED HAL_GPIO_ReadPin( limitSW_right_GPIO_Port, limitSW_right_Pin )

extern UART_HandleTypeDef huart3;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc3;
extern TIM_HandleTypeDef htim2;

/* ========================================================================
 * GLOBALS
 * ========================================================================
 */
    // Holds data from ADC3 tranfered over DMA, init code is in motor_driver.c/dcm_init()
    volatile uint16_t adc_data_pot;

    // holds each byte received from console (uart3)
    uint8_t cRxedChar = 0x00;

    // These are made global but only basic_test_task will write to these
    // Only controller_task should read these
    // Pendulum magnetic encoder reading
    float angle[ 2 ] = { 0.0f };                             // Angle current & previous sample
    float D_angle;                                           // Angle derivative
    FIR_filter low_pass_FIR_pend;                            // Low pas filter for angle derivative
    IIR_filter low_pass_IIR_pend;

    // These are made global but only basic_test_task will write to them
    // Only controller_task should read these
    // DCM encoder reading
    float cart_position[ 2 ] = { 0.0f };
    float D_cart_position;
    FIR_filter low_pass_FIR_cart;
    IIR_filter low_pass_IIR_cart;

    float cart_position_setpoint;

/* ========================================================================
 * stdio reroute - use it only for testing
 * ========================================================================
 */
int _write( int file, char *ptr, int len )
{
    HAL_UART_Transmit( &huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY );
    // HAL_UART_Transmit_IT(&huart3, (uint8_t *)ptr, len);
    return len;
}

/* ========================================================================
 * TASKS RELATED
 * ========================================================================
 */
/* Start task */
#define STARTTASK_STACKDEPTH 500
TaskHandle_t startTaskHandle = NULL;
void startTask( void *pvParameters );
StackType_t startTask_STACKBUFFER[ STARTTASK_STACKDEPTH ];
StaticTask_t startTask_TASKBUFFER_TCB;

/* console task */
#define MAX_INPUT_LENGTH    50
#define MAX_OUTPUT_LENGTH   100
#define CONSOLE_STACKDEPTH  4000
TaskHandle_t consoleTaskHandle;
void vCommandConsoleTask( void *pvParameters );
StackType_t console_STACKBUFFER[ CONSOLE_STACKDEPTH ];
StaticTask_t console_TASKBUFFER_TCB;
TickType_t time_at_which_consoleMutex_was_taken;

/* Basic test task */
#define BASIC_TEST_STACK_DEPTH 500
TaskHandle_t basicTestTaskHandle = NULL;
void basicTestTask( void *pvParameters );
StackType_t basicTestTask_STACKBUFFER [ BASIC_TEST_STACK_DEPTH ];
StaticTask_t basicTestTask_TASKBUFFER_TCB;

/* Controller test task */
#define CTRL_TEST_STACK_DEPTH 500
TaskHandle_t ctrlTestTaskHandle = NULL;
void ctrlTestTask( void *pvParameters );
StackType_t ctrlTestTask_STACKBUFFER [ CTRL_TEST_STACK_DEPTH ];
StaticTask_t ctrlTestTask_TASKBUFFER_TCB;

/* ========================================================================
 * LIP INIT & RUN
 * ========================================================================
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
}
void main_LIP_run( void )
{
    LIPcreateTasks();
    vTaskStartScheduler();

    for (;;) { /* void */ }
}

/* ========================================================================
 * Interrupts callback functions
 * ========================================================================
 */
uint8_t ZERO_POSITION_REACHED = 0;  // 1 only if left limit switch activated
uint8_t MAX_POSITION_REACHED = 0;   // 1 only if right limit switch activated

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
    if ( GPIO_Pin == limitSW_left_Pin ) // limit switch left
    {
        ZERO_POSITION_REACHED = 1;
        MAX_POSITION_REACHED = 0;
        dcm_set_output_volatage( 0.0f );
        dcm_enc_zero_counter();
    }
    if ( GPIO_Pin == limitSW_right_Pin ) // limit switch right
    {
        MAX_POSITION_REACHED = 1;
        ZERO_POSITION_REACHED = 0;
        dcm_set_output_volatage( 0.0f );
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

/* ========================================================================
 * Needed for freeeros objects static allocation
 * ========================================================================
 */
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

/* ========================================================================
 * CREATE TASKS
 * ========================================================================
 */
void LIPcreateTasks()
{
    // startTaskHandle = xTaskCreateStatic( startTask,
    //                                      (const char*) "enctask mag",
    //                                      STARTTASK_STACKDEPTH,
    //                                      (void *) 0,
    //                                      tskIDLE_PRIORITY+1,
    //                                      startTask_STACKBUFFER,
    //                                      &startTask_TASKBUFFER_TCB );

    // consoleTaskHandle = xTaskCreateStatic( vCommandConsoleTask,
    //                                        (const char*) "Console",
    //                                        CONSOLE_STACKDEPTH,
    //                                        (void *) 0,
    //                                        tskIDLE_PRIORITY+1,
    //                                        console_STACKBUFFER,
    //                                        &console_TASKBUFFER_TCB );

    basicTestTaskHandle = xTaskCreateStatic( basicTestTask,
                                             (const char*) "Basic test task",
                                             BASIC_TEST_STACK_DEPTH,
                                             (void *) 0,
                                             tskIDLE_PRIORITY+2,
                                             basicTestTask_STACKBUFFER,
                                             &basicTestTask_TASKBUFFER_TCB );

    // ctrlTestTaskHandle = xTaskCreateStatic( ctrlTestTask,
    //                                         (const char*) "Controller test task",
    //                                         CTRL_TEST_STACK_DEPTH,
    //                                         (void *) 0,
    //                                         tskIDLE_PRIORITY+3,
    //                                         ctrlTestTask_STACKBUFFER,
    //                                         &ctrlTestTask_TASKBUFFER_TCB );
}

/* ========================================================================
 * START TASK
 * ========================================================================
 */
void startTask( void * pvParameters )
{
    // float deg;

    if ( !READ_ZERO_POSITION_REACHED )
    {
        dcm_set_output_volatage( -2.0f ); // start moving cart to the left
    }

    for ( ;; )
    {
    }
}

/* ========================================================================
 * BASIC TEST TASK
 * ========================================================================
 */
#define dt      10
#define dt_inv  100.0f
void basicTestTask( void *pvParameters )
{
    /*
     * This task is used to :
     *    1. Read dcm encoder,
     *    2. Read pendulum magnetic encoder
     *    3. Calculate derivatives of cart position and pend angular position
     *    SHOULD BE MOVED TO ANOTHER TASK :
     *        4. Transmit these variables for testing with serial osciloscope
     *
     * Poll the pnedulum encoder at least 3 times per full revolution
     *
     * This task runs every 10ms
     *
     * saving previous sample readings convention:
     *    reading [0] : reading [n]     : current sample (n)
     *    reading [1] : reading [n - 1] : previous sample (n-1)
     *    etc.
     */
    TickType_t xLastWakeTime = xTaskGetTickCount(); // For RTOS vTaskDelayUntil()

    /////////////////////////////////////////////////////////////////////////////////////////
    // FILTERS
    /////////////////////////////////////////////////////////////////////////////////////////
    // Pendulum magnetic encoder reading
        // FIR 1
        float filter_coeffs_pend[ FIR_BUFF_LEN ] = FIR_1;   // FIR_1 is #define in filtePrs_coeffs.h
        FIR_init( &low_pass_FIR_pend, filter_coeffs_pend );
        
        // IIR 1
        float alpha_pend = 0.5;
        IIR_init_fo( &low_pass_IIR_pend, alpha_pend );

    // DCM encoder reading
        // FIR 1
        float filter_coeffs_cart[ FIR_BUFF_LEN ] = FIR_1;
        FIR_init( &low_pass_FIR_cart, filter_coeffs_cart );
        
        // IIR 1
        float alpha_cart = 0.75;
        IIR_init_fo( &low_pass_IIR_cart, alpha_cart );

    /////////////////////////////////////////////////////////////////////////////////////////
    // For serial osciloscope - testing/debug purposes
    /////////////////////////////////////////////////////////////////////////////////////////
    char msg[ 128 ]; // msg for uart data transfer
    float voltage_setting;

    /////////////////////////////////////////////////////////////////////////////////////////
    // Raw data transmission for matlab/simulink com
    /////////////////////////////////////////////////////////////////////////////////////////
    typedef struct
    {
        float cart_pos;     // float 4 Bytes
        float cart_speed;   
        float angle;        
        float pend_speed;   
        float volt_setting; 
        float cart_setpoint;
    } tx_data;              // sum: 24 Bytes
    tx_data data;
    char tx_buff[ 24 ];

    // Task main loop
    for ( ;; )
    {
        // Pendulum magnetic encoder reading
        /////////////////////////////////////////////////////////////////////////////////////////
        // read_in_degrees = pend_raw_read / 4096 * 360 + 180 + 9.404
        // 9.404 is base value which as5600 outputs when pendulum is in down position
        // Todo: as5600 setting for base ang. position can be changed later
        angle[ 1 ] = angle[ 0 ];
        angle[ 0 ] = (float) pend_enc_get_cumulative_count() / 4096.0f * PI2 + 0.1455 + PI; // 360/4096=0.087890625
        D_angle = ( angle[0] - angle[1] ) * dt_inv; // Pend. angle first derivative (wrt time)

        // FILTERS
            // FIR for pendulum angle
            // FIR_update( &low_pass_FIR_pend, angle[0] );

            // FIR filter for pendulum angle derivative
            // FIR_update( &low_pass_FIR_pend, D_angle );
            // D_angle = low_pass_FIR_pend.out;
            // if ( ( D_angle < 0.2 ) && ( D_angle > -0.2 ) ) // dead zone for derivative about +-10 deg/sec
            // {
            //     D_angle = 0;
            // }

            // IIR filter for pendulum angle derivative
            IIR_update_fo( &low_pass_IIR_pend, D_angle );
            // D_angle = low_pass_IIR_pend.out;
            if ( ( D_angle < 0.2 ) && ( D_angle > -0.2 ) ) // dead zone for derivative about +-10 deg/sec
            {
                D_angle = 0;
            }

        // DCM encoder reading
        //////////////////////////////////////////////////////////////////////////////////////////
        cart_position[ 1 ] = cart_position[ 0 ];
        cart_position[ 0 ] = dcm_enc_get_cart_position_cm();

        // FILTERS
            // FIR filter for cart position;
            // FIR_update( &low_pass_FIR_cart, cart_position[0] );

            D_cart_position = ( cart_position[ 0 ] - cart_position[ 1 ] ) * dt_inv; // 1st deriv

            // // FIR for cart velocity
            FIR_update( &low_pass_FIR_cart, D_cart_position );

            // IIR filter for cart speed
            IIR_update_fo( &low_pass_IIR_cart, D_cart_position );
                // D_angle = low_pass_IIR_pend.out;
                // if ( ( D_angle < 0.2 ) && ( D_angle > -0.2 ) ) // dead zone for derivative about +-10 deg/sec
                // {
                //     D_angle = 0;
                // }

        //////////////////////////////////////////////////////////////////////////////////////////
        // LOGGING - good enough to be used with serial osciloscope and data logging with
        // for eg. hterm
        //////////////////////////////////////////////////////////////////////////////////////////

        // Message 1
        // voltage_setting = dcm_get_output_voltage();
        sprintf( msg,
                 "%f,%f,%f,%f,%f,%f,%f,%ld\r\n",
                 (double) cart_position[0],            // CH1
                 (double) D_cart_position,             // CH2
                 (double) low_pass_FIR_cart.out,        // CH3
                 (double) angle[0],                    // CH4
                 (double) D_angle,                     // CH5
                 (double) low_pass_FIR_pend.out,       // CH6
                 (double) voltage_setting,             // CH7
                 xLastWakeTime                         // CH8
        );

        // Message 2
        // sprintf( msg,
        //          "%.3f,%.3f,0,%.3f,%.3f\r\n",
        //          (double)cart_position[0],
        //          (double)D_cart_position,
        //          (double)angle[0]);

        // Message 3
        // sprintf( msg,
        //          "%d,%f\r\n",
        //          adc_data_pot,
        //          (double)angle[0]);

        // Message 4
        sprintf( msg,
                 "%f,%f,%f,%f,%f,%f,%f,%ld\r\n",
                 // for pendulum
                 (double) angle[0],
                 (double) D_angle,
                 (double) low_pass_IIR_pend.out,
                 // for cart
                 (double) cart_position[ 0 ],
                 (double) D_cart_position,
                 (double) low_pass_IIR_cart.out,
                 //
                 (double) voltage_setting,
                 xLastWakeTime
        );

        // Message 5 - used for potentiometer testing
        // sprintf( msg, "%d\r\n", adc_data_pot );
        
        com_send( msg, strlen(msg) );

        //////////////////////////////////////////////////////////////////////////////////////////
        // Com for matlab/simulink - raw byte com
        //////////////////////////////////////////////////////////////////////////////////////////
        // Note:
        //     115200 baudrate
        //     so about 115200/8=14400 bytes/sec
        //     so 1/14400=0.00006944444 sec/byte
        //     so 0.00083333333 sec for 12 bytes
        //     which is 0.83(3) < 1 ms for one data packet
        //     Tx loop sample period is 10 ms so should be
        //     allright
        // data.cart_pos       = cart_position[ 0 ];
        // data.cart_speed     = D_cart_position;
        // data.angle          = angle[ 0 ];
        // data.pend_speed     = D_angle;
        // data.volt_setting   = dcm_get_output_voltage();
        // data.cart_setpoint  = cart_position_setpoint;
        // memcpy( tx_buff, &data, 24 );
        // com_send( tx_buff, 24 );


        // Task delay
        vTaskDelayUntil( &xLastWakeTime, dt );
    }

}

/* ========================================================================
 * CONTROLLER TEST TASK
 * ========================================================================
 */
void ctrlTestTask( void *pvParameters )
{
    /*
     *  This task is used to:
     *      1. Read LIP state variables defined as global, these are:
     *          Cart position:      cart_position[ 0 ]
     *          Cart speed:         D_cart_position
     *          Pendulum angle:     angle[ 0 ]
     *          Pendulum speed:     D_angle
     *      2. Calculate control signal for inverted pendulum - dc motor voltage.
     *
     *  This task runs every 10ms
     */

    TickType_t xLastWakeTime = xTaskGetTickCount(); // For RTOS vTaskDelayUntil()

    // // state feedback gains:

    // dla dół zwykły lqr
    float xw_gain   = -60.42;
    float the_gain  = -6.115;
    float Dxw_gain  = -0.0;
    float Dthe_gain = -2.456;

    // dla góra zwykły lqr - nie działa
    // float xw_gain   = 30.5148F;
    // float the_gain  = 140.6046;
    // float Dxw_gain  = 0.0F;
    // float Dthe_gain = 0.0F;

    // dla dół stan x + u
    // 28.2843    0.7116   -2.0411    0.4627    1.8406
    // float xw_gain   = -28.2843 ;
    // float the_gain  = -0.7116;
    // float Dxw_gain  = -2.0411;
    // float Dthe_gain = 0.4627;
    // float u_gain    = 1.8406;

    float ctrl_signal = 0;
    float I_ctrl_signal[ 2 ] = {0};

    float cart_position_m;
    float angle_setpoint = PI;
    float up_position_controller_treshold = 25.0*PI/180.0f;

    for( ;; )
    {

        // Calculate cart position setpoint;
        cart_position_setpoint = (float) adc_data_pot / 4096.0f * 47.0f * 0.01;

        // Convert cart position from cm to m
        cart_position_m = cart_position[0] * 0.01;

        // controller update - pozycja dół - działa
        // ( non_lin_state - setpoint_state ) .* ( state_feedback_gains )
        ctrl_signal = ( cart_position_m - cart_position_setpoint )          * xw_gain +
                      ( angle[ 0 ] - PI )                                   * the_gain +
                      D_cart_position                                       * Dxw_gain +
                      D_angle                                               * Dthe_gain;

        // // controller update - pozycja góra - nie działa
        // if( ( angle[ 0 ] < up_position_controller_treshold ) &&
        //     ( angle[ 0 ] > -up_position_controller_treshold ) &&
        //     ( cart_position[0] > 5.0f ) &&
        //     ( cart_position[0] < 42.0f ) )
        // {
        //     ctrl_signal[ 0 ] = ( cart_position_m - cart_position_setpoint )          * xw_gain +
        //                        ( angle[ 0 ] - angle_setpoint )                       * the_gain +
        //                        D_cart_position                                       * Dxw_gain +
        //                        D_angle                                               * Dthe_gain;
        // }
        // else
        // {
        //     ctrl_signal[ 0 ] = 0;
        // }

        // // regulator na oko na dół
        // ctrl_signal[ 0 ] = ( cart_position_setpoint - cart_position_m )    * 100.0f +
        //                    ( PI - angle[ 0 ] )                             * 20.0f;

        if( ctrl_signal < 1.0f && ctrl_signal > -1.0f )
        {
            ctrl_signal = 0;
        }
        // else if ( ctrl_signal > 9.0f  )
        // {
        //     ctrl_signal = 9.0f;
        // }
        // else if ( ctrl_signal < -9.0f )
        // {
        //     ctrl_signal = -9.0f;
        // }

        dcm_set_output_volatage( ctrl_signal );
        // ----------------------------------------------------------------------------------------------

        // // regulator na dół stan + u
        // ctrl_signal = ( cart_position_m - cart_position_setpoint )             * xw_gain +
        //               ( angle[ 0 ] - PI )                                      * the_gain +
        //               D_cart_position                                          * Dxw_gain +
        //               D_angle                                                  * Dthe_gain +
        //               I_ctrl_signal[0]                                         * u_gain;

        // I_ctrl_signal[ 0 ] = I_ctrl_signal[ 1 ] + ctrl_signal * 0.01;
        // I_ctrl_signal[ 1 ] = I_ctrl_signal[ 0 ];

        // if( I_ctrl_signal < 1.0f && I_ctrl_signal > -1.0f )
        // {
        //     I_ctrl_signal = 0;
        // }
        // else if ( ctrl_signal > 9.0f  )
        // {
        //     ctrl_signal = 9.0f;
        // }
        // else if ( ctrl_signal < -9.0f )
        // {
        //     ctrl_signal = -9.0f;
        // }

        // dcm_set_output_volatage( I_ctrl_signal[ 0 ] );

        vTaskDelayUntil( &xLastWakeTime, dt );
    }

}

/* ========================================================================
 * CONSOLE TASK
 * ========================================================================
 */
// Source: https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_CLI/FreeRTOS_Plus_CLI_IO_Interfacing_and_Task.html
void vCommandConsoleTask( void *pvParameters )
{
    int8_t cInputIndex = 0;
    BaseType_t xMoreDataToFollow;
    /* The input and output buffers are declared static to keep them off the stack. */
    static int8_t pcOutputString[ MAX_OUTPUT_LENGTH ], pcInputString[ MAX_INPUT_LENGTH ];

    vRegisterCLICommands();

    printf( "============ FreeRTOS CLI ============ \r\n" );

    for( ;; )
    {
        if ( cRxedChar != 0x00 ) // better to use notification here
        {
            if( cRxedChar == '\n' || cRxedChar == '\r' )
            {
                printf("\r\n");
                fflush(stdout);

                /* The command interpreter is called repeatedly until it returns
                pdFALSE.  See the "Implementing a command" documentation for an
                exaplanation of why this is. */
                do
                {
                    /* Send the command string to the command interpreter.  Any
                    output generated by the command interpreter will be placed in the
                    pcOutputString buffer. */
                    xMoreDataToFollow = FreeRTOS_CLIProcessCommand
                                (
                                    pcInputString,   /* The command string.*/
                                    pcOutputString,  /* The output buffer. */
                                    MAX_OUTPUT_LENGTH/* The size of the output buffer. */
                                );

                    for ( int i = 0; i < ( xMoreDataToFollow == pdTRUE ? MAX_OUTPUT_LENGTH : strlen( ( const char * ) pcOutputString ) ); i++ )
                    {
                        printf( "%c", *(pcOutputString + i) );
                        fflush(stdout);
                    }

                } while( xMoreDataToFollow != pdFALSE );

                cInputIndex = 0;
                memset( pcInputString, 0x00, MAX_INPUT_LENGTH );
                memset( pcOutputString, 0x00, MAX_INPUT_LENGTH );
            }
            else
            {
                /* The if() clause performs the processing after a newline character
                is received.  This else clause performs the processing if any other
                character is received. */

                if( cRxedChar ==  '\b' )
                {
                    /* Backspace was pressed.  Erase the last character in the input
                    buffer - if there are any. */
                    if( cInputIndex > 0 )
                    {
                        cInputIndex--;
                        pcInputString[ cInputIndex ] = '\0';
                    }
                }
                else
                {
                    /* A character was entered.  It was not a new line, backspace
                    or carriage return, so it is accepted as part of the input and
                    placed into the input buffer.  When a n is entered the complete
                    string will be passed to the command interpreter. */
                    if( cInputIndex < MAX_INPUT_LENGTH )
                    {
                        pcInputString[ cInputIndex ] = cRxedChar;
                        cInputIndex++;
                        printf( "%c", cRxedChar );
                        fflush( stdout );
                    }
                }
            }
            cRxedChar = 0x00;
            fflush( stdout );
        } // if (cRxedChar != 0x00)
    } // for( ;; )
} //vCommandConsoleTask
