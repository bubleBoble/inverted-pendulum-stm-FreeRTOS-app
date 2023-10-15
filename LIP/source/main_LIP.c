/*
*   Opis
*/
#include "main_LIP.h"

/* For limit switch interrupts */
#define READ_ZERO_POSITION_REACHED HAL_GPIO_ReadPin(limitSW_left_GPIO_Port, limitSW_left_Pin)
#define READ_MAX_POSITION_REACHED HAL_GPIO_ReadPin(limitSW_right_GPIO_Port, limitSW_right_Pin)

/* holds each byte received from console (uart3) */
uint8_t cRxedChar = 0x00;

extern UART_HandleTypeDef huart3;

/* stdio reroute */
int _write(int file, char *ptr, int len) 
{
    HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
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
StackType_t startTask_STACKBUFFER [ STARTTASK_STACKDEPTH ];
StaticTask_t startTask_TASKBUFFER_TCB;

/* console task */
#define MAX_INPUT_LENGTH    50
#define MAX_OUTPUT_LENGTH   100
#define CONSOLE_STACKDEPTH  4000
TaskHandle_t consoleTaskHandle;
void vCommandConsoleTask( void *pvParameters );
StackType_t console_STACKBUFFER [ CONSOLE_STACKDEPTH ];
StaticTask_t console_TASKBUFFER_TCB;
TickType_t time_at_which_consoleMutex_was_taken;
// SemaphoreHandle_t xConsoleWriteMutex; // mutex for console writing

/* Encoders test task */
#define ENC_TEST_STACK_DEPTH 500
TaskHandle_t encTestTaskHandle = NULL;
void encTestTask( void *pvParameters );
StackType_t encTestTask_STACKBUFFER [ ENC_TEST_STACK_DEPTH ];
StaticTask_t encTestTask_TASKBUFFER_TCB;

/* --------------------- REMOVE - RAMPA TEST --------------------- */
#define RAMPA_TASK_STACK_DEPTH 500
TaskHandle_t rampaTaskHandle = NULL;
void rampaTask( void *pvParameters );
StackType_t rampaTask_STACKBUFFER [ RAMPA_TASK_STACK_DEPTH ];
StaticTask_t rampaTask_TASKBUFFER_TCB;
/* --------------------- REMOVE - RAMPA TEST --------------------- */

/* ========================================================================
 * LIP INIT & RUN
 * ========================================================================
 */
void main_LIP_init(void)
{
	SCB->CPACR |= ((3 << 10*2)|(3 << 11*2)); // FPU initialization
                                             // FPU must be enabled before any FPU
                                             // instruction is executed, otherwise 
                                             // hardware exception will be raised.
    dcm_init_output_voltage();               // Initialize PWM timer and zero its PWM output    
    enc_init();                              // Initialize encoder timer
    pend_enc_init();                         // Initialize AS5600 encoder 
}
void main_LIP_run(void)
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

/* --------------------- REMOVE - RAMPA TEST --------------------- */
uint8_t RAMPA_START=0;
/* --------------------- REMOVE - RAMPA TEST --------------------- */

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
    if (GPIO_Pin == limitSW_left_Pin) // limit switch left
    {
        ZERO_POSITION_REACHED = 1;
        MAX_POSITION_REACHED = 0;
        dcm_set_output_volatage(0.0f);
        dcm_enc_zero_counter();
    }
    if (GPIO_Pin == limitSW_right_Pin) // limit switch right
    {
        MAX_POSITION_REACHED = 1;
        ZERO_POSITION_REACHED = 0;
        dcm_set_output_volatage(0.0f);
    }
    if (GPIO_Pin == blue_btn_Pin) // built in blue button
    {
        if (ZERO_POSITION_REACHED)              // go to the max position if trolley on the zero 
        {
            // dcm_set_output_volatage(2.0f);
            /* --------------------- REMOVE - RAMPA TEST --------------------- */
            RAMPA_START=1;
            /* --------------------- REMOVE - RAMPA TEST --------------------- */
        } else if (MAX_POSITION_REACHED)        // go to the zero position if trolley on the max 
        { 
            // dcm_set_output_volatage(-2.0f);
            /* --------------------- REMOVE - RAMPA TEST --------------------- */
            RAMPA_START=0;
            /* --------------------- REMOVE - RAMPA TEST --------------------- */
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

    encTestTaskHandle = xTaskCreateStatic( encTestTask,
                                           (const char*) "magnetic encoder task",
                                           ENC_TEST_STACK_DEPTH,
                                           (void *) 0,
                                           tskIDLE_PRIORITY+2,
                                           encTestTask_STACKBUFFER,
                                           &encTestTask_TASKBUFFER_TCB );

    /* --------------------- REMOVE - RAMPA TEST --------------------- */
    rampaTaskHandle = xTaskCreateStatic( rampaTask,
                                         (const char*) "rampatest",
                                         RAMPA_TASK_STACK_DEPTH,
                                         (void *) 0,
                                         tskIDLE_PRIORITY+2,
                                         rampaTask_STACKBUFFER,
                                         &rampaTask_TASKBUFFER_TCB );
    /* --------------------- REMOVE - RAMPA TEST --------------------- */
}

/* ========================================================================
 * TASKS
 * ========================================================================
 */
void startTask(void * pvParameters)
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

#define dt      10
#define dt_inv  100.0f
void encTestTask( void *pvParameters )
{
    /*
        This task is used to test encoder functionality

        Poll the pnedulum encoder at least 3 times per full revolution

        saving previous sample readings convention:
            reading [0] : reading [n]     : current sample (n)
            reading [1] : reading [n - 1] : previous sample (n-1)
            etc.
    */

    TickType_t xLastWakeTime = xTaskGetTickCount(); // For RTOS vTaskDelayUntil()

    // Pendulum magnetic encoder reading 
    float angle[2] = {0.0f};                             // Angle current & previous sample
    float D_angle;                                       // Angle derivative
    FIR_filter low_pass_FIR_pend;                        // Low pas filter for angle derivative
    float filter_coeffs_pend[ FIR_BUFF_LEN ] = FIR_1;    // FIR_1 is #define in filtePrs_coeffs.h
    FIR_init( &low_pass_FIR_pend, filter_coeffs_pend );

    // DCM encoder reading
    float trolley_position[2] = {0.0f};
    float D_trolley_position;
    FIR_filter low_pass_FIR_dcm;
    float filter_coeffs_dcm[ FIR_BUFF_LEN ] = FIR_1;
    FIR_init( &low_pass_FIR_dcm, filter_coeffs_dcm );

    char msg[128]; // msg for uart data transfer
    float voltage_setting;

    com_send("xw,Dxw,DxwF,a,Da,DaF,V,T\r\n", 24);
    vTaskDelay(1000);

    for (;;)
    { 
        // Pendulum magnetic encoder reading
        angle[1] = angle[0];
        angle[0] = (float) pend_enc_get_cumulative_count() / 4096.0f * 360.0f;
        
        D_angle = ( angle[0] - angle[1] ) * dt_inv; // Pend. angle first derivative (wrt time)
        FIR_update( &low_pass_FIR_pend, D_angle );

        // DCM encoder reading
        trolley_position[1] = trolley_position[0];
        trolley_position[0] = dcm_enc_get_trolley_position_cm();
        
        D_trolley_position = ( trolley_position[0] - trolley_position[1] ) * dt_inv; // 1st deriv
        FIR_update( &low_pass_FIR_dcm, D_trolley_position );

        voltage_setting = dcm_get_output_voltage();

        /* LOGGING SCENARIO 1 */
        // Cast to dobule to remove warning about implicit cast to double
        sprintf( msg, "%f,%f,%f,%f,%f,%f,%f,%ld\r\n", 
            (double)trolley_position[0], (double)D_trolley_position, (double)low_pass_FIR_dcm.out,
            (double)angle[0],            (double)D_angle           , (double)low_pass_FIR_pend.out,
            (double)voltage_setting, xLastWakeTime );
        com_send( msg, strlen(msg) );

        /* LOGGING SCENARIO 2 */
        // sprintf( msg, "%f,%f,%f,%f,%f,%f,%f,%ld\r\n", 
        //     (double)trolley_position[0], (double)angle[0], (double)voltage_setting, xLastWakeTime );
        // com_send( msg, strlen(msg) );

        vTaskDelayUntil( &xLastWakeTime, dt );
    }
}

#include "math.h"
/* --------------------- REMOVE - RAMPA TEST --------------------- */
void rampaTask( void *pvParameters )
{
    uint16_t i=0;

    TickType_t starttime = xTaskGetTickCount();
    uint32_t time = 0;
    float vol_out = 0;

    for (;;)
    {
        if (RAMPA_START)
        {
            vol_out = 6.0f * sinf((float)time * 0.001f * PI2 * 0.70f) + sinf((float)time * 0.001f * PI2 * 1.4f);
        }
        else
        {
            vol_out = 0.0f;
        }
        dcm_set_output_volatage(vol_out);
        time = xTaskGetTickCount()-starttime;

        vTaskDelay(10);
    }
}
/* --------------------- REMOVE - RAMPA TEST --------------------- */




/* ------------------------------------------------------------ */
/* --------------------- TASK FOR CONSOLE --------------------- */
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
                        printf("%c", cRxedChar);
                        fflush(stdout);            
                    }
                }
            }
            cRxedChar = 0x00;
            fflush(stdout);
        } // if (cRxedChar != 0x00)
    } // for( ;; )
} //vCommandConsoleTask
