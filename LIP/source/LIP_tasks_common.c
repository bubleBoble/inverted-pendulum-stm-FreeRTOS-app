/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides default LIPcreateTasks function that creates default tasks.
 * Global variables which are used by more than one task are defined here.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "LIP_tasks_common.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Globals used by all tasks.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
/* Holds data from ADC3 tranfered over DMA, init code is in motor_driver.c/dcm_init(). */
volatile uint16_t adc_data_pot;

/* holds each byte received from console (uart3) */
uint8_t cRxedChar = 0x00;

/* These are made global but only basic_test_task will write to these
Only controller_task should read these
Pendulum magnetic encoder reading. */
float pend_angle[ 2 ] = { 0.0f };   // Angle current & previous sample
float pend_speed;                   // Angle derivative
IIR_filter low_pass_IIR_pend;

/* These are made global but only basic_test_task will write to them
Only controller_task should read them
DCM encoder reading */
float cart_position[ 2 ] = { 0.0f };
float cart_speed;
IIR_filter low_pass_IIR_cart;

/* Cart position setpoint from adc reading, converetd to [0, 47] range in cm. */
float cart_position_setpoint_cm_pot;
/* Cart position setpoint set by cli command, range [0, 47] in cm. */
float cart_position_setpoint_cm_cli;
/* This variable points to cart position setpoint from selected source, so either
cart_position_setpoint_cm_pot or cart_position_setpoint_cm_cli. This setpoint is used
by controllers. By default it points to converted potentiometer reading. */
float *cart_position_setpoint_cm = &cart_position_setpoint_cm_pot;

/* Pendulum magnetic encoder reading at down position. Can sometimes be different if
the pendulum shaft is forced to rotate inside a bearings. The default pendulum
position is assumed to be down position, from control/model perspective down
position corresponds to PI radians, so PI has to be subtracted from initial reading and
resultant value is offset that has to subtracted from each angle reading */
float pend_init_angle_offset;
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Start task */
TaskHandle_t startTaskHandle = NULL;
StackType_t startTask_STACKBUFFER[ STARTTASK_STACKDEPTH ];
StaticTask_t startTask_TASKBUFFER_TCB;

/* Console task */
TaskHandle_t consoleTaskHandle;
StackType_t console_STACKBUFFER[ CONSOLE_STACKDEPTH ];
StaticTask_t console_TASKBUFFER_TCB;
TickType_t time_at_which_consoleMutex_was_taken;

/* State estimation task */
TaskHandle_t stateEstimationTaskHandle = NULL;
StackType_t stateEstimationTask_STACKBUFFER [ STATE_ESTIMATION_STACK_DEPTH ];
StaticTask_t stateEstimationTask_TASKBUFFER_TCB;

/* Controller test task */
TaskHandle_t ctrlFSFTaskHandle = NULL;
StackType_t ctrlFSF_STACKBUFFER [ CTRL_FSF_STACK_DEPTH ];
StaticTask_t ctrlFSF_TASKBUFFER_TCB;

/* Communication task. */
TaskHandle_t comTaskHandle = NULL;
StackType_t COM_STACKBUFFER [ COM_STACK_DEPTH ];
StaticTask_t COM_TASKBUFFER_TCB;

/* Communication task - for raw bytes transmission. */
TaskHandle_t rawComTaskHandle = NULL;
StackType_t RAWCOM_STACKBUFFER [ RAWCOM_STACK_DEPTH ];
StaticTask_t RAWCOM_TASKBUFFER_TCB;

/* CREATE TASKS
Should it be moved to main_LIP.c ? - yes if it will only create one task. */
void LIPcreateTasks()
{
    /* ?????????? Basic start task. ?????????? */
    // startTaskHandle = xTaskCreateStatic( startTask,
    //                                      (const char*) "elo",
    //                                      STARTTASK_STACKDEPTH,
    //                                      (void *) 0,
    //                                      tskIDLE_PRIORITY+1,
    //                                      startTask_STACKBUFFER,
    //                                      &startTask_TASKBUFFER_TCB );

    /* Task that implements FreeRTOS console functionality */
    // consoleTaskHandle = xTaskCreateStatic( vCommandConsoleTask,
    //                                        (const char*) "ConsoleTask",
    //                                        CONSOLE_STACKDEPTH,
    //                                        (void *) 0,
    //                                        tskIDLE_PRIORITY+1,
    //                                        console_STACKBUFFER,
    //                                        &console_TASKBUFFER_TCB );

    /* State variables are calculated here. */
    stateEstimationTaskHandle = xTaskCreateStatic( stateEstimationTask,
                                                   (const char*) "StateEstimTask",
                                                   STATE_ESTIMATION_STACK_DEPTH,
                                                   (void *) 0,
                                                   tskIDLE_PRIORITY+3,
                                                   stateEstimationTask_STACKBUFFER,
                                                   &stateEstimationTask_TASKBUFFER_TCB );


    /* Human readable communication - for serialoscilloscope. */
    comTaskHandle = xTaskCreateStatic( comTask,
                                       (const char*) "CommunicationTask",
                                       COM_STACK_DEPTH,
                                       (void *) 0,
                                       tskIDLE_PRIORITY+2,
                                       COM_STACKBUFFER,
                                       &COM_TASKBUFFER_TCB );

    /* Raw byte communication task. */
    // rawComTaskHandle = xTaskCreateStatic( rawComTask,
    //                                       (const char*) "RawCommunicationTask",
    //                                       RAWCOM_STACK_DEPTH,
    //                                       (void *) 0,
    //                                       tskIDLE_PRIORITY + 2,
    //                                       RAWCOM_STACKBUFFER,
    //                                       &RAWCOM_TASKBUFFER_TCB);
    
    /* Controller 1 task - Full state feedback. */
    ctrlFSFTaskHandle = xTaskCreateStatic( ctrlFSFTask,
                                           (const char*) "ControllerTask",
                                           CTRL_FSF_STACK_DEPTH,
                                           (void *) 0,
                                           tskIDLE_PRIORITY+3,
                                           ctrlFSF_STACKBUFFER,
                                           &ctrlFSF_TASKBUFFER_TCB );
}