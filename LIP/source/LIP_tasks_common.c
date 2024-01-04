/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides global variables used by more than one task,
 * All handles to tasks, its stackbuffers and its taskbuffers TCBs,
 * definition of LIPcreateTasks function, which creates all app tasks.
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
float pend_speed_raw[ 2 ] = { 0.0f };   // Angle derivative
float pend_speed[ 2 ] = { 0.0f };
// IIR_filter low_pass_IIR_pend;
LP_filter low_pass_IIR_pend;

/* These are made global but only basic_test_task will write to them
Only controller_task should read them
DCM encoder reading */
float cart_position[ 2 ] = { 0.0f };
float cart_speed_raw[ 2 ] = { 0.0f };
float cart_speed[ 2 ] = { 0.0f };
// IIR_filter low_pass_IIR_cart;
LP_filter low_pass_IIR_cart;

/* Cart position setpoint from adc reading, converetd to [0, 40.7] range in cm.
[ 0 ] is current, [ 1 ] is previous sample. */
float cart_position_setpoint_cm_pot_raw; // raw read
float cart_position_setpoint_cm_pot;     // low pass filtered

/* Cart position setpoint set by cli command, range [0, 47] in cm.
[ 0 ] is current, [ 1 ] is previous sample. */
float cart_position_setpoint_cm_cli_raw = TRACK_LEN_MAX_CM/2.0f;
float cart_position_setpoint_cm_cli     = TRACK_LEN_MAX_CM/2.0f; // low pass filtered

/* This variable points to cart position setpoint from selected source, so either
cart_position_setpoint_cm_pot or cart_position_setpoint_cm_cli. This setpoint is used
by controllers. By default it points to setpoint set from cli by "spcli" command. */
float *cart_position_setpoint_cm = &cart_position_setpoint_cm_cli;

/* Setpoint for pendulum arm angle for DPC and UPC. */
float pendulum_arm_angle_setpoint_rad_upc;
float pendulum_arm_angle_setpoint_rad_dpc;

/* Holds number of pendulum full revolutions, negative number indicates
full revolution in counter clockwise direction. */
float number_of_pendulumarm_revolutions_dpc; // for dpc
float number_of_pendulumarm_revolutions_upc; // for upc

/* Pendulum arm angle in base range [0, 2pi]. */
float pendulum_angle_in_base_range_dpc; // for dpc
float pendulum_angle_in_base_range_upc; // for upc

/* Pendulum magnetic encoder reading at down position. The default pendulum
position is assumed to be down position, from control/model perspective, down
position corresponds to PI radians, so PI has to be subtracted from initial reading and
resultant value is offset that has to subtracted from each angle reading. */
float pend_init_angle_offset;

/* lip_app_states enum instance, which indicates current LIP app state. */
enum lip_app_states app_current_state; 

/* Cart position calibrated flag indicated that cart has real position match cart sensor reading. 
1 - calibrated,
0 - not calibrated. */
uint8_t cart_position_calibrated = 0;

/* cart_position_zones enum instance, which indicates current cart position zone. */
enum cart_position_zones cart_current_zone;

/* This flag indicates that bounceoff task was resumed and is probably running. */
uint32_t bounceoff_resumed = 0;

/* This flag indicates that bounce off action on track min/max is on. */
uint32_t bounce_off_action_on = 0;

/* This flag indicates that the swingup task is running. */
uint32_t swingup_task_resumed = 0;

/* Used as global flag to indicate that lookup_index in swingup task should be reset to zero. 
This have to be done on each call to cli command "swingup". */
uint32_t reset_lookup_index = 0;

/* Global flag to signal that swingup task has to be reset.
Reset meaning start swingup procedure from the very begining, it doesn't reset the task itself.
Should be set to 1 with every call to cli command "swingup". */
uint32_t reset_swingdown = 0;

/* Global flag to signal that home command should be reset.
Reset meaning start "home" command procedure from the very begining, it doesn't reset the task itself. 
Should be set to 1 with every call to cli command "swingup". */
uint32_t reset_home = 0;

// /* Global flag to signal that test command should be reset.
// Reset meaning start "test" command procedure from the very begining, it doesn't reset the task itself. */
// uint32_t reset_test = 0;

/* These are used as global variables to hold four control signal components from 
active controller task. COM_SEND_CTRL_DEBUG is defined in main_LIP.c */
#ifdef COM_SEND_CTRL_DEBUG
    float ctrl_xw = 0.0f;
    float ctrl_th = 0.0f;
    float ctrl_Dx = 0.0f;
    float ctrl_Dt = 0.0f;
#endif

/* Watchdog task - protection for cart min and max positions and default always running task. */
TaskHandle_t watchdog_taskHandle = NULL;
StackType_t WATCHDOG_STACKBUFFER[ WATCHDOG_STACK_DEPTH ];
StaticTask_t WATCHDOG_TASKBUFFER_TCB;

/* Console task */
TaskHandle_t console_taskHandle;
StackType_t console_STACKBUFFER[ CONSOLE_STACKDEPTH ];
StaticTask_t console_TASKBUFFER_TCB;
TickType_t time_at_which_consoleMutex_was_taken;

/* State estimation task */
TaskHandle_t util_taskHandle = NULL;
StackType_t utilTask_STACKBUFFER [ UTIL_STACK_DEPTH ];
StaticTask_t utilTask_TASKBUFFER_TCB;

/* Communication task. */
TaskHandle_t com_taskHandle = NULL;
StackType_t COM_STACKBUFFER [ COM_STACK_DEPTH ];
StaticTask_t COM_TASKBUFFER_TCB;

/* Communication task - for raw bytes transmission. */
TaskHandle_t rawcom_taskHandle = NULL;
StackType_t RAWCOM_STACKBUFFER [ RAWCOM_STACK_DEPTH ];
StaticTask_t RAWCOM_TASKBUFFER_TCB;

/* Worker task - this task is only active when command "zero" or "home" is called.
Its purpose is to move the cart without any controller. */
TaskHandle_t cartworker_TaskHandle = NULL;
StackType_t CARTWORKER_STACKBUFFER [ CARTWORKER_STACK_DEPTH ];
StaticTask_t CARTWORKER_TASKBUFFER_TCB;

/* Down-position controller
Full state feedback down position with better deadzone compensation. */
TaskHandle_t ctrl_downposition_taskHandle = NULL;
StackType_t ctrl_downposition_STACKBUFFER [ CTRL_3_FSF_DOWNPOS_STACK_DEPTH ];
StaticTask_t ctrl_downposition_TASKBUFFER_TCB;
// StackType_t ctrl_3_FSF_downpos_STACKBUFFER [ CTRL_3_FSF_DOWNPOS_STACK_DEPTH ];
// StaticTask_t ctrl_3_FSF_downpos_TASKBUFFER_TCB;

/* Up-position controller
Full state feedback up position with deadzone compensation. */
TaskHandle_t ctrl_upposition_taskHandle = NULL;
StackType_t ctrl_upposition_STACKBUFFER [ CTRL_5_FSF_UPPOS_STACK_DEPTH ];
StaticTask_t ctrl_upposition_TASKBUFFER_TCB;
// TaskHandle_t ctrl_5_FSF_uppos_task_handle = NULL;
// StackType_t ctrl_5_FSF_uppos_STACKBUFFER [ CTRL_5_FSF_UPPOS_STACK_DEPTH ];
// StaticTask_t ctrl_5_FSF_uppos_TASKBUFFER_TCB;

/* Swingup, trajopt. */
TaskHandle_t swingup_task_handle = NULL;
StackType_t swingup_STACKBUFFER [ SWINGUP_STACK_DEPTH ];
StaticTask_t swingup_TASKBUFFER_TCB;

/* Swingdown */
TaskHandle_t swingdown_task_handle = NULL;
StackType_t swingdown_STACKBUFFER [ SWINGDOWN_STACK_DEPTH ];
StaticTask_t swingdown_TASKBUFFER_TCB;

/* Bounce off task */
TaskHandle_t bounceoff_task_handle = NULL;
StackType_t bounceoff_STACKBUFFER [ BOUNCEOFF_STACK_DEPTH ];
StaticTask_t bounceoff_TASKBUFFER_TCB;

/* Test task. */
TaskHandle_t test_task_handle = NULL;
StackType_t test_STACKBUFFER [ TEST_STACK_DEPTH ];
StaticTask_t test_TASKBUFFER_TCB;


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* CREATE TASKS. */
void LIPcreateTasks()
{
    watchdog_taskHandle = xTaskCreateStatic( watchdogTask,
                                             (const char*) "Watchdog",
                                             WATCHDOG_STACK_DEPTH,
                                             (void *) 0,
                                             tskIDLE_PRIORITY+PRIORITY_WATCHDOG,
                                             WATCHDOG_STACKBUFFER,
                                             &WATCHDOG_TASKBUFFER_TCB );

    /* Task that implements FreeRTOS console functionality */
    console_taskHandle = xTaskCreateStatic( vCommandConsoleTask,
                                            (const char*) "Console",
                                            CONSOLE_STACKDEPTH,
                                            (void *) 0,
                                            tskIDLE_PRIORITY+PRIORITY_CONSOLE,
                                            console_STACKBUFFER,
                                            &console_TASKBUFFER_TCB );

    /* State variables are calculated here. */
    util_taskHandle = xTaskCreateStatic( utilTask,
                                         (const char*) "Util",
                                         UTIL_STACK_DEPTH,
                                         (void *) 0,
                                         tskIDLE_PRIORITY+PRIORITY_UTIL,
                                         utilTask_STACKBUFFER,
                                         &utilTask_TASKBUFFER_TCB );


    /* Human readable communication - for serialoscilloscope. */
    com_taskHandle = xTaskCreateStatic( comTask,
                                        (const char*) "Communication",
                                        COM_STACK_DEPTH,
                                        (void *) 0,
                                        tskIDLE_PRIORITY+PRIORITY_COM,
                                        COM_STACKBUFFER,
                                        &COM_TASKBUFFER_TCB );
    /* Data streaming is suspended right after task creation */
    vTaskSuspend( com_taskHandle );

    /* Worker task - this tas is only active when command "zero" or "home" are called.
    Its purpose is to move the cart without any controller. */
    cartworker_TaskHandle = xTaskCreateStatic( cartWorkerTask,
                                               (const char*) "CartWorker",
                                               CARTWORKER_STACK_DEPTH,
                                               (void *) 0,
                                               tskIDLE_PRIORITY+PRIORITY_CARTWORKER,
                                               CARTWORKER_STACKBUFFER,
                                               &CARTWORKER_TASKBUFFER_TCB );

    swingup_task_handle = xTaskCreateStatic( swingup_task,
                                             ( const char * ) "SwingupOpt", 
                                             SWINGUP_STACK_DEPTH,
                                             ( void * ) 0,
                                             //  tskIDLE_PRIORITY+PRIORITY_CTRL,
                                             tskIDLE_PRIORITY+PRIORITY_CTRL,
                                             swingup_STACKBUFFER,
                                             &swingup_TASKBUFFER_TCB);
    vTaskSuspend( swingup_task_handle );

    swingdown_task_handle = xTaskCreateStatic( swingdown_task,
                                               ( const char * ) "Swingdown", 
                                               SWINGDOWN_STACK_DEPTH,
                                               ( void * ) 0,
                                               //  tskIDLE_PRIORITY+PRIORITY_CTRL,
                                               tskIDLE_PRIORITY+PRIORITY_CTRL,
                                               swingdown_STACKBUFFER,
                                               &swingdown_TASKBUFFER_TCB);
    vTaskSuspend( swingdown_task_handle );

    bounceoff_task_handle = xTaskCreateStatic( bounceoff_task,
                                               ( const char * ) "BounceOff", 
                                               BOUNCEOFF_STACK_DEPTH,
                                               ( void * ) 0,
                                               tskIDLE_PRIORITY+PRIORITY_CTRL,
                                               bounceoff_STACKBUFFER,
                                               &bounceoff_TASKBUFFER_TCB);
    /* Bounce off task is resumed only in case of emergency. */  
    vTaskSuspend( bounceoff_task_handle );

    /* Down position controller
    Full state feedback down position ctrl-er with "tanh switching" deadzone compensation 
    (nonlinear cart position gain). */
    ctrl_downposition_taskHandle = xTaskCreateStatic( ctrl_3_FSF_downpos_task,
                                                      ( const char* ) "DownPosCtrl",
                                                      CTRL_3_FSF_DOWNPOS_STACK_DEPTH,
                                                      ( void * ) 0,
                                                      tskIDLE_PRIORITY+PRIORITY_CTRL,
                                                      ctrl_downposition_STACKBUFFER,
                                                      &ctrl_downposition_TASKBUFFER_TCB );
    /* All controller tasks are suspended right after their creation. */  
    vTaskSuspend( ctrl_downposition_taskHandle );

    /* Up position controller
    Full state feedback up position ctrl-er with "tanh-switching" deadzone compensation
    (nonlinear cart position gain). */
    ctrl_upposition_taskHandle = xTaskCreateStatic( ctrl_5_FSF_uppos_task,
                                                    (const char*) "UpPosCtrl",
                                                    CTRL_5_FSF_UPPOS_STACK_DEPTH,
                                                    (void *) 0,
                                                    tskIDLE_PRIORITY+PRIORITY_CTRL,
                                                    ctrl_upposition_STACKBUFFER,
                                                    &ctrl_upposition_TASKBUFFER_TCB );
    /* All controller tasks are suspended right after their creation. */  
    vTaskSuspend( ctrl_upposition_taskHandle );

    /* Raw byte communication task. */
    // rawcom_taskHandle = xTaskCreateStatic( rawComTask,
    //                                       (const char*) "RawCommunicationTask",
    //                                       RAWCOM_STACK_DEPTH,
    //                                       (void *) 0,
    //                                       tskIDLE_PRIORITY + 2,
    //                                       RAWCOM_STACKBUFFER,
    //                                       &RAWCOM_TASKBUFFER_TCB);

    test_task_handle = xTaskCreateStatic( test_task,
                                          (const char*) "test",
                                          TEST_STACK_DEPTH,
                                          (void *) 0,
                                          tskIDLE_PRIORITY+PRIORITY_TEST,
                                          test_STACKBUFFER,
                                          &test_TASKBUFFER_TCB );
}