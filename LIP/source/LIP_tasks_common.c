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
float pend_speed[ 2 ] = { 0.0f };   // Angle derivative
IIR_filter low_pass_IIR_pend;

/* These are made global but only basic_test_task will write to them
Only controller_task should read them
DCM encoder reading */
float cart_position[ 2 ] = { 0.0f };
float cart_speed[ 2 ];
IIR_filter low_pass_IIR_cart;

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

/* These are used as global variables to hold four control signal components from 
active controller task. COM_SEND_CTRL_DEBUG is defined in main_LIP.c */
#ifdef COM_SEND_CTRL_DEBUG
    float ctrl_xw = 0.0f;
    float ctrl_th = 0.0f;
    float ctrl_Dx = 0.0f;
    float ctrl_Dt = 0.0f;
#endif

/* Watchdog task - protection for cart min and max positions and default always running task. */
TaskHandle_t watchdogTaskHandle = NULL;
StackType_t WATCHDOG_STACKBUFFER[ WATCHDOG_STACK_DEPTH ];
StaticTask_t WATCHDOG_TASKBUFFER_TCB;

/* Console task */
TaskHandle_t consoleTaskHandle;
StackType_t console_STACKBUFFER[ CONSOLE_STACKDEPTH ];
StaticTask_t console_TASKBUFFER_TCB;
TickType_t time_at_which_consoleMutex_was_taken;

/* State estimation task */
TaskHandle_t utilTaskHandle = NULL;
StackType_t utilTask_STACKBUFFER [ UTIL_STACK_DEPTH ];
StaticTask_t utilTask_TASKBUFFER_TCB;

/* Communication task. */
TaskHandle_t comTaskHandle = NULL;
StackType_t COM_STACKBUFFER [ COM_STACK_DEPTH ];
StaticTask_t COM_TASKBUFFER_TCB;

/* Communication task - for raw bytes transmission. */
TaskHandle_t rawComTaskHandle = NULL;
StackType_t RAWCOM_STACKBUFFER [ RAWCOM_STACK_DEPTH ];
StaticTask_t RAWCOM_TASKBUFFER_TCB;

/* Worker task - this task is only active when command "zero" or "home" is called.
Its purpose is to move the cart without any controller. */
TaskHandle_t cartWorkerTaskHandle = NULL;
StackType_t CARTWORKER_STACKBUFFER [ CARTWORKER_STACK_DEPTH ];
StaticTask_t CARTWORKER_TASKBUFFER_TCB;

/* Controller 1 task 
Basic full state feedback down position, output voltage is zero in specified deadzone. */
TaskHandle_t ctrl_FSF_downpos_task_handle = NULL;
StackType_t ctrl_FSF_downpos_STACKBUFFER [ CTRL_FSF_DOWNPOS_STACK_DEPTH ];
StaticTask_t ctrl_FSF_downpos_TASKBUFFER_TCB;

/* Controller 2 task
Full state feedback down position with better deadzone compensation, 
nonlinear cart position gain "hard switching". */
TaskHandle_t ctrl_2_FSF_downpos_task_handle = NULL;
StackType_t ctrl_2_FSF_downpos_STACKBUFFER [ CTRL_2_FSF_DOWNPOS_STACK_DEPTH ];
StaticTask_t ctrl_2_FSF_downpos_TASKBUFFER_TCB;

/* Controller 3 task
Full state feedback down position with better deadzone compensation, 
nonlinear cart position gain "tanh switching". */
TaskHandle_t ctrl_3_FSF_downpos_task_handle = NULL;
StackType_t ctrl_3_FSF_downpos_STACKBUFFER [ CTRL_3_FSF_DOWNPOS_STACK_DEPTH ];
StaticTask_t ctrl_3_FSF_downpos_TASKBUFFER_TCB;

/* Controller 4 task
Full state feedback with integral action on cart position error */
TaskHandle_t ctrl_4_I_FSF_downpos_task_handle = NULL;
StackType_t ctrl_4_I_FSF_downpos_STACKBUFFER [ CTRL_4_I_FSF_DOWNPOS_STACK_DEPTH ];
StaticTask_t ctrl_4_I_FSF_downpos_TASKBUFFER_TCB;

/* Controller 5 task
Full state feedback up position with deadzone compensation, 
nonlinear cart position gain "tanh switching". */
TaskHandle_t ctrl_5_FSF_uppos_task_handle = NULL;
StackType_t ctrl_5_FSF_uppos_STACKBUFFER [ CTRL_5_FSF_UPPOS_STACK_DEPTH ];
StaticTask_t ctrl_5_FSF_uppos_TASKBUFFER_TCB;

/* Controller 6 task
Full state feedback with integral action, up position with deadzone compensation, 
nonlinear cart position gain "tanh switching". */
TaskHandle_t ctrl_6_I_FSF_uppos_task_handle = NULL;
StackType_t ctrl_6_I_FSF_uppos_STACKBUFFER [ CTRL_6_I_FSF_UPPOS_STACK_DEPTH ];
StaticTask_t ctrl_6_I_FSF_uppos_TASKBUFFER_TCB;

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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* CREATE TASKS. */
void LIPcreateTasks()
{
    watchdogTaskHandle = xTaskCreateStatic( watchdogTask,
                                            (const char*) "Watchdog",
                                            WATCHDOG_STACK_DEPTH,
                                            (void *) 0,
                                            tskIDLE_PRIORITY+PRIORITY_WATCHDOG,
                                            WATCHDOG_STACKBUFFER,
                                            &WATCHDOG_TASKBUFFER_TCB );

    /* Task that implements FreeRTOS console functionality */
    consoleTaskHandle = xTaskCreateStatic( vCommandConsoleTask,
                                           (const char*) "Console",
                                           CONSOLE_STACKDEPTH,
                                           (void *) 0,
                                           tskIDLE_PRIORITY+PRIORITY_CONSOLE,
                                           console_STACKBUFFER,
                                           &console_TASKBUFFER_TCB );

    /* State variables are calculated here. */
    utilTaskHandle = xTaskCreateStatic( utilTask,
                                        (const char*) "Util",
                                        UTIL_STACK_DEPTH,
                                        (void *) 0,
                                        tskIDLE_PRIORITY+PRIORITY_UTIL,
                                        utilTask_STACKBUFFER,
                                        &utilTask_TASKBUFFER_TCB );


    /* Human readable communication - for serialoscilloscope. */
    comTaskHandle = xTaskCreateStatic( comTask,
                                       (const char*) "Communication",
                                       COM_STACK_DEPTH,
                                       (void *) 0,
                                       tskIDLE_PRIORITY+PRIORITY_COM,
                                       COM_STACKBUFFER,
                                       &COM_TASKBUFFER_TCB );
    /* Data streaming is suspended right after task creation */
    vTaskSuspend( comTaskHandle );

    /* Worker task - this tas is only active when command "zero" or "home" are called.
    Its purpose is to move the cart without any controller. */
    cartWorkerTaskHandle = xTaskCreateStatic( cartWorkerTask,
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

    /* Down position controller 1
    Full state feedback down position ctrl-er with "tanh switching" deadzone compensation 
    (nonlinear cart position gain). */
    ctrl_3_FSF_downpos_task_handle = xTaskCreateStatic( ctrl_3_FSF_downpos_task,
                                                        ( const char* ) "DownPosCtrl",
                                                        CTRL_3_FSF_DOWNPOS_STACK_DEPTH,
                                                        ( void * ) 0,
                                                        tskIDLE_PRIORITY+PRIORITY_CTRL,
                                                        ctrl_3_FSF_downpos_STACKBUFFER,
                                                        &ctrl_3_FSF_downpos_TASKBUFFER_TCB );
    /* All controller tasks are suspended right after their creation. */  
    vTaskSuspend( ctrl_3_FSF_downpos_task_handle );
    
    /* Down position controller 2
    Full state feedback with integral action on cart position error */
    ctrl_4_I_FSF_downpos_task_handle = xTaskCreateStatic( ctrl_4_I_FSF_downpos_task,
                                                          (const char*) "DownPosCtrlInt",
                                                          CTRL_4_I_FSF_DOWNPOS_STACK_DEPTH,
                                                          (void *) 0,
                                                          tskIDLE_PRIORITY+3,
                                                          ctrl_4_I_FSF_downpos_STACKBUFFER,
                                                          &ctrl_4_I_FSF_downpos_TASKBUFFER_TCB );
    /* All controller tasks are suspended right after their creation. */  
    vTaskSuspend( ctrl_4_I_FSF_downpos_task_handle );

    /* Up position controller 1
    Full state feedback up position ctrl-er with "tanh-switching" deadzone compensation
    (nonlinear cart position gain). */
    ctrl_5_FSF_uppos_task_handle = xTaskCreateStatic( ctrl_5_FSF_uppos_task,
                                                      (const char*) "UpPosCtrl",
                                                      CTRL_5_FSF_UPPOS_STACK_DEPTH,
                                                      (void *) 0,
                                                      tskIDLE_PRIORITY+3,
                                                      ctrl_5_FSF_uppos_STACKBUFFER,
                                                      &ctrl_5_FSF_uppos_TASKBUFFER_TCB );
    /* All controller tasks are suspended right after their creation. */  
    vTaskSuspend( ctrl_5_FSF_uppos_task_handle );

    /* Up position controller 2
    Full state feedback up position ctrl-er with "tanh-switching" deadzone compensation
    (nonlinear cart position gain), with integral action on cart position. */
    ctrl_6_I_FSF_uppos_task_handle = xTaskCreateStatic( ctrl_6_I_FSF_uppos_task,
                                                      (const char*) "UpPosCtrlInt",
                                                      CTRL_6_I_FSF_UPPOS_STACK_DEPTH,
                                                      (void *) 0,
                                                      tskIDLE_PRIORITY+3,
                                                      ctrl_6_I_FSF_uppos_STACKBUFFER,
                                                      &ctrl_6_I_FSF_uppos_TASKBUFFER_TCB );
    /* All controller tasks are suspended right after their creation. */  
    vTaskSuspend( ctrl_6_I_FSF_uppos_task_handle );

    /* Raw byte communication task. */
    // rawComTaskHandle = xTaskCreateStatic( rawComTask,
    //                                       (const char*) "RawCommunicationTask",
    //                                       RAWCOM_STACK_DEPTH,
    //                                       (void *) 0,
    //                                       tskIDLE_PRIORITY + 2,
    //                                       RAWCOM_STACKBUFFER,
    //                                       &RAWCOM_TASKBUFFER_TCB);

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
    /* Down position controllers test tasks */
    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
    /* Controller 1 task 
    Basic full state feedback down position, output voltage is zero in specified deadzone. */
    // ctrl_FSF_downpos_task_handle = xTaskCreateStatic( ctrl_FSF_downpos_task,
    //                                                   (const char*) "ControllerDownPosTask",
    //                                                   CTRL_FSF_DOWNPOS_STACK_DEPTH,
    //                                                   (void *) 0,
    //                                                   tskIDLE_PRIORITY+3,
    //                                                   ctrl_FSF_downpos_STACKBUFFER,
    //                                                   &ctrl_FSF_downpos_TASKBUFFER_TCB );

    /* Controller 2 task
    Full state feedback down position with better deadzone compensation, 
    nonlinear cart position gain "hard switching". */
    // ctrl_2_FSF_downpos_task_handle = xTaskCreateStatic( ctrl_2_FSF_downpos_task,
    //                                                   (const char*) "Controller2DownPosTask",
    //                                                   CTRL_2_FSF_DOWNPOS_STACK_DEPTH,
    //                                                   (void *) 0,
    //                                                   tskIDLE_PRIORITY+3,
    //                                                   ctrl_2_FSF_downpos_STACKBUFFER,
    //                                                   &ctrl_2_FSF_downpos_TASKBUFFER_TCB );

    /* Controller 3 task
    Full state feedback down position with better deadzone compensation, 
    nonlinear cart position gain "tanh switching". */
    // ctrl_3_FSF_downpos_task_handle = xTaskCreateStatic( ctrl_3_FSF_downpos_task,
    //                                                   (const char*) "Controller3DownPosTask",
    //                                                   CTRL_3_FSF_DOWNPOS_STACK_DEPTH,
    //                                                   (void *) 0,
    //                                                   tskIDLE_PRIORITY+3,
    //                                                   ctrl_3_FSF_downpos_STACKBUFFER,
    //                                                   &ctrl_3_FSF_downpos_TASKBUFFER_TCB );
    // /* Controller task is suspended right after task creation */  
    // vTaskSuspend( ctrl_3_FSF_downpos_task_handle );

    /* Controller 4 task
    Full state feedback with integral action on cart position error */
    // ctrl_4_I_FSF_downpos_task_handle = xTaskCreateStatic( ctrl_4_I_FSF_downpos_task,
    //                                                       (const char*) "Controller4DownPosTask",
    //                                                       CTRL_4_I_FSF_DOWNPOS_STACK_DEPTH,
    //                                                       (void *) 0,
    //                                                       tskIDLE_PRIORITY+3,
    //                                                       ctrl_4_I_FSF_downpos_STACKBUFFER,
    //                                                       &ctrl_4_I_FSF_downpos_TASKBUFFER_TCB );

    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */    
    /* UP position controllers test tasks */
    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
    /* Controller 5 task
    Full state feedback up position with deadzone compensation, 
    nonlinear cart position gain "tanh switching". */
    // ctrl_5_FSF_uppos_task_handle = xTaskCreateStatic( ctrl_5_FSF_uppos_task,
    //                                                   (const char*) "Controller5UpPosTask",
    //                                                   CTRL_5_FSF_UPPOS_STACK_DEPTH,
    //                                                   (void *) 0,
    //                                                   tskIDLE_PRIORITY+3,
    //                                                   ctrl_5_FSF_uppos_STACKBUFFER,
    //                                                   &ctrl_5_FSF_uppos_TASKBUFFER_TCB );
    /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
}