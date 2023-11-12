/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides function prototypes and #defines related to LIP app tasks.
 * Also provides prototype of LIPcreateTasks fucntion
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include "main_LIP.h"

/* Enum which lists all implemented LIP app states. */
#ifndef LIP_APP_STATES_ENUM
#define LIP_APP_STATES_ENUM
enum lip_app_states 
{
    /* Uninitialized state: app state before call to "zero" cli command.
    This state indicates that current cart position reading is not correct,
    pendulum cart has to be moved to zero position.
    Available commands: zero, <enter_key> */
    UNITIALIZED,
    /* Default state: All sesnor readings should be correct in this app state.
    Down position controller can be started.
    Available commands: zero, home, dpc, sp, <enter_key> */
    DEFAULT,
    /* DPC state: Down Position Controller turned on.
    Available commands: home, dpc, spcli, sppot, sp, swingup, <enter_key> */
    DPC
};
#endif // LIP_APP_STATES_ENUM

/* Watchdog task - protection for cart min and max positions. */
void watchdogTask( void *pvParameters );
#define WATCHDOG_STACK_DEPTH 500

/* console task. */
void vCommandConsoleTask( void *pvParameters );
#define MAX_INPUT_LENGTH    50
#define MAX_OUTPUT_LENGTH   500
#define CONSOLE_STACKDEPTH  4000

/* util task (State estimation & setpoint calc. task). */
void utilTask( void *pvParameters );
#define UTIL_STACK_DEPTH 500

/* Communication task - for serialOscilloscope. */
void comTask( void *pvParameters );
#define COM_STACK_DEPTH 500

/* Communication task - for raw bytes transmission. */
void rawComTask( void *pvParameters );
#define RAWCOM_STACK_DEPTH 500

/* Worker task - this tas is only active when command "zero" or "home" are called.
Its purpose is to move the cart without any controller. */
void workerTask( void *pvParameters );
#define WORKER_STACK_DEPTH 500

/* Controllers tasks */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* Controller 1 task 
Basic full state feedback, pendulum down position, \
output voltage is zero in specified deadzone (pm 1V). */
void ctrl_FSF_downpos_task( void *pvParameters );
#define CTRL_FSF_DOWNPOS_STACK_DEPTH 500

/* Controller 2 task
Full state feedback with better deadzone compensation, 
pendulum down position, nonlinear cart position gain "hard switching". */
void ctrl_2_FSF_downpos_task( void *pvParameters );
#define CTRL_2_FSF_DOWNPOS_STACK_DEPTH 500

/* Controller 3 task - USE THIS ONE
Full state feedback Full state feedback with better deadzone compensation, 
pendulum down position, nonlinear cart position gain "tanh switching".
(smoothed hard switching) */
void ctrl_3_FSF_downpos_task( void *pvParameters );
#define CTRL_3_FSF_DOWNPOS_STACK_DEPTH 500

/* Controller 4 task
Full state feedback with integral action on cart position error,
pendulum down position. */
void ctrl_4_I_FSF_downpos_task( void *pvParameters );
#define CTRL_4_I_FSF_DOWNPOS_STACK_DEPTH 500

/* Controller 5 task
Full state feedback up position with deadzone compensation, 
nonlinear cart position gain "tanh switching". */
void ctrl_5_FSF_uppos_task( void *pvParameters );
#define CTRL_5_FSF_UPPOS_STACK_DEPTH 500
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Function to create tasks. */
void LIPcreateTasks(void);

