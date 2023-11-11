/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides stuff related for LIP app tasks.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "main_LIP.h"

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

