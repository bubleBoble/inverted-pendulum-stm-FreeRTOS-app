/*
*   Opis.
*/
#ifndef LIP_MAIN
#define LIP_MAIN

#include "stdarg.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS_CLI.h"
#include "semphr.h"
#include "list.h"
#include "stream_buffer.h"
#include "timers.h"

#include "stm32f4xx_hal.h"
#include "main.h"

#include "motor_driver.h"
#include "dcm_encoder_driver.h"
#include "com_driver.h"
#include "pend_enc_driver.h"
#include "FIR_filter.h"
#include "filters_coeffs.h"
#include "IIR_filter.h"
#include "LIP_tasks_common.h"
#include "LP_filter.h"

/* Note: define only one COM_SEND_* */ 
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* If defined communication task will send components of LQR controller control signal. 
Send: 
    ctrl error cart position, ctrl error pendulum angle, 0,  
    ctrl error cart speed, ctrl error pendulum speed, 0 */
// #define COM_SEND_CTRL_DEBUG

/* If defained communication task will send default info. 
Send: pend angle, pend speed, 0,
      cart position, cart speed, cart position setpoint,
      output voltage, tick time */
#define COM_SEND_DEFAULT

/* If defined communication task will send angle setpoint for upc. */
// #define COM_SEND_UPC

/* If defined communication task will send angle setpoint for dpc. */
// #define COM_SEND_DPC

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Used inside limit switch ISR */
#define READ_ZERO_POSITION_REACHED HAL_GPIO_ReadPin( limitSW_left_GPIO_Port, limitSW_left_Pin )
#define READ_MAX_POSITION_REACHED HAL_GPIO_ReadPin( limitSW_right_GPIO_Port, limitSW_right_Pin )

/* Sampling period in ms for controllers and util tasks. */
#define dt                  10
/* multiply by dt_inv instead of dividing by dt. */
#define dt_inv              100.0f
/* Sampling period in ms for watchdog task. */
#define dt_watchdog         25
/* Sampling period in ms for console task. */
#define dt_console          50
/* Sampling period in ms for communication task. */
#define dt_com              10
// #define dt_com              10 // used for tests
/* Sampling period in ms for worker task. */
#define dt_cartworker       50
/* Sampling period in ms for swingup task. 
Don't change this value or swingup routing will not work properly. 
Swingup output voltage lookup table was calculated with 10ms sampling period. */
#define dt_swingup          10

/* Priority for watchdog task. */
#define PRIORITY_WATCHDOG   4 
/* Priority for util task - has to be the same as for controler task. */
#define PRIORITY_UTIL       3 
/* Priority for any controller task. */
#define PRIORITY_CTRL       3 
/* Priority for console task. */
#define PRIORITY_CONSOLE    2 
/* Priority for communication task. */
#define PRIORITY_COM        1 
/* Priority for cartworker task. */
#define PRIORITY_CARTWORKER 1 
/* Priority for test task. */
#define PRIORITY_TEST 2 

/* For freertos config. */
#define RTOS_USE_PREEMPTION     1
#define RTOS_USE_TIME_SLICING   0

void main_LIP_init(void);
void main_LIP_run(void);

/* Defined in main_LIP.c */
void LIP_create_Tasks(void);

/* Defined in cli_commands.c */
void vRegisterCLICommands(void);

#endif /* LIP_MAIN */
