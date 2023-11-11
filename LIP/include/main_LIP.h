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
#include "SP_filter.h"

/* Sampling period and freeRTOS tasks priorities definitions. */ 
#define dt                  10      // Sampling period in ms for controllers and util tasks,
#define dt_inv              100.0f  // multiply by dt_inv instead of dividing by dt.
#define dt_watchdog         10      // Sampling period in ms for watchdog task.
#define dt_console          50      // Sampling period in ms for console task.
#define dt_com              50      // Sampling period in ms for communication task.

#define PRIORITY_WATCHDOG   4       // Priority for watchdog task.
#define PRIORITY_UTIL       3       // Priority for util task - has to be the same as for controler task.
#define PRIORITY_CTRL       3       // Priority for any controller task.
#define PRIORITY_CONSOLE    2       // Priority for console task. 
#define PRIORITY_COM        1       // Priority for communication task.

/* For freertos config.
If time slicing was used, task "watchdog", "util" and "controler" could take 
a little bit more time to execute in their 10ms time period becouse of 
more frequent context switching - leaving no time for "com" and "console" tasks. 
In setting with preemtion and no time slicing, context switching should happen
only when one task with the same priority finished its work - less frequent.
ANYWAY, BOTH WAYS WORK AS FOR NOW SO I'LL LEAVE TIME SLICING. LOL */
#define RTOS_USE_PREEMPTION     1
#define RTOS_USE_TIME_SLICING   0

void main_LIP_init(void);
void main_LIP_run(void);

/* Defined in main_LIP.c */
void LIPcreateTasks(void);

/* Defined in cli_commands.c */
void vRegisterCLICommands(void);

#endif /* LIP_MAIN */
