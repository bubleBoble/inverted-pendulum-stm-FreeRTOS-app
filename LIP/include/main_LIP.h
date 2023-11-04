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

/* Sampling period in milli seconds and its inverse used for calculations.
Constant dt is used by all controllers tasks and comunication task. */
#define dt      10
#define dt_inv  100.0f

void main_LIP_init(void);
void main_LIP_run(void);

/* Defined in main_LIP.c */
void LIPcreateTasks(void);

/* Defined in cli_commands.c */
void vRegisterCLICommands(void);

#endif /* LIP_MAIN */
