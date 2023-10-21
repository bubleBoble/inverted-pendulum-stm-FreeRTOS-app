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

// turn on ramp input task 
#define TEST_RAMP_INPUT 0

void main_LIP_init(void);
void main_LIP_run(void);

// in main_LIP.c
void LIPcreateTasks(void);

// in cli_commands.c
void vRegisterCLICommands(void);

#endif /* LIP_MAIN */
