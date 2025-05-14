/*
 * main_LIP.h
 */
#ifndef LIP_MAIN
#define LIP_MAIN

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

/*
 * Note: define only one COM_SEND_* form below
 */

/* errors for state variables */
// #define COM_SEND_CTRL_DEBUG

/* state variables, setpoint, output voltage and tick */
#define COM_SEND_DEFAULT

/* angle setpoint for upc */
// #define COM_SEND_UPC

/* angle setpoint for dpc */
// #define COM_SEND_DPC
 
/*
 * Used inside limit switch ISR
 */
#define READ_ZERO_POSITION_REACHED \
        HAL_GPIO_ReadPin(limitSW_left_GPIO_Port, limitSW_left_Pin)
#define READ_MAX_POSITION_REACHED \
        HAL_GPIO_ReadPin(limitSW_right_GPIO_Port, limitSW_right_Pin)

/*
 * Sampling periods in mili seconds (ms)
 */
#define dt            10     /* for controllers and util tasks */
#define dt_inv        100.0f /* inverse of 10 ms delay */
#define dt_watchdog   25
#define dt_console    50
#define dt_com        10
#define dt_cartworker 50
#define dt_swingup    10 /* don't change this value */

/*
 * Task priorities
 */
#define PRIORITY_WATCHDOG 4
#define PRIORITY_UTIL 3
#define PRIORITY_CTRL 3
#define PRIORITY_CONSOLE 2
#define PRIORITY_COM 1
#define PRIORITY_CARTWORKER 1
#define PRIORITY_TEST 2

/*
 * For freertos config
 */
#define RTOS_USE_PREEMPTION   1
#define RTOS_USE_TIME_SLICING 0

void main_LIP_init(void);
void main_LIP_run(void);

void LIP_create_Tasks(void);

/*
 * Defined in cli_commands.c
 */
void register_CLI_commands(void);

#endif /* LIP_MAIN */
