/*
 * This file provides function prototypes and #defines related to LIP app tasks.
 * Also provides prototype of LIP_create_Tasks fucntion
 */
#ifndef LIP_TASKS_COMMON
#define LIP_TASKS_COMMON

#include "main_LIP.h"

// Enum which lists all implemented LIP app states
#ifndef LIP_APP_STATES_ENUM
#define LIP_APP_STATES_ENUM
enum lip_app_states {
        /* 
         * Uninitialized state: app state before call to "zero" cli command.
         * This state indicates that current cart position reading is not
         * correct, pendulum cart has to be moved to zero position.
         * Available commands: zero, <enter_key>
         */
        UNINITIALIZED,
        /* 
         * Default state: All sesnor readings should be correct in this app
         * state. Down position controller can be started.
         * Available commands: zero, home, dpc, sp, <enter_key> 
         */
        DEFAULT,
        /* 
         * DPC state: Down Position Controller turned on.
         * Available commands: home, dpc, spcli, sppot, sp, swingup, <enter_key> 
         */
        DPC,
        /* 
         * UPC state: Up Position Controller turned on.
         * Available commands: home, upc, spcli, sppot, sp, swingdown, 
         * <enter_key>
         */
        UPC,
        /* 
         * SWINGUP state
         */
        SWINGUP,
        /* 
         * TEST state
         */
        TEST
};
#endif /* LIP_APP_STATES_ENUM */

/* Enum for cart position zones */
#ifndef CART_POSITION_ZONE_FLAGS
#define CART_POSITION_ZONE_FLAGS
enum cart_position_zones { FREEZING_ZONE_R, OK_ZONE, FREEZING_ZONE_L };
#endif /* CART_POSITION_ZONE_FLAGS */

/* 
 * Task notification values for worker task
 */
#define GO_RIGHT 0x01 /* Move cart to the right */
#define GO_LEFT  0x02 /* Move cart to the left */
#define SP_HOME  0x04 /* Change controler cart setpoint to home position */

/* 
 * Task notification values for test task
 */
#define TEST_1 0x01 /* Move cart to the right */
#define TEST_2 0x02 /* Move cart to the left */
#define TEST_3 0x04 /* Change controler cart setpoint to home position */

/*
 * FreeRTOS tasks
 */
void watchdog_task(void *pvParameters);
#define WATCHDOG_STACK_DEPTH 2000

void console_task(void *pvParameters);
#define MAX_INPUT_LENGTH  50
#define MAX_OUTPUT_LENGTH 500
#define CONSOLE_STACKDEPTH 6000

void util_task(void *pvParameters);
#define UTIL_STACK_DEPTH 1000

void com_task(void *pvParameters);
#define COM_STACK_DEPTH 500

void raw_com_task(void *pvParameters);
#define RAWCOM_STACK_DEPTH 500

void cart_worker_task(void *pvParameters);
#define CARTWORKER_STACK_DEPTH 500

void ctrl_3_FSF_downpos_task(void *pvParameters);
#define CTRL_3_FSF_DOWNPOS_STACK_DEPTH 500

void ctrl_5_FSF_uppos_task(void *pvParameters);
#define CTRL_5_FSF_UPPOS_STACK_DEPTH 500

void swingup_task(void *pvParameters);
#define SWINGUP_STACK_DEPTH 1000

void swingdown_task(void *pvParameters);
#define SWINGDOWN_STACK_DEPTH 1000

void bounceoff_task(void *pvParameters);
#define BOUNCEOFF_STACK_DEPTH 1000

void test_task(void *pvParameters);
#define TEST_STACK_DEPTH 500

void LIP_create_Tasks(void);

#endif /* LIP_TASKS_COMMON */
