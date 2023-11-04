/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides stuff related for LIP app tasks.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "main_LIP.h"

/* Start task. */
void startTask( void *pvParameters );
#define STARTTASK_STACKDEPTH 500

/* console task. */
void vCommandConsoleTask( void *pvParameters );
#define MAX_INPUT_LENGTH    50
#define MAX_OUTPUT_LENGTH   100
#define CONSOLE_STACKDEPTH  4000

/* State estimation task. */
void stateEstimationTask( void *pvParameters );
#define STATE_ESTIMATION_STACK_DEPTH 500

/* Controller 1 - FSF (full state feedback), task. */
void ctrlFSFTask( void *pvParameters );
#define CTRL_FSF_STACK_DEPTH 500

/* Communication task - for serialOscilloscope. */
void comTask( void *pvParameters );
#define COM_STACK_DEPTH 500

/* Communication task - for raw bytes transmission. */
void rawComTask( void *pvParameters );
#define RAWCOM_STACK_DEPTH 500

/* Function to create tasks. */
void LIPcreateTasks(void);

