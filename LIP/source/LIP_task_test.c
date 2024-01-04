#include "LIP_tasks_common.h"
#include "limits.h"

void testProcedure1( void );

// extern uint32_t reset_test;
extern enum cart_position_zones cart_current_zone;
extern float *cart_position_setpoint_cm;
extern float cart_position_setpoint_cm_cli;
extern enum lip_app_states app_current_state; 
extern TaskHandle_t ctrl_downposition_taskHandle;
extern float cart_position_setpoint_cm_cli_raw;
extern uint32_t swingup_task_resumed;
extern uint32_t reset_lookup_index;
extern TaskHandle_t swingup_task_handle;
extern uint32_t reset_swingdown;
extern TaskHandle_t swingdown_task_handle;

void test_task( void *pvParameters )
{
    /* Holds value retrieved from task notification. */
    uint32_t notifValueReceived;

    for( ;; )
    {   
        /* Wait for notification at index 0. */
        xTaskNotifyWaitIndexed( 0,                     /* Notification index */
                                0x00,                  /* Bits to clear on entry (before save). */ 
                                ULONG_MAX,             /* Bits to clear on exit (after save). */
                                &notifValueReceived,   /* Value of received notification. Can be set to NULL if not used. */
                                portMAX_DELAY );       /* Block time (while waiting for notification). */
        if( notifValueReceived == TEST_1 )
        {
            /* Test procedure nr. 1 selected */
            testProcedure1();
        }

    }
}

void testProcedure1( void )
{
    /* If this function is run, it means that the app is in TEST state. */
    
    /* For RTOS vTaskDelayUntil(). */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    char msg[128];

    // /* [ 01 ] dpc on */
    // if( cart_current_zone != FREEZING_ZONE_L || cart_current_zone != FREEZING_ZONE_R )
    // {
    //     /* Ensure that setpoint for cart postion from cli is the same as the setpoint used by controller tasks.
    //     If it's not true, this means that the user changed source of cart pos. setpoint for controllers. 
    //     ALL CONTROLLER TASKS SHOULD BE TURNING ON WITH CART POS. SETPOINT SOURCE SET TO CLI, OTHERWISE DON'T TURN ON CONTROLLER. */
    //     if( cart_position_setpoint_cm == &cart_position_setpoint_cm_cli )
    //     {
    //         /* Turn on down position controller, "dcp on" / "dpc 1" are both valid commands. */
    //         vTaskResume( ctrl_downposition_taskHandle );
            
    //         /* Change app state to "down position controller" state. 
    //         This will ensure that some cli commands can't be called. */
    //         // app_current_state = DPC;
    //     }
    //     else
    //     {
    //         /* Prompt the user to change setpoint source to cli with "spcli" command. */
    //         strcpy( msg, "\r\nERROR: SET CART POSITION SETPOINT SOURCE TO CLI WITH COMMAND: spcli\r\n" );
    //         com_send( msg, strlen(msg) );
    //     }
    // }
    // else
    // {
    //     strcpy( msg, "\r\nERROR: CAN'T TURN ON DPC, CART TOO CLOSE TO TRACK LIMITS\r\n" );
    //     com_send( msg, strlen(msg) );
    // }
    // vTaskDelayUntil( &xLastWakeTime, 2000 );

    // /* [ 02 ] home */
    // cart_position_setpoint_cm_cli_raw = 20.0f;

    // /* [ 03 ] wait 2 sec. */
    // vTaskDelayUntil( &xLastWakeTime, 2000 );

    // /* [ 04 ] sp 10 */
    // cart_position_setpoint_cm_cli_raw = 10.0f;

    // /* [ 05 ] wait 2 sec. */
    // vTaskDelayUntil( &xLastWakeTime, 2000 );

    // /* [ 06 ] sp 30 */
    // cart_position_setpoint_cm_cli_raw = 30.0f;

    // /* [ 07 ] wait 2 sec. */
    // vTaskDelayUntil( &xLastWakeTime, 2000 );

    // /* [ 08 ] sp 10 */
    // cart_position_setpoint_cm_cli_raw = 10.0f;

    // /* [ 09 ] wait 2 sec. */
    // vTaskDelayUntil( &xLastWakeTime, 2000 );

    // /* [ 10 ] sp 30 */
    // cart_position_setpoint_cm_cli_raw = 30.0f;
    
    // /* [ 11 ] wait 2 sec. */
    // vTaskDelayUntil( &xLastWakeTime, 2000 );

    // /* [ 12 ] sp 10 */
    // cart_position_setpoint_cm_cli_raw = 10.0f;

    // /* [ 13 ] wait 2 sec. */
    // vTaskDelayUntil( &xLastWakeTime, 2000 );

    // /* [ 14 ] sp 30 */
    // cart_position_setpoint_cm_cli_raw = 30.0f;

    // /* [ 15 ] wait 2 sec. */
    // vTaskDelayUntil( &xLastWakeTime, 2000 );

    // /* [ 16 ] home */
    // cart_position_setpoint_cm_cli_raw = 20.0f;

    // /* [ 17 ] wait 2 sec. */
    // vTaskDelayUntil( &xLastWakeTime, 2000 );

    // /* [ 18 ] dpc off */
    // vTaskSuspend( ctrl_downposition_taskHandle );
    // // app_current_state = DEFAULT;
    // dcm_set_output_volatage( 0.0f );

    /* [ 19 ] swingup */
    /* App is in DEFAULT STATE and cart position is at position 20cm pm 1cm.
    Swingup can be started. */
    
    /* Reset lookup_index in swingup task for loop. */
    reset_lookup_index = 1;

    /* Global flag to indicate that swingup is running and to tell watchdog task to
    start keeping track of pendulum angle to switch between swingup and up position controller. */
    swingup_task_resumed = 1;
    
    /* Change app state to swingup. */
    app_current_state = SWINGUP;
    
    /* Resume swingup task. */
    vTaskResume( swingup_task_handle );
    
    /* [ 20 ] wait 6 sec. */
    vTaskDelayUntil( &xLastWakeTime, 6000 );

    /* [ 21 ] home */
    cart_position_setpoint_cm_cli_raw = 20.0f;

    /* [ 22 ] wait 2 sec. */
    vTaskDelayUntil( &xLastWakeTime, 3000 );

    /* [ 23 ] sp 10 */
    cart_position_setpoint_cm_cli_raw = 10.0f;

    /* [ 24 ] wait 2 sec. */
    vTaskDelayUntil( &xLastWakeTime, 3000 );

    /* [ 25 ] sp 30 */
    cart_position_setpoint_cm_cli_raw = 30.0f;

    /* [ 26 ] wait 2 sec. */
    vTaskDelayUntil( &xLastWakeTime, 3000 );

    /* [ 27 ] sp 10 */
    cart_position_setpoint_cm_cli_raw = 10.0f;

    /* [ 28 ] wait 2 sec. */
    vTaskDelayUntil( &xLastWakeTime, 3000 );

    /* [ 29 ] sp 30 */
    cart_position_setpoint_cm_cli_raw = 30.0f;

    /* [ 30 ] wait 2 sec. */
    vTaskDelayUntil( &xLastWakeTime, 3000 );

    /* [ 31 ] sp 10 */
    cart_position_setpoint_cm_cli_raw = 10.0f;

    /* [ 32 ] wait 2 sec. */
    vTaskDelayUntil( &xLastWakeTime, 3000 );

    /* [ 32 ] sp 30 */
    cart_position_setpoint_cm_cli_raw = 30.0f;

    /* [ 33 ] wait 2 sec. */
    vTaskDelayUntil( &xLastWakeTime, 3000 );
    
    /* [ 34 ] home */
    cart_position_setpoint_cm_cli_raw = 20.0f;
    
    // /* [ 35 ] swingdown */
    // /* Reset lookup_index in swingup task for loop. */
    // reset_swingdown = 1;

    // /* Change app state to swingup. */
    // app_current_state = DPC;
    
    // /* Resume swingup task. */
    // vTaskResume( swingdown_task_handle );
}