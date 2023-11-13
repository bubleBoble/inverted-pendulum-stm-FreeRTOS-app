/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides cli commands definitions and functionality
 *
 * Note: commands callback functions change app state, which is indicated by
 * preprompt string in cli prompt ( (preprompt)>>> ). All logic related to
 * LIP app state changes is contained in cli commands callback functions.
 *
 * Note2: in FreeRTOSConfig.h, configTASK_NOTIFICATION_ARRAY_ENTRIES is set to 5
 * so max 5 notifications for a task.
 *
 * All variables related to LIP app state are defined in file: LIP_tasks_common.c
 *
 * FreeRTOS CLI demo:
 * https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_IO/Demo_Applications/LPCXpresso_LPC1769/NXP_LPC1769_Demo_Description.html
 */
#include "main_LIP.h"

extern enum lip_app_states app_current_state;

/* Tasks handles from LIP_tasks_common.c */
extern TaskHandle_t watchdogTaskHandle;
extern TaskHandle_t consoleTaskHandle;
extern TaskHandle_t utilTaskHandle;
extern TaskHandle_t comTaskHandle;
extern TaskHandle_t rawComTaskHandle;
extern TaskHandle_t WorkerTaskHandle;
extern TaskHandle_t ctrl_3_FSF_downpos_task_handle;

/* Global cart position variable, defined in LIP_tasks_common.c */
extern float cart_position[ 2 ];

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CLI commands prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
/* This command shows task statistics
command : task-stats */
static portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to turn communication on or off,
command : ENTER_KEY */
static portBASE_TYPE prvComOnOffCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to move cart to default position - center of the track, no controller is used,
command : home */
static portBASE_TYPE prvHomeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to turn on or off down position controller,
command : dpc */
static portBASE_TYPE prvDpcCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to clear console screen, at least for putty,
command : clc */
static portBASE_TYPE prvClcCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to change cart position setpoint to potentiometer,
command: sppot */
static portBASE_TYPE prvSPPOTCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to change cart position setpoint to cli,
command: spcli */
static portBASE_TYPE prvSPCLICommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to change current cart position setpoint,
command: sp */
static portBASE_TYPE prvSPCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to start swingup action,
command: swingup */
static portBASE_TYPE prvSWINGUPCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CLI commands definition structures & registration
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static const CLI_Command_Definition_t commands_list[] =
{
    {
        .pcCommand                      = ( const int8_t * const ) "task-stats",
        .pcHelpString                   = ( const int8_t * const ) "task-stats  :    Displays a table showing the state of each FreeRTOS task.\r\n                 (B)blocked, (R)Ready, (D)Deleted, (S)Suspended or block w/o timeout\r\n",
        .pxCommandInterpreter           = prvTaskStatsCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "",
        .pcHelpString                   = ( const int8_t * const ) "<enter_key> :    Start / stop data streaming.\r\n",
        .pxCommandInterpreter           = prvComOnOffCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "home",
        .pcHelpString                   = ( const int8_t * const ) "home        :    Go to home cart positon - center of the track, no controller used.\r\n",
        .pxCommandInterpreter           = prvHomeCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "dpc",
        .pcHelpString                   = ( const int8_t * const ) "dpc         :    Turn on/off down position controller, default cart pos. setpoint is home pos.\r\n",
        .pxCommandInterpreter           = prvDpcCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "clc",
        .pcHelpString                   = ( const int8_t * const ) "clc         :    Clears console screen.\r\n",
        .pxCommandInterpreter           = prvClcCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "sppot",
        .pcHelpString                   = ( const int8_t * const ) "sppot       :    Change cart possition setpoint source to potentiometer.\r\n",
        .pxCommandInterpreter           = prvSPPOTCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "spcli",
        .pcHelpString                   = ( const int8_t * const ) "spcli       :    Change cart possition setpoint source to CLI.\r\n",
        .pxCommandInterpreter           = prvSPCLICommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "sp",
        .pcHelpString                   = ( const int8_t * const ) "sp          :    Change cart possition setpoint source to CLI.\r\n                 When run with no arguemnt display current cart position setpoint.\r\n",
        .pxCommandInterpreter           = prvSPCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "swingup",
        .pcHelpString                   = ( const int8_t * const ) "swingup     :    Start pendulum swingup routine.\r\n",
        .pxCommandInterpreter           = prvSWINGUPCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand = NULL
    }
};

void vRegisterCLICommands(void)
{
    uint8_t command_index = 0;
    while (commands_list[command_index].pcCommand != NULL)
    {
        FreeRTOS_CLIRegisterCommand(&commands_list[command_index++]);
    }
}
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CLI commands callback functions definitions
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
/* command: task-stats */
static portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    const int8_t *const pcTaskTableHeader = ( int8_t * ) "Task          State  Priority  Stack	#\r\n******************************************\r\n";

    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Generate a table of task stats. */
    strcpy( ( char * ) pcWriteBuffer, ( const char * ) pcTaskTableHeader );
    vTaskList( (char *) pcWriteBuffer + strlen( ( const char * ) pcTaskTableHeader ) );
    // configSTATS_BUFFER_MAX_LENGTH

    /* There is no more data to return after this single string, so return pdFALSE. */
    return pdFALSE;
}

/* command: <enter_key> (data logging on/off) */
static portBASE_TYPE prvComOnOffCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    if( eTaskGetState( comTaskHandle ) == eSuspended )
    {
        vTaskResume( comTaskHandle );
    }
    else
    {
        vTaskSuspend( comTaskHandle );
    }

    /* There is no more data to return after this single string, so return pdFALSE. */
    return pdFALSE;
}

/* [ ] command: home */
static portBASE_TYPE prvHomeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* This command will send notification to worker task that will take action based on notification value/index (?) */
    char msg_buffer[70];

    if( app_current_state == UNINITIALIZED )
    {
        /* App is in UNINITIALIZED state (right after uC powerup). 
        Cart position can be arbitrary - so cart position have to be calibrated.
        First the cart goes to min left position, then moves to the track center. */
        if( READ_ZERO_POSITION_REACHED )
        {
            /* Cart is already in zero position.
            Send notification to worker_task,
            set notification bit 0 - go to the right until cart is in the middle. */
            xTaskNotifyIndexed( WorkerTaskHandle,         /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                GO_RIGHT,                 /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

            sprintf( msg_buffer, "\r\nCart already in min position, going right.\r\n" ); // 44 chars
            com_send( msg_buffer, strlen( msg_buffer ) );
        }
        else
        {
            /* CALIBRATION - Cart position is arbitrary.
            Send notification to worker_task,
            set notification bit 1 - go left until min position reached, then go right to track center. */
            xTaskNotifyIndexed( WorkerTaskHandle,         /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                GO_LEFT+GO_RIGHT,         /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

            sprintf( msg_buffer, "\r\nCart position arbitrary, going left, then to track center.\r\n" ); // 59 chars
            com_send( msg_buffer, strlen( msg_buffer ) );
        }
    }
    else if( app_current_state == DEFAULT )
    {
        /* App is not in DEFAULT state - cart position should be calibrated. */
        if( cart_position[ 0 ] < TRACK_LEN_MAX_CM/2 )
        {
            /* Cart is to the left of track center. */
            xTaskNotifyIndexed( WorkerTaskHandle,         /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                GO_RIGHT,                 /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

            sprintf( msg_buffer, "\r\nGoing right\r\n" );
            com_send( msg_buffer, strlen( msg_buffer ) );
        }
        else
        {
            /* Cart is to the right of track center. */
            xTaskNotifyIndexed( WorkerTaskHandle,         /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                GO_LEFT,                  /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

            sprintf( msg_buffer, "\r\nGoing left\r\n" );
            com_send( msg_buffer, strlen( msg_buffer ) );
        }
    }
    else if( app_current_state == DPC || app_current_state == UPC )
    {
        /* App is not in DEFAULT or UNINITIALIZED state, 
        its either in DOWN_POS_CONTROL(DPC) or UP_POSITION_CONTROL(UPC) state. While in either one of these two control states,
        calling "home" command should change cart position setpoint to home position (center of the track). */
        xTaskNotifyIndexed( WorkerTaskHandle,         /* Task to notify. */
                            0,                        /* Notification index is 0. */
                            SP_HOME,                  /* Used to update notification value. */
                            eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

        sprintf( msg_buffer, "\r\nController cart setpoint changed to home position.\r\n" );
        com_send( msg_buffer, strlen( msg_buffer ) );
    }
    else
    {
        /* App is in swingup state. */
        sprintf( msg_buffer, "\r\nERROR: COMMAND NOT AVALIABLE IN SWINGUP STATE.\r\n" );
        com_send( msg_buffer, strlen( msg_buffer ) );
    }

    /* There is no more data to return after this single string, so return pdFALSE. */
    return pdFALSE;
}

/* [ ] command: dpc */
static portBASE_TYPE prvDpcCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* There is no more data to return after this single string, so return pdFALSE. */
    return pdFALSE;
}

/* [ ] command: clc */
static portBASE_TYPE prvClcCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Send clear screen char sequence. */
    com_send("\e[1;1H\e[2J", 10);
    // printf("\e[1;1H\e[2J");

    /* There is no more data to return after this single string, so return pdFALSE. */
    return pdFALSE;
}

/* [ ] command: sppot */
static portBASE_TYPE prvSPPOTCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* There is no more data to return after this single string, so return pdFALSE. */
    return pdFALSE;
}

/* [ ] command: spcli */
static portBASE_TYPE prvSPCLICommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* There is no more data to return after this single string, so return pdFALSE. */
    return pdFALSE;
}

/* [ ] command: sp */
static portBASE_TYPE prvSPCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* There is no more data to return after this single string, so return pdFALSE. */
    return pdFALSE;
}

/* [ ] command: swingup */
static portBASE_TYPE prvSWINGUPCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    /* Remove compile time warnings about unused parameters, and check the
    write buffer is not NULL.  NOTE - for simplicity, this example assumes the
    write buffer length is adequate, so does not check for buffer overflows. */
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* There is no more data to return after this single string, so return pdFALSE. */
    return pdFALSE;
}



