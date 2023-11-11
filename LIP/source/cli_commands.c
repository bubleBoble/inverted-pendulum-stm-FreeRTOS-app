/* demo: https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_IO/Demo_Applications/LPCXpresso_LPC1769/NXP_LPC1769_Demo_Description.html */
#include "main_LIP.h"

extern TaskHandle_t comTaskHandle;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CLI commands prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
/* This command shows task statistics - doesn't work XD, 
command : task-stats */
static portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to turn communication on or off, 
command : ENTER_KEY */
static portBASE_TYPE prvComOnOffCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to zero the cart position, performed by watchdog task
command : zero */
static portBASE_TYPE prvZeroCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to move cart to default position - center of the track, no controller is used, 
command : home */
static portBASE_TYPE prvHomeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to turn on or off down position controller,
command : dpc */
static portBASE_TYPE prvDpcCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to clear console screen, at least for putty,
command : clc */
static portBASE_TYPE prvClcCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

// '\033[3J'

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CLI commands definition structures & registration
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
/* command : task-stats */
static const CLI_Command_Definition_t commands_list[] = 
{
    {
        .pcCommand                      = ( const int8_t * const ) "task-stats",
        .pcHelpString                   = ( const int8_t * const ) "task-stats  :    Displays a table showing the state of each FreeRTOS task\r\n                 (B)blocked, (R)Ready, (D)Deleted, (S)Suspended or block w/o timeout\r\n",
        .pxCommandInterpreter           = prvTaskStatsCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "",
        .pcHelpString                   = ( const int8_t * const ) "<enter_key> :    Start / stop data streaming\r\n",
        .pxCommandInterpreter           = prvComOnOffCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "zero",
        .pcHelpString                   = ( const int8_t * const ) "zero        :    Go to zero cart positon, no controller used\r\n",
        .pxCommandInterpreter           = prvZeroCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "home",
        .pcHelpString                   = ( const int8_t * const ) "home        :    Go to home cart positon - center of the track, no controller used\r\n",
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
        .pcHelpString                   = ( const int8_t * const ) "clc         :    Clears console screen\r\n",    
        .pxCommandInterpreter           = prvClcCommand,
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
 * CLI commands definitions
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
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

static portBASE_TYPE prvZeroCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
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

static portBASE_TYPE prvHomeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
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