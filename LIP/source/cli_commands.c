#include "main_LIP.h"

/* ========================================================================
 * CLI commands prototypes
 * ========================================================================
*/
static portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvCmdOk( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* ========================================================================
 * CLI commands definition structures
 * ========================================================================
*/
static const CLI_Command_Definition_t prvTaskStatsCommandDefinition =
{
    ( const int8_t * const ) "task-stats",
    ( const int8_t * const ) "task-stats:\r\n Displays a table showing the state of each FreeRTOS task\r\n\r\n",
    prvTaskStatsCommand,
    0
};

static const CLI_Command_Definition_t prvCmdOkDef = {
    ( const int8_t * const ) "ok",
    ( const int8_t * const ) "ok:\r\n Shows an OK message\r\n", 
    prvCmdOk,
    0
};

/* ========================================================================
 * CLI commands registrations
 * ========================================================================
*/
void vRegisterCLICommands()
{
    FreeRTOS_CLIRegisterCommand( &prvTaskStatsCommandDefinition );
    FreeRTOS_CLIRegisterCommand( &prvCmdOkDef );
}

/* ========================================================================
 * CLI commands definitions
 * ========================================================================
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

    /* There is no more data to return after this single string, so return pdFALSE. */
    return pdFALSE;
}

static portBASE_TYPE prvCmdOk( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    const char *const okMessage = "everything OK! \r\n";

    memset( pcWriteBuffer, 0x00, xWriteBufferLen );
    strcpy( (char *) pcWriteBuffer, (char *) okMessage );
    
    return pdFALSE;
}


