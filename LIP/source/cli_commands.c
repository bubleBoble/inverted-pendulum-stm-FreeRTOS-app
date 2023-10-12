#include "main_LIP.h"
#define runtimeStats_active 0
/* ========================================================================
 * CLI commands prototypes
 * ========================================================================
*/

// Some tasks are taken from demo: https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_IO/Demo_Applications/LPCXpresso_LPC1769/NXP_LPC1769_Demo_Description.html
static portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// static portBASE_TYPE prvRunTimeStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvThreeParameterEchoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
static portBASE_TYPE prvMultiParameterEchoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// static portBASE_TYPE prvCreateTaskCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// static portBASE_TYPE prvDeleteTaskCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
// The task that is created by the create-task command
// static void prvCreatedTask( void *pvParameters );
// static xTaskHandle xCreatedTaskHandle = NULL;

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

// static const CLI_Command_Definition_t prvRunTimeStatsCommandDefinition =
// {
// 	( const int8_t * const ) "run-time-stats", /* The command string to type. */
// 	( const int8_t * const ) "run-time-stats:\r\n Displays a table showing how much processing time each FreeRTOS task has used\r\n\r\n",
// 	prvRunTimeStatsCommand,
// 	0
// };

static const CLI_Command_Definition_t prvThreeParameterEchoCommandDefinition =
{
	( const int8_t * const ) "echo-3-parameters",
	( const int8_t * const ) "echo-3-parameters <param1> <param2> <param3>:\r\n Expects three parameters, echos each in turn\r\n\r\n",
	prvThreeParameterEchoCommand,
	3
};

static const CLI_Command_Definition_t prvMultiParameterEchoCommandDefinition =
{
	( const int8_t * const ) "echo-parameters",
	( const int8_t * const ) "echo-parameters <...>:\r\n Take variable number of parameters, echos each in turn\r\n\r\n",
	prvMultiParameterEchoCommand,
	-1
};

// static const CLI_Command_Definition_t prvCreateTaskCommandDefinition =
// {
// 	( const int8_t * const ) "create-task",
// 	( const int8_t * const ) "create-task <param>:\r\n Creates a new task that periodically writes the <param> to the CLI output\r\n\r\n",
// 	prvCreateTaskCommand,
// 	1
// };

// static const CLI_Command_Definition_t prvDeleteTaskCommandDefinition =
// {
// 	( const int8_t * const ) "delete-task",
// 	( const int8_t * const ) "delete-task:\r\n Deletes the task created by the create-task command\r\n\r\n",
// 	prvDeleteTaskCommand,
// 	0
// };

/* ========================================================================
 * CLI commands registrations
 * ========================================================================
*/
void vRegisterCLICommands()
{
    FreeRTOS_CLIRegisterCommand( &prvTaskStatsCommandDefinition );
    // FreeRTOS_CLIRegisterCommand( &prvRunTimeStatsCommandDefinition );
    FreeRTOS_CLIRegisterCommand( &prvThreeParameterEchoCommandDefinition );
	FreeRTOS_CLIRegisterCommand( &prvMultiParameterEchoCommandDefinition );
	// FreeRTOS_CLIRegisterCommand( &prvCreateTaskCommandDefinition );
	// FreeRTOS_CLIRegisterCommand( &prvDeleteTaskCommandDefinition );
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

// static portBASE_TYPE prvRunTimeStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
// {
//     const int8_t * const pcStatsTableHeader = ( int8_t * ) "Task            Abs Time      % Time\r\n****************************************\r\n";

// 	( void ) pcCommandString;
// 	( void ) xWriteBufferLen;
// 	configASSERT( pcWriteBuffer );

// 	strcpy( ( char * ) pcWriteBuffer, ( char * ) pcStatsTableHeader );
// 	vTaskGetRunTimeStats( pcWriteBuffer + strlen( ( char * ) pcStatsTableHeader ) );

// 	return pdFALSE;
// }

static portBASE_TYPE prvThreeParameterEchoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    int8_t *pcParameterString;
    portBASE_TYPE xParameterStringLength, xReturn;
    static portBASE_TYPE xParameterNumber = 0;

	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( xParameterNumber == 0 )
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf( ( char * ) pcWriteBuffer, "The three parameters were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		xParameterNumber = 1L;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										xParameterNumber,		/* Return the next parameter. */
										&xParameterStringLength	/* Store the parameter string length. */
									);

		/* Sanity check something was returned. */
		configASSERT( pcParameterString );

		/* Return the parameter string. */
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		sprintf( ( char * ) pcWriteBuffer, "%ld: ", xParameterNumber );
		strncat( ( char * ) pcWriteBuffer, ( const char * ) pcParameterString, xParameterStringLength );
		// strncat( ( char * ) pcWriteBuffer, "\r\n", strlen( "\r\n" ) );
		strncat( ( char * ) pcWriteBuffer, "\r\n", 2 );

		/* If this is the last of the three parameters then there are no more
		strings to return after this one. */
		if( xParameterNumber == 3L )
		{
			/* If this is the last of the three parameters then there are no more
			strings to return after this one. */
			xReturn = pdFALSE;
			xParameterNumber = 0L;
		}
		else
		{
			/* There are more parameters to return after this one. */
			xReturn = pdTRUE;
			xParameterNumber++;
		}
	}

	return xReturn;
}

static portBASE_TYPE prvMultiParameterEchoCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    int8_t *pcParameterString;
    portBASE_TYPE xParameterStringLength, xReturn;
    static portBASE_TYPE xParameterNumber = 0;

	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( xParameterNumber == 0 )
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf( ( char * ) pcWriteBuffer, "The parameters were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		xParameterNumber = 1L;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter
									(
										pcCommandString,		/* The command string itself. */
										xParameterNumber,		/* Return the next parameter. */
										&xParameterStringLength	/* Store the parameter string length. */
									);

		if( pcParameterString != NULL )
		{
			/* Return the parameter string. */
			memset( pcWriteBuffer, 0x00, xWriteBufferLen );
			sprintf( ( char * ) pcWriteBuffer, "%ld: ", xParameterNumber );
			strncat( ( char * ) pcWriteBuffer, ( const char * ) pcParameterString, xParameterStringLength );
			// strncat( ( char * ) pcWriteBuffer, "\r\n", strlen( "\r\n" ) );
			strncat( ( char * ) pcWriteBuffer, "\r\n", 2 );

			/* There might be more parameters to return after this one. */
			xReturn = pdTRUE;
			xParameterNumber++;
		}
		else
		{
			/* No more parameters were found.  Make sure the write buffer does
			not contain a valid string. */
			pcWriteBuffer[ 0 ] = 0x00;

			/* No more data to return. */
			xReturn = pdFALSE;

			/* Start over the next time this command is executed. */
			xParameterNumber = 0;
		}
	}

	return xReturn;
}


        // static portBASE_TYPE prvCreateTaskCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
        // {
        //     int8_t *pcParameterString;
        //     portBASE_TYPE xParameterStringLength;
        //     static const int8_t *pcSuccessMessage = ( int8_t * ) "Task created\r\n";
        //     static const int8_t *pcFailureMessage = ( int8_t * ) "Task not created\r\n";
        //     static const int8_t *pcTaskAlreadyCreatedMessage = ( int8_t * ) "The task has already been created. Execute the delete-task command first.\r\n";
        //     int32_t lParameterValue;

        // 	( void ) xWriteBufferLen;
        // 	configASSERT( pcWriteBuffer );

        // 	/* Obtain the parameter string. */
        // 	pcParameterString = ( int8_t * ) FreeRTOS_CLIGetParameter
        // 								(
        // 									pcCommandString,		/* The command string itself. */
        // 									1,						/* Return the first parameter. */
        // 									&xParameterStringLength	/* Store the parameter string length. */
        // 								);

        // 	/* Turn the parameter into a number. */
        // 	lParameterValue = ( int32_t ) atol( ( const char * ) pcParameterString );

        // 	/* Attempt to create the task. */
        // 	if( xCreatedTaskHandle != NULL )
        // 	{
        // 		strcpy( ( char * ) pcWriteBuffer, ( const char * ) pcTaskAlreadyCreatedMessage );
        // 	}
        // 	else
        // 	{
        // 		if( xTaskCreate( prvCreatedTask, ( const char * ) "Created", configMINIMAL_STACK_SIZE,  ( void * ) lParameterValue, tskIDLE_PRIORITY, &xCreatedTaskHandle ) == pdPASS )
        // 		{
        // 			strcpy( ( char * ) pcWriteBuffer, ( const char * ) pcSuccessMessage );
        // 		}
        // 		else
        // 		{
        // 			strcpy( ( char * ) pcWriteBuffer, ( const char * ) pcFailureMessage );
        // 		}
        // 	}

        // 	/* There is no more data to return after this single string, so return
        // 	pdFALSE. */
        // 	return pdFALSE;
        // }

        // static portBASE_TYPE prvDeleteTaskCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
        // {
        //     static const int8_t *pcSuccessMessage = ( int8_t * ) "Task deleted\r\n";
        //     static const int8_t *pcFailureMessage = ( int8_t * ) "The task was not running.  Execute the create-task command first.\r\n";

        // 	( void ) pcCommandString;
        // 	( void ) xWriteBufferLen;
        // 	configASSERT( pcWriteBuffer );

        // 	/* See if the task is running. */
        // 	if( xCreatedTaskHandle != NULL )
        // 	{
        // 		vTaskDelete( xCreatedTaskHandle );
        // 		xCreatedTaskHandle = NULL;
        // 		strcpy( ( char * ) pcWriteBuffer, ( const char * ) pcSuccessMessage );
        // 	}
        // 	else
        // 	{
        // 		strcpy( ( char * ) pcWriteBuffer, ( const char * ) pcFailureMessage );
        // 	}

        // 	/* There is no more data to return after this single string, so return
        // 	pdFALSE. */
        // 	return pdFALSE;
        // }

        // // Console mutex related
        // extern SemaphoreHandle_t xConsoleWriteMutex;
        // extern TickType_t time_at_which_consoleMutex_was_taken;

        // static void prvCreatedTask( void *pvParameters )
        // {
        //     int32_t lParameterValue;
        //     static uint8_t pucLocalBuffer[ 60 ];
        //     void vOutputString( const uint8_t * const pucMessage );

        // 	/* Cast the parameter to an appropriate type. */
        // 	lParameterValue = ( int32_t ) pvParameters;

        // 	memset( ( void * ) pucLocalBuffer, 0x00, sizeof( pucLocalBuffer ) );
        // 	sprintf( ( char * ) pucLocalBuffer, "Created task running.  Received parameter %ld\r\n\r\n", ( long ) lParameterValue );
            
        //     // mutex for console write
        //     xSemaphoreTake( xConsoleWriteMutex, 1 );
        //     time_at_which_consoleMutex_was_taken = xTaskGetTickCount();

        //     printf( "%s", pucLocalBuffer );
        //     fflush(stdout);

        //     // mutex for console write end
        //     xSemaphoreGive( xConsoleWriteMutex );
        //     // if ( xTaskGetTickCount() != time_at_which_consoleMutex_was_taken ) { taskYIELD(); }

        // 	for( ;; )
        // 	{
        // 		vTaskDelay( portMAX_DELAY );
        // 	}
        // }


