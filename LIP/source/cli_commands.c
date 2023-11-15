/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This file provides cli commands functionality
 *
 * All app states: 
 *     UNINTIALIZED
 *     DEFAULT
 *     DPC
 *     UPC
 *     SWINGUP
 * 
 * Commands: 
 *     task-stats       -    Displays a table showing the state of each FreeRTOS task
 *     <enter-key>      -    Start/stop data streaming
 *     home             -    Go to home cart position - center of the track, no controller used
 *     reset/rr         -    Reset uC
 *     vol              -    Manually set DC motor voltage
 *     br               -    Brake, sets output voltage to zero, suspend any active control task
 *     sp               -    Change cart possition setpoint source to CLI
 *     sppot            -    Change cart possition setpoint source to potentiometer
 *     spcli            -    Change cart possition setpoint source to CLI
 *     dpc              -    Turn on/off down position controller, default cart pos. setpoint is current cart position
 *     upc              -    Turn on/off up position controller ??????????????????????????????????????????????????????
 *     swingup          -    Turn on pendulum swingup procedure
 * 
 * Note: commands callback functions change app state, which is indicated by
 * preprompt string in cli prompt ( (preprompt)>>> ). All logic related to
 * LIP app state changes is contained in cli commands callback functions.
 *
 * Note2: in FreeRTOSConfig.h, configTASK_NOTIFICATION_ARRAY_ENTRIES is set to 5
 * so max 5 notifications for a task.
 *
 * Note3: 
 *     Commands for UNINITIALIZED state:
 *         task-stats, <enter_key>, home, reset/rr, vol, br
 *         Only "home" command will change state of the app from UNINITIALIZED TO DEFAULT
 *     Commands for DEFAULT state:
 *         task-stats, <enter_key>, home, reset/rr, vol, br
 *         sp, sppot, spcli
 *         dpc, upc, swingup
 *         In this state, only commands "dpc", "upc" and "swingup" will change app state to their respective states
 * 
 * All variables related to LIP app state are defined in file: LIP_tasks_common.c
 *
 * FreeRTOS CLI demo:
 * https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_IO/Demo_Applications/LPCXpresso_LPC1769/NXP_LPC1769_Demo_Description.html
 */
#include "main_LIP.h"
#include "stdlib.h"     /* For strtox functions (string to float). */
#include <errno.h>      /* error numbers */

/* Keeps track of current app state, defined in LIP_tasks_common.c */
extern enum lip_app_states app_current_state;

/* Tasks handles, defined in LIP_tasks_common.c */
extern TaskHandle_t watchdogTaskHandle;
extern TaskHandle_t consoleTaskHandle;
extern TaskHandle_t utilTaskHandle;
extern TaskHandle_t comTaskHandle;
extern TaskHandle_t rawComTaskHandle;
extern TaskHandle_t cartWorkerTaskHandle;
extern TaskHandle_t ctrl_3_FSF_downpos_task_handle;

/* Global cart position and pendulum angle, defined in LIP_tasks_common.c */
extern float cart_position[ 2 ];
extern float pend_angle[ 2 ];

/* Defined in LIP_tasks_common.c. This variable hold setpoint for cart position set by cli command "spcli".
It is the default setpoint source used by all controllers tasks. When turning on any controller (dpc or upc),
for safety reasons, value under this variable should be set to current cart position. */
extern float cart_position_setpoint_cm_cli; 

/* Defined in LIP_tasks_common.c. Value under this variable is used as cart postion setpoint by all controller tasks. */
extern float *cart_position_setpoint_cm;

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
/* Command to reset uC,
command: reset */
static portBASE_TYPE prvRstUCCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to manually set dc motor output voltage (via setpoint pointer so it is filtered / smooth),
command: vol <voltage_setting> */
static portBASE_TYPE prvVolCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* Command to brake the cart, sets dc motor output voltage instantly to zero,
command: br <voltage_setting> */
static portBASE_TYPE prvBrCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CLI commands definition structures & registration
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
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
        .pcCommand                      = ( const int8_t * const ) "home",
        .pcHelpString                   = ( const int8_t * const ) "home        :    Go to home cart positon - center of the track, no controller used\r\n",
        .pxCommandInterpreter           = prvHomeCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "dpc",
        .pcHelpString                   = ( const int8_t * const ) "dpc         :    Turn on/off down position controller, default cart pos. setpoint is current cart position\r\n                 dpc on/off, or dpc 1/0, available only in DEFAULT state\r\n",
        .pxCommandInterpreter           = prvDpcCommand,
        .cExpectedNumberOfParameters    = 1
    },
    {
        .pcCommand                      = ( const int8_t * const ) "clc",
        .pcHelpString                   = ( const int8_t * const ) "clc         :    Clears console screen\r\n",
        .pxCommandInterpreter           = prvClcCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "sppot",
        .pcHelpString                   = ( const int8_t * const ) "sppot       :    Change cart possition setpoint source to potentiometer\r\n",
        .pxCommandInterpreter           = prvSPPOTCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "spcli",
        .pcHelpString                   = ( const int8_t * const ) "spcli       :    Change cart possition setpoint source to CLI\r\n",
        .pxCommandInterpreter           = prvSPCLICommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "sp",
        .pcHelpString                   = ( const int8_t * const ) "sp          :    Change cart possition setpoint source to CLI\r\n                 When run with no arguemnt display current cart position setpoint\r\n",
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
        .pcCommand                      = ( const int8_t * const ) "reset", 
        .pcHelpString                   = ( const int8_t * const ) "reset       :    Resets uC\r\n",
        .pxCommandInterpreter           = prvRstUCCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "rr", 
        .pcHelpString                   = ( const int8_t * const ) "rr          :    Alias for \"reset\" command\r\n",
        .pxCommandInterpreter           = prvRstUCCommand,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "vol", 
        .pcHelpString                   = ( const int8_t * const ) "vol         :    Manually set DC motor voltage\r\n",
        .pxCommandInterpreter           = prvVolCommand,
        .cExpectedNumberOfParameters    = 1
    },
    {
        .pcCommand                      = ( const int8_t * const ) "br", 
        .pcHelpString                   = ( const int8_t * const ) "br          :    Brake, sets output voltage to zero, suspend any active control task\r\n",
        .pxCommandInterpreter           = prvBrCommand,
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
/* [x] command: task-stats 
Availabe in all states. */
static portBASE_TYPE prvTaskStatsCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    const int8_t *const pcTaskTableHeader = ( int8_t * ) "Task          State  Priority  Stack	#\r\n******************************************\r\n";

    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Generate a table of task stats. */
    strcpy( ( char * ) pcWriteBuffer, ( const char * ) pcTaskTableHeader );
    vTaskList( (char *) pcWriteBuffer + strlen( ( const char * ) pcTaskTableHeader ) );
    // configSTATS_BUFFER_MAX_LENGTH

    return pdFALSE;
}

/* [x] command: <enter_key> (data logging on/off)
Availabe in all states. */
static portBASE_TYPE prvComOnOffCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
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

    return pdFALSE;
}

/* [x] command: home
Available in all states except swingup state. */
static portBASE_TYPE prvHomeCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    // /* This command will send notification to worker task that will take action based on notification value/index (?) */
    // char msg_buffer[70];

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
            xTaskNotifyIndexed( cartWorkerTaskHandle,     /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                GO_RIGHT,                 /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

            // sprintf( msg_buffer, "\r\nCart already in min position, going right.\r\n" ); // 44 chars
            // com_send( msg_buffer, strlen( msg_buffer ) );
            strcpy( ( char * ) pcWriteBuffer, "\r\nCart already in min position, going right.\r\n" );
        }
        else
        {
            /* CALIBRATION - Cart position is arbitrary.
            Send notification to worker_task,
            set notification bit 1 - go left until min position reached, then go right to track center. */
            xTaskNotifyIndexed( cartWorkerTaskHandle,     /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                GO_LEFT+GO_RIGHT,         /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

            // sprintf( msg_buffer, "\r\nCart position arbitrary, going left, then to track center.\r\n" ); // 59 chars
            // com_send( msg_buffer, strlen( msg_buffer ) );
            strcpy( ( char * ) pcWriteBuffer, "\r\nCart position arbitrary, going left, then to track center.\r\n" );
        }
    }
    else if( app_current_state == DEFAULT )
    {
        /* App is not in DEFAULT state - cart position should be calibrated. */
        if( cart_position[ 0 ] < TRACK_LEN_MAX_CM/2 )
        {
            /* Cart is to the left of track center. */
            xTaskNotifyIndexed( cartWorkerTaskHandle,     /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                GO_RIGHT,                 /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

            // sprintf( msg_buffer, "\r\nGoing right\r\n" );
            // com_send( msg_buffer, strlen( msg_buffer ) );
            strcpy(  ( char * ) pcWriteBuffer, "\r\nGoing right\r\n" );
        }
        else
        {
            /* Cart is to the right of track center. */
            xTaskNotifyIndexed( cartWorkerTaskHandle,     /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                GO_LEFT,                  /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

            // sprintf( msg_buffer, "\r\nGoing left\r\n" );
            // com_send( msg_buffer, strlen( msg_buffer ) );
            strcpy(  ( char * ) pcWriteBuffer, "\r\nGoing left\r\n" );
        }
    }
    else if( app_current_state == DPC || app_current_state == UPC )
    {
        /* App is not in DEFAULT or UNINITIALIZED state, 
        its either in DOWN_POS_CONTROL(DPC) or UP_POSITION_CONTROL(UPC) state. While in either one of these two control states,
        calling "home" command should change cart position setpoint to home position (center of the track). Command not avaliable in swinggup state. */
        if( cart_position_setpoint_cm == &cart_position_setpoint_cm_cli )
        {
            /* This command should only make changes to cart position setpoint value - if and only if - the source of setpoint 
            is set to setpoint from cli command "spcli", otherwise if the source of setpoint is external potentiometer, which reading 
            can't be overwritten, the value of this setpoint will be arbitrary - the same as physical pot setting, and 
            the behaviour of cart will be less predictable. */
            xTaskNotifyIndexed( cartWorkerTaskHandle,     /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                SP_HOME,                  /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */
        
            // sprintf( msg_buffer, "\r\nController cart setpoint changed to home position.\r\n" );
            // com_send( msg_buffer, strlen( msg_buffer ) );
            strcpy(  ( char * ) pcWriteBuffer, "\r\nController cart setpoint changed to home position.\r\n" );
        }
    }
    // else if( app_current_state == SWINGUP )
    else
    {
        /* App is in swingup state. */
        // sprintf( msg_buffer, "\r\nERROR: COMMAND NOT AVAILABLE IN SWINGUP STATE.\r\n" );
        // com_send( msg_buffer, strlen( msg_buffer ) );
        strcpy(  ( char * ) pcWriteBuffer, "\r\nERROR: COMMAND NOT AVAILABLE IN SWINGUP STATE.\r\n" );
    }

    return pdFALSE;
}

/* [x] command: dpc */
static portBASE_TYPE prvDpcCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    int8_t *pcParameter1;
    BaseType_t xParameter1StringLength;

    /* Get first command arguemnt. */
    pcParameter1 = ( int8_t * ) FreeRTOS_CLIGetParameter( pcCommandString,            /* The command string itself. */
                                                          1,                          /* Which parameter to return. */
                                                          &xParameter1StringLength);  /* Store the parameter string length. */
    
    /* Terminate arguemnt string. */
    pcParameter1[ xParameter1StringLength ] = 0x00;

    if( !strcmp( ( const char * ) pcParameter1, "off" ) || !strcmp( ( const char * ) pcParameter1, "0" ) )
    {
        /* Controller Turn off case. */
        /* Turn off down position controller, "dcp off" / "dpc 0" are both valid commands. */
        vTaskSuspend( ctrl_3_FSF_downpos_task_handle );
        dcm_set_output_volatage( 0.0f );
        
        /* Change current app state back to DEFAULT. */
        app_current_state = DEFAULT;
    }
    else if( !strcmp( ( const char * ) pcParameter1, "on" ) || !strcmp( ( const char * ) pcParameter1, "1" ) )
    {
        /* Controller turn on case. */
        /* Even if controller is turned on, it will only work if the pendulum angle is in range [switch_angle_low, switch_angle_high]. */

        /* Ensure that setpoint for cart postion from cli is the same as the setpoint used by controller tasks.
        If it's not true, this means that the user changed source of cart pos. setpoint for controllers. */
        if( cart_position_setpoint_cm == &cart_position_setpoint_cm_cli )
        {
            /* Set starting setpoint for cart position to its current position, so that the cart won't 
            instantly jump when the controller is turned on. Main cart position setpoint
            used by any controller task has to be the same as cart position set point from cli */
            cart_position_setpoint_cm_cli = cart_position[ 1 ];
            
            if( app_current_state == DEFAULT )
            {
                /* This command should only turn on "down position controller" when app/pendulum is in the DEFAULT state.
                This means that it's not possible to use this command while app is in UNINITIALIZED, SWINGUP or UPPOSITION CONTROLLER state. */
                
                /* Turn on down position controller, "dcp on" / "dpc 1" are both valid commands. */
                vTaskResume( ctrl_3_FSF_downpos_task_handle );
                
                /* Change app state to "down position controller" state. 
                This will ensure that some cli commands can't be called. */
                app_current_state = DPC;
            }
        }
        else
        {
            /* Prompt the use to change setpoint source to cli with "spcli" command. */
            strcpy( ( char * ) pcWriteBuffer, "\r\nERROR: SET CART POSITION SETPOINT SOURCE TO CLI WITH COMMAND: spcli\r\n" );
        }
    }
    else
    {
        /* Command arguments was neither "on", "1", "off" or "0". */
        strcpy( ( char * ) pcWriteBuffer, "ERROR: INVALID ARGUMENT, SHOULD BE: on, 1, off, 0\r\n" );
    }

    return pdFALSE;
}

/* [x] command: clc */
static portBASE_TYPE prvClcCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Send clear screen char sequence. */
    com_send("\e[1;1H\e[2J", 10);
    // printf("\e[1;1H\e[2J");

    return pdFALSE;
}

/* [ ] command: sppot */
static portBASE_TYPE prvSPPOTCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    return pdFALSE;
}

/* [ ] command: spcli */
static portBASE_TYPE prvSPCLICommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    return pdFALSE;
}

/* [ ] command: sp */
static portBASE_TYPE prvSPCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    return pdFALSE;
}

/* [ ] command: swingup */
static portBASE_TYPE prvSWINGUPCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    return pdFALSE;
}

/* [x] command: reset
Available in: ALL STATES. */
static portBASE_TYPE prvRstUCCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    dcm_set_output_volatage( 0.0f );
    /* Resets whole micro controller. The same function is used in hard fault interrupt handler. */
    NVIC_SystemReset();

    return pdFALSE;
}

/* [x] command: vol
Available in: UNINITIALIZED and DEFAULT states. */
static portBASE_TYPE prvVolCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )  
{
    configASSERT( pcWriteBuffer );
    ( void ) xWriteBufferLen;

    int8_t *pcParameter1;
    BaseType_t xParameter1StringLength;
    float voltageSetting = 0.0f;
    char* errCheck;

    if( app_current_state == UNINITIALIZED || app_current_state == DEFAULT )
    {
        /* Command available only in this two states. */
        pcParameter1 = ( int8_t * ) FreeRTOS_CLIGetParameter( pcCommandString,            /* The command string itself. */
                                                              1,                          /* Return the first parameter. */
                                                              &xParameter1StringLength);  /* Store the parameter string length. */
        
        /* Terminate command string. */
        pcParameter1[ xParameter1StringLength ] = 0x00;
        
        voltageSetting = strtof( ( const char * )pcParameter1, &errCheck );
        if( ( int8_t * ) errCheck == pcParameter1 ) 
        {
            strcpy( ( char * ) pcWriteBuffer, ( const char * ) "\r\nERROR: ARGUMENT HAS TO BE A NUMBER\r\n" );
        }
        else
        {
            sprintf( ( char * ) pcWriteBuffer, "\r\nVoltage set to: %f\r\n", (double) voltageSetting );
            dcm_set_output_volatage( voltageSetting );
        }
    }
    else
    {
        /* Inform the use that they can't use this command in current state. */
        strcpy( ( char * ) pcWriteBuffer, "\r\nERROR: This command is available in UNINITIALIZED and DEFAULT states\r\n" );
    }

    return pdFALSE;
}

/* [x] command: br 
Avaliable in: ALL STATES. */
static portBASE_TYPE prvBrCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Suspend any control task. Calls to vTaskSuspend are not cumulative 
    so it can be used with task which is already suspended and one vTaskResume will
    be enoguh to bring that task back to work. */
    vTaskSuspend( ctrl_3_FSF_downpos_task_handle );
    // vTaskSuspend( UP_POSITION_CONTROL_TASK );
    // vTaskSuspend( SWINGUP_TASK );

    /* Set dc motor input voltage to zero. */
    dcm_set_output_volatage( 0.0f );

    return pdFALSE;
}
