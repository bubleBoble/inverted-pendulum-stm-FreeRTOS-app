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
 *     dpci             -    Turn on/off down position controller with integral action on cart position error
 *     upc              -    Turn on/off up position controller
 *     upci             -    Turn on/off up position controller with integral action on cart position error
 *     swingup          -    Turn on pendulum swingup procedure
 *     swingdown
 *     bounceoff        -    Turn on or off cart min max bounce off protection    
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
#include <stdlib.h>     /* For strtox functions (string to float). */
#include <errno.h>      /* error codes. */
#include "math.h"

#include "main_LIP.h"

/* Defined in LIP_tasks_common.c */
extern float cart_position[ 2 ];
extern float pend_angle[ 2 ];
extern float cart_position_setpoint_cm_cli_raw; 
extern float cart_position_setpoint_cm_cli;
extern float cart_position_setpoint_cm_pot; 
extern float *cart_position_setpoint_cm;
extern enum cart_position_zones cart_current_zone;
extern uint32_t bounce_off_action_on;
extern uint32_t swingup_task_resumed;
extern enum lip_app_states app_current_state;
extern uint32_t reset_lookup_index;
extern uint32_t reset_swingdown;
extern uint32_t reset_home;

extern TaskHandle_t watchdog_taskHandle;
extern TaskHandle_t console_taskHandle;
extern TaskHandle_t util_taskHandle;
extern TaskHandle_t com_taskHandle;
extern TaskHandle_t rawcom_taskHandle;
extern TaskHandle_t cartworker_TaskHandle;
extern TaskHandle_t ctrl_downposition_taskHandle;
extern TaskHandle_t ctrl_upposition_taskHandle;
extern TaskHandle_t swingup_task_handle;
extern TaskHandle_t swingdown_task_handle;
extern TaskHandle_t test_task_handle;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CLI commands prototypes
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
/* This command shows task statistics
command : task-stats */
static portBASE_TYPE taskStats_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to turn communication on or off,
command : ENTER_KEY */
static portBASE_TYPE comOnOff_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to move cart to default position - center of the track, no controller is used,
command : home */
static portBASE_TYPE home_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to turn on or off down position controller,
command : dpc */
static portBASE_TYPE dpc_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to turn on/off down position controller with integral action on cart position error,
command : dpci */
static portBASE_TYPE dpci_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to turn on or off up position controller,
command : upc */
static portBASE_TYPE upc_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to turn on or off up position controller with integral action,
command : upci */
static portBASE_TYPE upci_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to clear console screen, at least for putty,
command : clc */
static portBASE_TYPE clc_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to change cart position setpoint to potentiometer,
command: sppot */
static portBASE_TYPE sppot_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to change cart position setpoint to cli,
command: spcli */
static portBASE_TYPE spcli_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to change current cart position setpoint,
command: sp */
static portBASE_TYPE sp_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to start swingup action,
command: swingup */
static portBASE_TYPE swingup_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to start swingdown action,
command: swingdown */
static portBASE_TYPE swingdown_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to reset uC,
command: reset */
static portBASE_TYPE reset_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to manually set dc motor output voltage (via setpoint pointer so it is filtered / smooth),
command: vol <voltage_setting> */
static portBASE_TYPE vol_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* Command to brake the cart, sets dc motor output voltage instantly to zero,
command: br <voltage_setting> */
static portBASE_TYPE br_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* command: bounceoff */
static portBASE_TYPE bounceoff_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* command: test */
static portBASE_TYPE test_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* command: tf1 */
static portBASE_TYPE tf1_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * CLI commands definition structures & registration
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
static const CLI_Command_Definition_t commands_list[] =
{
    {
        .pcCommand                      = ( const int8_t * const ) "task-stats",
        .pcHelpString                   = ( const int8_t * const ) "task-stats  :    Displays a table showing the state of each FreeRTOS task\r\n                 (B)blocked, (R)Ready, (D)Deleted, (S)Suspended or block w/o timeout\r\n",
        .pxCommandInterpreter           = taskStats_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "",
        .pcHelpString                   = ( const int8_t * const ) "<enter_key> :    Start / stop data streaming\r\n",
        .pxCommandInterpreter           = comOnOff_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "q",
        .pcHelpString                   = ( const int8_t * const ) "q :              Start / stop data streaming\r\n",
        .pxCommandInterpreter           = comOnOff_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "home",
        .pcHelpString                   = ( const int8_t * const ) "home        :    Go to home cart positon - center of the track, no controller used\r\n",
        .pxCommandInterpreter           = home_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "dpc",
        .pcHelpString                   = ( const int8_t * const ) "dpc         :    Turn on/off down position controller, default cart pos. setpoint is current cart position\r\n                 dpc on/off, or dpc 1/0, available only in DEFAULT state\r\n",
        .pxCommandInterpreter           = dpc_command,
        .cExpectedNumberOfParameters    = 1
    },
    {
        .pcCommand                      = ( const int8_t * const ) "dpci",
        .pcHelpString                   = ( const int8_t * const ) "dpci        :    Turn on/off down position controller with integral action on cart position error\r\n                 dpc on/off, or dpc 1/0, available only in DEFAULT state\r\n",
        .pxCommandInterpreter           = dpci_command,
        .cExpectedNumberOfParameters    = 1
    },
    {
        .pcCommand                      = ( const int8_t * const ) "upc",
        .pcHelpString                   = ( const int8_t * const ) "upc         :    Turn on/off up position controller, default cart pos. setpoint is current cart position\r\n                 dpc on/off, or dpc 1/0, available only in DEFAULT state\r\n",
        .pxCommandInterpreter           = upc_command,
        .cExpectedNumberOfParameters    = 1
    },
    {
        .pcCommand                      = ( const int8_t * const ) "upci",
        .pcHelpString                   = ( const int8_t * const ) "upci        :    Turn on/off up position controller with integral action on cart position error\r\n                 dpc on/off, or dpc 1/0, available only in DEFAULT state\r\n",
        .pxCommandInterpreter           = upci_command,
        .cExpectedNumberOfParameters    = 1
    },
    {
        .pcCommand                      = ( const int8_t * const ) "clc",
        .pcHelpString                   = ( const int8_t * const ) "clc         :    Clears console screen\r\n",
        .pxCommandInterpreter           = clc_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "sppot",
        .pcHelpString                   = ( const int8_t * const ) "sppot       :    Change cart possition setpoint source to potentiometer\r\n",
        .pxCommandInterpreter           = sppot_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "spcli",
        .pcHelpString                   = ( const int8_t * const ) "spcli       :    Change cart possition setpoint source to CLI\r\n",
        .pxCommandInterpreter           = spcli_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "sp",
        .pcHelpString                   = ( const int8_t * const ) "sp          :    Change cart possition setpoint source to CLI\r\n                 When run with no arguemnt display current cart position setpoint\r\n",
        .pxCommandInterpreter           = sp_command,
        .cExpectedNumberOfParameters    = 1
    },
    {
        .pcCommand                      = ( const int8_t * const ) "swingup",
        .pcHelpString                   = ( const int8_t * const ) "swingup     :    Start pendulum swingup routine\r\n",
        .pxCommandInterpreter           = swingup_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "swu",
        .pcHelpString                   = ( const int8_t * const ) "swu         :    Alias for swingup command\r\n",
        .pxCommandInterpreter           = swingup_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "swingdown",
        .pcHelpString                   = ( const int8_t * const ) "swingdown   :    Start pendulum swingdown routine\r\n",
        .pxCommandInterpreter           = swingdown_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "swd",
        .pcHelpString                   = ( const int8_t * const ) "swd         :    Alias for swingdown command\r\n",
        .pxCommandInterpreter           = swingdown_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "reset", 
        .pcHelpString                   = ( const int8_t * const ) "reset       :    Resets uC\r\n",
        .pxCommandInterpreter           = reset_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "rr", 
        .pcHelpString                   = ( const int8_t * const ) "rr          :    Alias for \"reset\" command\r\n",
        .pxCommandInterpreter           = reset_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "vol", 
        .pcHelpString                   = ( const int8_t * const ) "vol         :    Manually set DC motor voltage\r\n",
        .pxCommandInterpreter           = vol_command,
        .cExpectedNumberOfParameters    = 1
    },
    {
        .pcCommand                      = ( const int8_t * const ) "v", 
        .pcHelpString                   = ( const int8_t * const ) "v           :    Alias for \"vol\" command\r\n",
        .pxCommandInterpreter           = vol_command,
        .cExpectedNumberOfParameters    = 1
    },
    {
        .pcCommand                      = ( const int8_t * const ) "br", 
        .pcHelpString                   = ( const int8_t * const ) "br          :    Brake, sets output voltage to zero, suspend any active control task\r\n",
        .pxCommandInterpreter           = br_command,
        .cExpectedNumberOfParameters    = 0
    },
    {
        .pcCommand                      = ( const int8_t * const ) "bo", 
        .pcHelpString                   = ( const int8_t * const ) "bo          :    Turn on or off cart min max bounce off protection\r\n",
        .pxCommandInterpreter           = bounceoff_command,
        .cExpectedNumberOfParameters    = 1
    },
    {
        .pcCommand                      = ( const int8_t * const ) "test", 
        .pcHelpString                   = ( const int8_t * const ) "test        :    Starts a test procedure\r\n",
        .pxCommandInterpreter           = test_command,
        .cExpectedNumberOfParameters    = 1
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
static portBASE_TYPE taskStats_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    // const int8_t *const pcTaskTableHeader = ( int8_t * ) "Task          State  Priority  Stack	#\r\n******************************************\r\n";

    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Generate a table of task stats. */
    // strcpy( ( char * ) pcWriteBuffer, ( const char * ) pcTaskTableHeader );
    // vTaskList( (char *) pcWriteBuffer + strlen( ( const char * ) pcTaskTableHeader ) );
    // configSTATS_BUFFER_MAX_LENGTH

    return pdFALSE;
}

/* command: <enter_key> (data logging on/off) */
static portBASE_TYPE comOnOff_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    if( eTaskGetState( com_taskHandle ) == eSuspended )
    {
        vTaskResume( com_taskHandle );
    }
    else
    {
        vTaskSuspend( com_taskHandle );
    }

    return pdFALSE;
}

/* command: home */
static portBASE_TYPE home_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    /* This command will send notification to worker task that will take action based on notification value. */

    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Set reset_home flag to 1, it should be reset to 0 in cart_worker task. */
    reset_home = 1;

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
            xTaskNotifyIndexed( cartworker_TaskHandle,     /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                GO_RIGHT,                 /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

            // sprintf( msg_buffer, "\r\nCart already in min position, going right.\r\n" ); // 44 chars
            // com_send( msg_buffer, strlen( msg_buffer ) );
            // strcpy( ( char * ) pcWriteBuffer, "\r\nCart already in min position, going right.\r\n" );
        }
        else
        {
            /* CALIBRATION - Cart position is arbitrary.
            Send notification to worker_task,
            set notification bit 1 - go left until min position reached, then go right to track center. */
            xTaskNotifyIndexed( cartworker_TaskHandle,     /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                GO_LEFT+GO_RIGHT,         /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

            // sprintf( msg_buffer, "\r\nCart position arbitrary, going left, then to track center.\r\n" ); // 59 chars
            // com_send( msg_buffer, strlen( msg_buffer ) );
            // strcpy( ( char * ) pcWriteBuffer, "\r\nCart position arbitrary, going left, then to track center.\r\n" );
        }
    }
    else if( app_current_state == DEFAULT )
    {
        /* App is not in DEFAULT state - cart position should be calibrated. */
        if( cart_position[ 0 ] < TRACK_LEN_MAX_CM/2 )
        {
            /* Cart is to the left of track center. */
            xTaskNotifyIndexed( cartworker_TaskHandle,     /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                GO_RIGHT,                 /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

            // sprintf( msg_buffer, "\r\nGoing right\r\n" );
            // com_send( msg_buffer, strlen( msg_buffer ) );
            // strcpy(  ( char * ) pcWriteBuffer, "\r\nGoing right\r\n" );
        }
        else
        {
            /* Cart is to the right of track center. */
            xTaskNotifyIndexed( cartworker_TaskHandle,     /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                GO_LEFT,                  /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

            // sprintf( msg_buffer, "\r\nGoing left\r\n" );
            // com_send( msg_buffer, strlen( msg_buffer ) );
            // strcpy(  ( char * ) pcWriteBuffer, "\r\nGoing left\r\n" );
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
            xTaskNotifyIndexed( cartworker_TaskHandle,     /* Task to notify. */
                                0,                        /* Notification index is 0. */
                                SP_HOME,                  /* Used to update notification value. */
                                eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */
        
            // sprintf( msg_buffer, "\r\nController cart setpoint changed to home position.\r\n" );
            // com_send( msg_buffer, strlen( msg_buffer ) );
            // strcpy(  ( char * ) pcWriteBuffer, "\r\nController cart setpoint changed to home position.\r\n" );
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

/* command: dpc */
static portBASE_TYPE dpc_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
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
        vTaskSuspend( ctrl_downposition_taskHandle );
        dcm_set_output_volatage( 0.0f );
        
        /* Change current app state back to DEFAULT. */
        app_current_state = DEFAULT;
    }
    else if( !strcmp( ( const char * ) pcParameter1, "on" ) || !strcmp( ( const char * ) pcParameter1, "1" ) )
    {
        if( cart_current_zone != FREEZING_ZONE_L || cart_current_zone != FREEZING_ZONE_R )
        {
            /* Controller turn on case. */
            /* Even if controller is turned on, it will only work if the pendulum angle is in range [switch_angle_low, switch_angle_high]. */

            /* Ensure that setpoint for cart postion from cli is the same as the setpoint used by controller tasks.
            If it's not true, this means that the user changed source of cart pos. setpoint for controllers. 
            ALL CONTROLLER TASKS SHOULD BE TURNING ON WITH CART POS. SETPOINT SOURCE SET TO CLI, OTHERWISE DON'T TURN ON CONTROLLER. */
            if( cart_position_setpoint_cm == &cart_position_setpoint_cm_cli )
            {
                // /* Set starting setpoint for cart position to its current position, so that the cart won't 
                // instantly jump when the controller is turned on. Main cart position setpoint
                // used by any controller task has to be the same as cart position set point from cli */
                // cart_position_setpoint_cm_cli_raw = cart_position[ 0 ];
                
                if( app_current_state == DEFAULT )
                {
                    /* This command should only turn on "down position controller" when app/pendulum is in the DEFAULT state.
                    This means that it's not possible to use this command while app is in UNINITIALIZED, SWINGUP or UPPOSITION CONTROLLER state. */
                    
                    /* Turn on down position controller, "dcp on" / "dpc 1" are both valid commands. */
                    vTaskResume( ctrl_downposition_taskHandle );
                    
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
            strcpy( ( char * ) pcWriteBuffer, "\r\nERROR: CAN'T TURN ON DPC, CART TOO CLOSE TO TRACK LIMITS\r\n" );
        }
    }
    else
    {
        /* Command parameters were neither "on", "1", "off" or "0". */
        strcpy( ( char * ) pcWriteBuffer, "ERROR: INVALID PARAMETER VALUE, SHOULD BE: on, 1, off, 0\r\n" );
    }

    return pdFALSE;
}

/* command: dpci */
static portBASE_TYPE dpci_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
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
        vTaskSuspend( ctrl_downposition_taskHandle );
        dcm_set_output_volatage( 0.0f );
        
        /* Change current app state back to DEFAULT. */
        app_current_state = DEFAULT;
    }
    else if( !strcmp( ( const char * ) pcParameter1, "on" ) || !strcmp( ( const char * ) pcParameter1, "1" ) )
    {
        if( cart_current_zone != FREEZING_ZONE_L || cart_current_zone != FREEZING_ZONE_R )
        {
            /* Controller turn on case. */
            /* Even if controller is turned on, it will only work if the pendulum angle is in range [switch_angle_low, switch_angle_high]. */

            /* Ensure that setpoint for cart postion from cli is the same as the setpoint used by controller tasks.
            If it's not true, this means that the user changed source of cart pos. setpoint for controllers. 
            ALL CONTROLLER TASKS SHOULD BE TURNING ON WITH CART POS. SETPOINT SOURCE SET TO CLI, OTHERWISE DON'T TURN ON CONTROLLER. */
            if( cart_position_setpoint_cm == &cart_position_setpoint_cm_cli )
            {
                // /* Set starting setpoint for cart position to its current position, so that the cart won't 
                // instantly jump when the controller is turned on. Main cart position setpoint
                // used by any controller task has to be the same as cart position set point from cli */
                // cart_position_setpoint_cm_cli_raw = cart_position[ 0 ];
                
                if( app_current_state == DEFAULT )
                {
                    /* This command should only turn on "down position controller" when app/pendulum is in the DEFAULT state.
                    This means that it's not possible to use this command while app is in UNINITIALIZED, SWINGUP or UPPOSITION CONTROLLER state. */
                    
                    /* Turn on down position controller, "dcp on" / "dpc 1" are both valid commands. */
                    vTaskResume( ctrl_downposition_taskHandle );
                    
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
            strcpy( ( char * ) pcWriteBuffer, "\r\nERROR: CAN'T TURN ON DPC, CART TOO CLOSE TO TRACK LIMITS\r\n" );
        }
    }
    else
    {
        /* Command parameters were neither "on", "1", "off" or "0". */
        strcpy( ( char * ) pcWriteBuffer, "ERROR: INVALID PARAMETER VALUE, SHOULD BE: on, 1, off, 0\r\n" );
    }

    return pdFALSE;
}

/* command: upc */
static portBASE_TYPE upc_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
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
        /* Turn off controller, "upc off" / "upc 0" are both valid commands. */
        vTaskSuspend( ctrl_upposition_taskHandle );
        dcm_set_output_volatage( 0.0f );
        
        /* Change current app state back to DEFAULT. */
        app_current_state = DEFAULT;
    }
    else if( !strcmp( ( const char * ) pcParameter1, "on" ) || !strcmp( ( const char * ) pcParameter1, "1" ) )
    {
        if( cart_current_zone != FREEZING_ZONE_L || cart_current_zone != FREEZING_ZONE_R )
        {
            /* Controller turn on case. */
            /* Even if controller is turned on, it will only work if the pendulum angle is in range [switch_angle_low, switch_angle_high]. */

            /* Ensure that setpoint for cart postion from cli is the same as the setpoint used by controller tasks.
            If it's not true, this means that the user changed source of cart pos. setpoint for controllers. 
            ALL CONTROLLERS TASKS SHOULD BE TURNING ON WITH CART POS. SETPOINT SOURCE SET TO CLI, OTHERWISE DON'T TURN ON CONTROLLER. */
            if( cart_position_setpoint_cm == &cart_position_setpoint_cm_cli )
            {
                // /* Set starting setpoint for cart position to its current position, so that the cart won't 
                // instantly jump when the controller is turned on. Main cart position setpoint
                // used by any controller task has to be the same as cart position set point from cli */
                // cart_position_setpoint_cm_cli_raw = cart_position[ 0 ];
                
                if( app_current_state == DEFAULT )
                {
                    /* This command should only turn on "up position controller" when app is in the DEFAULT state.
                    This means that it's not possible to use this command while app is in UNINITIALIZED, SWINGUP or DOWN POSITION CONTROLLER state. */
                    
                    /* Turn on up position controller, "upc on" / "upc 1" are both valid commands. */
                    vTaskResume( ctrl_upposition_taskHandle );
                    
                    /* Change app state to "down position controller" state. 
                    This will ensure that some cli commands can't be called. */
                    app_current_state = UPC;
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
            strcpy( ( char * ) pcWriteBuffer, "\r\nERROR: CAN'T TURN ON UPC, CART TOO CLOSE TO TRACK LIMITS\r\n" );
        }
    }
    else
    {
        /* Command parameters were neither "on", "1", "off" or "0". */
        strcpy( ( char * ) pcWriteBuffer, "ERROR: INVALID PARAMETER VALUE, SHOULD BE: on, 1, off, 0\r\n" );
    }

    return pdFALSE;
}

/* command: upci */
static portBASE_TYPE upci_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
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
        /* Turn off controller, "upc off" / "upc 0" are both valid commands. */
        vTaskSuspend( ctrl_upposition_taskHandle );
        dcm_set_output_volatage( 0.0f );
        
        /* Change current app state back to DEFAULT. */
        app_current_state = DEFAULT;
    }
    else if( !strcmp( ( const char * ) pcParameter1, "on" ) || !strcmp( ( const char * ) pcParameter1, "1" ) )
    {
        if( cart_current_zone != FREEZING_ZONE_L || cart_current_zone != FREEZING_ZONE_R )
        {
            /* Controller turn on case. */
            /* Even if controller is turned on, it will only work if the pendulum angle is in range [switch_angle_low, switch_angle_high]. */

            /* Ensure that setpoint for cart postion from cli is the same as the setpoint used by controller tasks.
            If it's not true, this means that the user changed source of cart pos. setpoint for controllers. 
            ALL CONTROLLERS TASKS SHOULD BE TURNING ON WITH CART POS. SETPOINT SOURCE SET TO CLI, OTHERWISE DON'T TURN ON CONTROLLER. */
            if( cart_position_setpoint_cm == &cart_position_setpoint_cm_cli )
            {
                // /* Set starting setpoint for cart position to its current position, so that the cart won't 
                // instantly jump when the controller is turned on. Main cart position setpoint
                // used by any controller task has to be the same as cart position set point from cli */
                // cart_position_setpoint_cm_cli_raw = cart_position[ 0 ];
                
                if( app_current_state == DEFAULT )
                {
                    /* This command should only turn on "up position controller" when app is in the DEFAULT state.
                    This means that it's not possible to use this command while app is in UNINITIALIZED, SWINGUP or DOWN POSITION CONTROLLER state. */
                    
                    /* Turn on up position controller, "upc on" / "upc 1" are both valid commands. */
                    vTaskResume( ctrl_upposition_taskHandle );
                    
                    /* Change app state to "down position controller" state. 
                    This will ensure that some cli commands can't be called. */
                    app_current_state = UPC;
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
            strcpy( ( char * ) pcWriteBuffer, "\r\nERROR: CAN'T TURN ON UPC, CART TOO CLOSE TO TRACK LIMITS\r\n" );
        }
    }
    else
    {
        /* Command parameters were neither "on", "1", "off" or "0". */
        strcpy( ( char * ) pcWriteBuffer, "ERROR: INVALID PARAMETER VALUE, SHOULD BE: on, 1, off, 0\r\n" );
    }

    return pdFALSE;
}

/* command: clc */
static portBASE_TYPE clc_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Send clear screen char sequence. */
    com_send("\e[1;1H\e[2J", 10);
    // printf("\e[1;1H\e[2J");

    return pdFALSE;
}

/* command: sppot */
static portBASE_TYPE sppot_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Set cart position setpoint source to external potentiometer. */
    cart_position_setpoint_cm = &cart_position_setpoint_cm_pot;
        
    return pdFALSE;
}

/* command: spcli */
static portBASE_TYPE spcli_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Set cart position setpoint source to external potentiometer. */
    cart_position_setpoint_cm = &cart_position_setpoint_cm_cli;

    return pdFALSE;
}

/* command: sp */
static portBASE_TYPE sp_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    int8_t *pcParameter1;
    BaseType_t xParameter1StringLength;
    char* errCheck;

    /* New setpoint for cart position. */
    float new_setpoint;
    
    /* Get first command parameter. */
    pcParameter1 = ( int8_t * ) FreeRTOS_CLIGetParameter( pcCommandString,          /* The command string itself. */
                                                        1,                          /* Which parameter to return. */
                                                        &xParameter1StringLength);  /* Store the parameter string length. */
    
    /* Terminate arguemnt string. */
    pcParameter1[ xParameter1StringLength ] = 0x00;

    if( !strcmp( ( const char * ) pcParameter1, "." ) )
    {
        /* Command "setpoint" was called with "." argument. 
        Display current setpoint. */
        if( cart_position_setpoint_cm == &cart_position_setpoint_cm_cli )
        {
            sprintf( ( char * ) pcWriteBuffer, "\r\nCurrent setpoint is: %f\r\nSource: CLI\r\n", ( double ) *cart_position_setpoint_cm );  
        }
        else
        {
            sprintf( ( char * ) pcWriteBuffer, "\r\nCurrent setpoint is: %f\r\nSource: POT\r\n", ( double ) *cart_position_setpoint_cm );            
        }
    }
    else
    {
        if( app_current_state == DPC || app_current_state == UPC )
        {
            /* User should be able to change cart position setpoint only while in DPC or UPC control states.
            In any other state, calling "sp" command with numeric argument should not be allowed.
            While in default state, the value of cart position setpoint is constantly updated in LIP_util_task.c to current cart position,
            so that when the down controller is turned on there won't be a jump in setpoint value. (cart_position_setpoint_cm_cli_raw) */
     
            /* Parameter passed to "sp" command was not ".". */
            if( cart_position_setpoint_cm == &cart_position_setpoint_cm_cli )
            {
                /* Cart position setpoint source for controllers is setpoint from cli. */
                
                /* Parameter string to float. */
                new_setpoint = strtof( ( const char * ) pcParameter1, &errCheck );
                if( ( int8_t * ) errCheck == pcParameter1 ) 
                {
                    /* Parameter passed is not a number. */
                    strcpy( ( char * ) pcWriteBuffer, ( const char * ) "\r\nERROR: sp PARAMETER HAS TO BE A NUMBER OR \".\"\r\n" );
                }
                else
                {
                    /* Parameter passed is a number, display new setpoint. */
                    // sprintf( ( char * ) pcWriteBuffer, "\r\nNew cart setpoint: %f\r\n", (double) new_setpoint );

                    /* Write new setpoint to _raw cli setpoint (unfiltered). 
                    cart_position_setpoint_cm_cli_raw acts as input to cart_position_setpoint_cm_cli low-pass filter.
                    Low-pass filter is used for both setpoint sources to smooth out discontinous input. */
                    cart_position_setpoint_cm_cli_raw = new_setpoint;
                }
            }
            else
            {
                /* Main cart position setpoint source for controllers isn't setpoint from cli, command 
                should throw an error. */
                strcpy( ( char * ) pcWriteBuffer, "\r\nERROR: TO USE THIS COMMAND, SET SETPOINT SOURCE TO CLI.\r\n" );
            }
        }
    }

    return pdFALSE;
}

/* command: swingup */
static portBASE_TYPE swingup_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    if( app_current_state == DEFAULT )
    {
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
    }
    else
    {
        strcpy( ( char * ) pcWriteBuffer, "ERROR: APP NOT IN DEFAULT STATE OR CART POSITION NOT 20cm\r\n" );
    }

    return pdFALSE;
}

/* command: swingdown */
static portBASE_TYPE swingdown_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    if( app_current_state == UPC )
    {
        /* Reset lookup_index in swingup task for loop. */
        reset_swingdown = 1;

        /* Change app state to swingup. */
        app_current_state = DPC;
        
        /* Resume swingup task. */
        vTaskResume( swingdown_task_handle );
    }
    else
    {
        strcpy( ( char * ) pcWriteBuffer, "ERROR: APP NOT IN DEFAULT STATE OR CART POSITION NOT 20cm\r\n" );
    }

    return pdFALSE;
}

/* command: reset */
static portBASE_TYPE reset_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    dcm_set_output_volatage( 0.0f );
    /* Resets whole micro controller. The same function is used in hard fault interrupt handler. */
    NVIC_SystemReset();

    return pdFALSE;
}

/* command: vol
Available in: UNINITIALIZED and DEFAULT states. */
static portBASE_TYPE vol_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )  
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
            strcpy( ( char * ) pcWriteBuffer, ( const char * ) "\r\nERROR: PARAMETER HAS TO BE A NUMBER\r\n" );
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

/* command: br */
static portBASE_TYPE br_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Set dc motor input voltage to zero. */
    dcm_set_output_volatage( 0.0f );

    /* Change app state to DEFAULT or stay in UNINITIALIZED (if it was the last state). */
    if( app_current_state != UNINITIALIZED )
    {
        app_current_state = DEFAULT;

        /* Suspend any control task. Calls to vTaskSuspend are not cumulative 
        so it can be used on task which is already suspended and one vTaskResume will
        be enoguh to bring that task back to work. */
        vTaskSuspend( ctrl_downposition_taskHandle );
        vTaskSuspend( ctrl_upposition_taskHandle );
        vTaskSuspend( swingup_task_handle );
    }

    /* Set dc motor input voltage to zero again :). */
    dcm_set_output_volatage( 0.0f );

    return pdFALSE;
}

/* command: bo (bounceoff) */
static portBASE_TYPE bounceoff_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    int8_t *pcParameter1;
    BaseType_t xParameter1StringLength;

    /* Get first command argument. */
    pcParameter1 = ( int8_t * ) FreeRTOS_CLIGetParameter( pcCommandString,            /* The command string itself. */
                                                          1,                          /* Which parameter to return. */
                                                          &xParameter1StringLength);  /* Store the parameter string length. */
    
    /* Terminate argumesnt string. */
    pcParameter1[ xParameter1StringLength ] = 0x00;

    if( !strcmp( ( const char * ) pcParameter1, "off" ) || !strcmp( ( const char * ) pcParameter1, "0" ) )
    {
        bounce_off_action_on = 0;
    }
    else if( !strcmp( ( const char * ) pcParameter1, "on" ) || !strcmp( ( const char * ) pcParameter1, "1" ) )
    {
        bounce_off_action_on = 1;
    }

    return pdFALSE;
}

/* command: test */
static portBASE_TYPE test_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    /* This command will send notification to test task that will performe selected test procedure. */

    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    // /* Set reset_test flag to 1, it should be reset to 0 in test task. */
    // reset_test = 1;

    if( app_current_state == UNINITIALIZED )
    {
        strcpy( ( char * ) pcWriteBuffer, "\r\nInitialize first. Run \"home\" command\r\n" );
    }
    else if( app_current_state != DEFAULT )
    {
        strcpy( ( char * ) pcWriteBuffer, "\r\nApp state not DEFAULT. Run \"break\" command\r\n" );
    }
    else if( app_current_state == DEFAULT )
    {
        /* Change app state to TEST. */
        app_current_state = TEST;

        xTaskNotifyIndexed( test_task_handle,         /* Task to notify. */
                            0,                        /* Notification index is 0. */
                            TEST_1,                   /* Used to update notification value. */
                            eSetValueWithOverwrite ); /* Overwrite task notif. value even if it hasn't been read. */

        strcpy( ( char * ) pcWriteBuffer, "\r\nStarting test procedure nr. 1\r\n" );
    }
    else
    {
        /* App not in any known state. */
        strcpy( ( char * ) pcWriteBuffer, "\r\nApp not in valid state. Run \"reset\" command\r\n" );
    }

    return pdFALSE;
}

// /* command: upcg */
// static portBASE_TYPE prvUpcgCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
// {
//     ( void ) pcCommandString;
//     ( void ) xWriteBufferLen;
//     configASSERT( pcWriteBuffer );

//     int8_t *pcParameter1;
//     int8_t *pcParameter2;
//     int8_t *pcParameter3;
//     int8_t *pcParameter4;
//     BaseType_t xParameter1StringLength;
//     BaseType_t xParameter2StringLength;
//     BaseType_t xParameter3StringLength;
//     BaseType_t xParameter4StringLength;

//     char* errCheck;
    
//     /* Get command arguemnts. */
//     pcParameter1 = ( int8_t * ) FreeRTOS_CLIGetParameter( pcCommandString,            /* The command string itself. */
//                                                           1,                          /* Which parameter to return. */
//                                                           &xParameter1StringLength);  /* Store the parameter string length. */
//     pcParameter2 = ( int8_t * ) FreeRTOS_CLIGetParameter( pcCommandString,
//                                                           2,
//                                                           &xParameter2StringLength);       
//     pcParameter3 = ( int8_t * ) FreeRTOS_CLIGetParameter( pcCommandString,
//                                                           3,
//                                                           &xParameter3StringLength);   
//     pcParameter4 = ( int8_t * ) FreeRTOS_CLIGetParameter( pcCommandString,
//                                                           4,
//                                                           &xParameter4StringLength);                                                      
    
//     /* Terminate arguemnt string. */
//     pcParameter1[ xParameter1StringLength ] = 0x00;
//     pcParameter2[ xParameter2StringLength ] = 0x00;
//     pcParameter3[ xParameter3StringLength ] = 0x00;
//     pcParameter4[ xParameter4StringLength ] = 0x00;

//     gains_upc[ 0 ] = strtof( ( const char * )pcParameter1, &errCheck );
//     if( ( int8_t * ) errCheck == pcParameter1 ) 
//     {
//         strcpy( ( char * ) pcWriteBuffer, ( const char * ) "\r\nERROR: PARAMETER HAS TO BE A NUMBER\r\n" );
//     }
//     gains_upc[ 1 ] = strtof( ( const char * )pcParameter2, &errCheck );
//     if( ( int8_t * ) errCheck == pcParameter1 ) 
//     {
//         strcpy( ( char * ) pcWriteBuffer, ( const char * ) "\r\nERROR: PARAMETER HAS TO BE A NUMBER\r\n" );
//     }
//     gains_upc[ 2 ] = strtof( ( const char * )pcParameter3, &errCheck );
//     if( ( int8_t * ) errCheck == pcParameter1 ) 
//     {
//         strcpy( ( char * ) pcWriteBuffer, ( const char * ) "\r\nERROR: PARAMETER HAS TO BE A NUMBER\r\n" );
//     }
//     gains_upc[ 3 ] = strtof( ( const char * )pcParameter4, &errCheck );
//     if( ( int8_t * ) errCheck == pcParameter1 ) 
//     {
//         strcpy( ( char * ) pcWriteBuffer, ( const char * ) "\r\nERROR: PARAMETER HAS TO BE A NUMBER\r\n" );
//     }

//     return pdFALSE;
// }