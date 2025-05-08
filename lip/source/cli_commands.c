/*
 * CLI commands functionality
 *
 * For list of implemented CLI commands see docs/cli_commands.md
 *
 * Note:
 *     commands callback functions change app state, which is indicated by
 *     preprompt string in cli prompt ( (preprompt)>>> ). All logic related to
 *     LIP app state changes (performed by a command) is contained in cli
 *     commands callback functions.
 *
 * All variables related to LIP app state are defined in LIP_tasks_common.c
 */
#include <stdlib.h>  // strtof()
#include <errno.h>   // error codes
#include "math.h"

#include "main_LIP.h"
#include "cli_commands.h"

// =============================================================================
// App globals defined in LIP_tasks_common.c
// =============================================================================
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
extern LP_filter LP_filter_cart;
extern LP_filter LP_filter_pendulum;

extern TaskHandle_t watchdog_task_handle;
extern TaskHandle_t console_task_handle;
extern TaskHandle_t util_task_handle;
extern TaskHandle_t com_task_handle;
extern TaskHandle_t rawcom_task_handle;
extern TaskHandle_t cartworker_TaskHandle;
extern TaskHandle_t ctrl_downposition_task_handle;
extern TaskHandle_t ctrl_upposition_task_handle;
extern TaskHandle_t swingup_task_handle;
extern TaskHandle_t swingdown_task_handle;
extern TaskHandle_t test_task_handle;

// =============================================================================
// CLI commands prototypes
// =============================================================================
static portBASE_TYPE taskStats_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: task-stats

static portBASE_TYPE comOnOff_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: ENTER_KEY

static portBASE_TYPE home_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: home

static portBASE_TYPE dpc_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: dpc

static portBASE_TYPE dpci_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: dpci

static portBASE_TYPE upc_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: upc

static portBASE_TYPE upci_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: upci

static portBASE_TYPE clc_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: clc

static portBASE_TYPE sppot_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: sppot

static portBASE_TYPE spcli_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: spcli

static portBASE_TYPE sp_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: sp

static portBASE_TYPE swingup_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: swingup

static portBASE_TYPE swingdown_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: swingdown

static portBASE_TYPE reset_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: reset

static portBASE_TYPE vol_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: vol <voltage_setting>

static portBASE_TYPE br_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: br

static portBASE_TYPE bounceoff_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: bounceoff

static portBASE_TYPE test_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: test

static portBASE_TYPE tcc_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: tcc

static portBASE_TYPE tcp_command(
    int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString
); // command: tcp

// =============================================================================
// CLI commands definition structures & registration
// =============================================================================
static const CLI_Command_Definition_t commands_list[] =
{
    {
        .pcCommand = ( const int8_t * const ) "task-stats",
        .pcHelpString = ( const int8_t * const ) TASKSTATS_HELPSTR,
        .pxCommandInterpreter = taskStats_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand = ( const int8_t * const ) "",
        .pcHelpString = ( const int8_t * const ) COMONOFF_HELPSTR,
        .pxCommandInterpreter = comOnOff_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = (const int8_t* const) "q",
        .pcHelpString                = (const int8_t* const) COMONOFFQ_HELPSTR,
        .pxCommandInterpreter        = comOnOff_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = (const int8_t* const) "home",
        .pcHelpString                = (const int8_t* const) HELP_HELPSTR,
        .pxCommandInterpreter        = home_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = ( const int8_t * const ) "dpc",
        .pcHelpString                = ( const int8_t * const ) DPC_HELPSTR,
        .pxCommandInterpreter        = dpc_command,
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand                   = ( const int8_t * const ) "dpci",
        .pcHelpString                = ( const int8_t * const ) DPCI_HELPSTR,
        .pxCommandInterpreter        = dpci_command,
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand                   = ( const int8_t * const ) "upc",
        .pcHelpString                = ( const int8_t * const ) UPC_HELPSTR,
        .pxCommandInterpreter        = upc_command,
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand                   = ( const int8_t * const ) "upci",
        .pcHelpString                = ( const int8_t * const ) UPCI_HELPSTR,
        .pxCommandInterpreter        = upci_command,
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand                   = ( const int8_t * const ) "clc",
        .pcHelpString                = ( const int8_t * const ) CLC_HELPSTR,
        .pxCommandInterpreter        = clc_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = ( const int8_t * const ) "sppot",
        .pcHelpString                = ( const int8_t * const ) SPPOT_HELPSTR,
        .pxCommandInterpreter        = sppot_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = ( const int8_t * const ) "spcli",
        .pcHelpString                = ( const int8_t * const ) SPCLI_HELPSTR,
        .pxCommandInterpreter        = spcli_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = ( const int8_t * const ) "sp",
        .pcHelpString                = ( const int8_t * const ) SP_HELPSTR,
        .pxCommandInterpreter        = sp_command,
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand                   = ( const int8_t * const ) "swingup",
        .pcHelpString                = ( const int8_t * const ) SWINGUP_HELPSTR,
        .pxCommandInterpreter        = swingup_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = ( const int8_t * const ) "swu",
        .pcHelpString                = ( const int8_t * const ) SWU_HELPSTR,
        .pxCommandInterpreter        = swingup_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = ( const int8_t * const ) "swingdown",
        .pcHelpString                = ( const int8_t * const ) SWINGDOWN_HELPSTR,
        .pxCommandInterpreter        = swingdown_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = ( const int8_t * const ) "swd",
        .pcHelpString                = ( const int8_t * const ) SWD_HELPSTR,
        .pxCommandInterpreter        = swingdown_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = ( const int8_t * const ) "reset",
        .pcHelpString                = ( const int8_t * const ) RESET_HELPSTR,
        .pxCommandInterpreter        = reset_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = ( const int8_t * const ) "rr",
        .pcHelpString                = ( const int8_t * const ) RR_HELPSTR,
        .pxCommandInterpreter        = reset_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = ( const int8_t * const ) "vol",
        .pcHelpString                = ( const int8_t * const ) VOL_HELPSTR,
        .pxCommandInterpreter        = vol_command,
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand                   = ( const int8_t * const ) "v",
        .pcHelpString                = ( const int8_t * const ) V_HELPSTR,
        .pxCommandInterpreter        = vol_command,
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand                   = ( const int8_t * const ) "br",
        .pcHelpString                = ( const int8_t * const ) BR_HELPSTR,
        .pxCommandInterpreter        = br_command,
        .cExpectedNumberOfParameters = 0
    },
    {
        .pcCommand                   = ( const int8_t * const ) "bo",
        .pcHelpString                = ( const int8_t * const ) BO_HELPSTR,
        .pxCommandInterpreter        = bounceoff_command,
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand                   = ( const int8_t * const ) "test",
        .pcHelpString                = ( const int8_t * const ) TEST_HELPSTR,
        .pxCommandInterpreter        = test_command,
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand                   = ( const int8_t * const ) "tcc",
        .pcHelpString                = ( const int8_t * const ) TCC_HELPSTR,
        .pxCommandInterpreter        = tcc_command,
        .cExpectedNumberOfParameters = 1
    },
    {
        .pcCommand                   = ( const int8_t * const ) "tcp",
        .pcHelpString                = ( const int8_t * const ) TCP_HELPSTR,
        .pxCommandInterpreter        = tcp_command,
        .cExpectedNumberOfParameters = 1
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

//==============================================================================
// CLI commands callback functions definitions
// =============================================================================
// command: task-stats
static portBASE_TYPE taskStats_command(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
                                       const int8_t *pcCommandString)
{
    // const int8_t *const pcTaskTableHeader = ( int8_t * )
    //     "Task          State  Priority  Stack	#\r\n"
    //     "******************************************\r\n";

    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    /* Generate a table of task stats. */
    // strcpy( ( char * ) pcWriteBuffer, ( const char * ) pcTaskTableHeader );
    // vTaskList(
    //     (char *) pcWriteBuffer + strlen( ( const char * ) pcTaskTableHeader )
    // );
    // configSTATS_BUFFER_MAX_LENGTH

    return pdFALSE;
}

// command: <enter_key> (data logging on/off)
static portBASE_TYPE comOnOff_command(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
                                      const int8_t *pcCommandString)
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    if( eTaskGetState( com_task_handle ) == eSuspended )
    {
        vTaskResume( com_task_handle );
    }
    else
    {
        vTaskSuspend( com_task_handle );
    }

    return pdFALSE;
}

// command: home
static portBASE_TYPE home_command(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
                                  const int8_t *pcCommandString)
{
    /* this command sends notification to worker task that will take action
    based on notification value. */
    (void) pcCommandString;
    (void) xWriteBufferLen;
    configASSERT(pcWriteBuffer);

    // set reset_home flag to 1, it should be reset to 0 in cart_worker task
    reset_home = 1;

    if (app_current_state == UNINITIALIZED) {
        // App is in UNINITIALIZED state (right after uC powerup). Cart position
        // can be arbitrary - so it has to be calibrated. First the cart goes to
        // min left position, then moves to the track center
        if (READ_ZERO_POSITION_REACHED) {
            // Cart is already in zero position. Send notification to
            // worker_task, set notification bit 0 - go to the right until cart
            // is in the middle
            xTaskNotifyIndexed(cartworker_TaskHandle, 0, GO_RIGHT,
                               eSetValueWithOverwrite);
        } else {
            // CALIBRATION - Cart position is arbitrary. Send notification to
            // worker_task, set notification bit 1 - go left until min position
            // reached, then go right to track center
            xTaskNotifyIndexed(cartworker_TaskHandle, 0, GO_LEFT+GO_RIGHT,
                               eSetValueWithOverwrite);
        }
    } else if (app_current_state == DEFAULT) {
        // App is not in DEFAULT state - cart position should be calibrated
        if (cart_position[ 0 ] < TRACK_LEN_MAX_CM/2) {
            // Cart is to the left of track center
            xTaskNotifyIndexed(cartworker_TaskHandle, 0, GO_RIGHT,
                               eSetValueWithOverwrite);
        } else {
            // Cart is to the right of track center
            xTaskNotifyIndexed(cartworker_TaskHandle, 0, GO_LEFT,
                               eSetValueWithOverwrite);
        }
    } else if (app_current_state == DPC || app_current_state == UPC) {
        // App is not in DEFAULT or UNINITIALIZED state, its either in
        // DOWN_POS_CONTROL(DPC) or UP_POSITION_CONTROL(UPC) state. While in
        // either one of these two control states, calling "home" command should
        // change cart position setpoint to home position (center of the track).
        //  Command not avaliable in swinggup state.
        if (cart_position_setpoint_cm == &cart_position_setpoint_cm_cli) {
            // This command should only make changes to cart position setpoint
            // value - if and only if - the source of setpoint is set to
            // setpoint from cli command "spcli", otherwise if the source of
            // setpoint is external potentiometer, which reading can't be
            // overwritten, value of this setpoint will be arbitrary - the same
            // as physical pot setting, and the behaviour of cart will be less
            // predictable
            xTaskNotifyIndexed(cartworker_TaskHandle, 0, SP_HOME,
                               eSetValueWithOverwrite);
        }
    } else {
        // App is in swingup state
        strcpy((char*) pcWriteBuffer,
               "\r\nERROR: COMMAND NOT AVAILABLE IN SWINGUP STATE.\r\n");
    }

    return pdFALSE;
}

// command: dpc
static portBASE_TYPE dpc_command(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
                                 const int8_t *pcCommandString)
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    int8_t *pcParameter1;
    BaseType_t xParameter1StringLength;

    // get first command arguemnt
    pcParameter1 = (int8_t*)FreeRTOS_CLIGetParameter(pcCommandString, 1,
                                                     &xParameter1StringLength);

    // terminate arguemnt string
    pcParameter1[ xParameter1StringLength ] = 0x00;

    if (
        !strcmp((const char*) pcParameter1, "off") ||
        !strcmp((const char*) pcParameter1, "0")
    ) {

        // turn off down position controller, "dcp off" / "dpc 0"
        // are both valid commands
        vTaskSuspend( ctrl_downposition_task_handle );
        dcm_set_output_volatage( 0.0f );

        // change current app state back to DEFAULT
        app_current_state = DEFAULT;
    } else if (
        !strcmp((const char*) pcParameter1, "on") ||
        !strcmp( ( const char * ) pcParameter1, "1" )
    ) {
        if(
            cart_current_zone != FREEZING_ZONE_L ||
            cart_current_zone != FREEZING_ZONE_R
        ) {
            // controller turn on case, even if controller is turned on, it will
            // only work if the pendulum angle is in range
            // [switch_angle_low, switch_angle_high].

            // ensure that setpoint for cart postion from cli is the same as the
            // setpoint used by controller tasks. If it's not true, this means
            // that the user changed source of cart pos. setpoint for
            // controllers. ALL CONTROLLER TASKS SHOULD BE TURNING ON WITH
            // CART POS. SETPOINT SOURCE SET TO CLI, OTHERWISE DON'T TURN ON
            // CONTROLLER
            if (cart_position_setpoint_cm == &cart_position_setpoint_cm_cli) {
                // set starting setpoint for cart position to its current
                // position, so that the cart won't instantly jump when the
                // controller is turned on. Main cart position setpoint used by
                // any controller task has to be the same as cart position set
                // point from cli
                // cart_position_setpoint_cm_cli_raw = cart_position[ 0 ];

                if (app_current_state == DEFAULT) {
                    // this command should only turn on "down position
                    // controller" when app/pendulum is in the DEFAULT state.
                    // This means that it's not possible to use this command
                    // while app is in UNINITIALIZED, SWINGUP or UPPOSITION
                    // CONTROLLER state

                    // Turn on down position controller, "dcp on" / "dpc 1"
                    // are both valid commands.
                    vTaskResume(ctrl_downposition_task_handle);

                    // change app state to "down position controller" state.
                    // This will ensure that some cli commands can't be called
                    app_current_state = DPC;
                }
            } else {
                // prompt the user to change setpoint source to cli with
                // "spcli" command
                strcpy(
                    ( char * ) pcWriteBuffer,
                    "\r\nERROR: SET CART POSITION SETPOINT SOURCE TO CLI WITH "
                    "COMMAND: spcli\r\n"
                );
            }
        } else {
            strcpy((char*) pcWriteBuffer,
                   "\r\nERROR: CAN'T TURN ON DPC, CART TOO CLOSE TO TRACK"
                   "LIMITS\r\n");
        }
    } else {
        // command parameters were neither "on", "1", "off" or "0"
        strcpy((char*) pcWriteBuffer,
               "ERROR: INVALID PARAMETER VALUE, SHOULD BE: on, 1, off, 0\r\n");
    }

    return pdFALSE;
}

// command: dpci
static portBASE_TYPE dpci_command(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
                                  const int8_t *pcCommandString)
{
    (void) pcCommandString;
    (void) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    int8_t *pcParameter1;
    BaseType_t xParameter1StringLength;

    // get first command arguemnt
    pcParameter1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
                                                      &xParameter1StringLength);

    // terminate arguemnt string
    pcParameter1[ xParameter1StringLength ] = 0x00;

    if (!strcmp( ( const char * ) pcParameter1, "off") ||
        !strcmp( ( const char * ) pcParameter1, "0")) {
        // turn off down position controller, "dcp off" / "dpc 0" are both valid
        vTaskSuspend( ctrl_downposition_task_handle );
        dcm_set_output_volatage( 0.0f );

        // change current app state back to DEFAULT
        app_current_state = DEFAULT;
    } else if (!strcmp((const char*) pcParameter1, "on") ||
               !strcmp( ( const char * ) pcParameter1, "1")) {
        if ((cart_current_zone != FREEZING_ZONE_L) ||
            (cart_current_zone != FREEZING_ZONE_R)) {
            // controller turn on case. Even if controller is turned on, it will
            // only work if the pendulum angle is in range 
            // [switch_angle_low, switch_angle_high]

            // ensure that setpoint for cart postion from cli is the same as the
            // setpoint used by controller tasks. If it's not true, this means 
            // that the user changed source of cart pos. setpoint for 
            // controllers. ALL CONTROLLER TASKS SHOULD BE TURNING ON WITH CART 
            // POS. SETPOINT SOURCE SET TO CLI, OTHERWISE DON'T TURN ON 
            // CONTROLLER
            if (cart_position_setpoint_cm == &cart_position_setpoint_cm_cli) {
                // set starting setpoint for cart position to its current 
                // position, so that the cart won't instantly jump when the 
                // controller is turned on. Main cart position setpoint used by 
                // any controller task has to be the same as cart position set 
                // point from cli 
                // cart_position_setpoint_cm_cli_raw = cart_position[ 0 ];

                if (app_current_state == DEFAULT) {
                    // this command should only turn on "down position 
                    // controller" when app/pendulum is in the DEFAULT state.
                    // This means that it's not possible to use this command 
                    // while app is in UNINITIALIZED, SWINGUP or UPPOSITION 
                    // CONTROLLER state

                    // turn on down position controller, "dcp on" / "dpc 1" are 
                    // both valid commands
                    vTaskResume( ctrl_downposition_task_handle );

                    // change app state to "down position controller" state.
                    // This will ensure that some cli commands can't be called
                    app_current_state = DPC;
                }
            } else {
                // prompt the use to change setpoint source to cli with "spcli" 
                // command
                strcpy((char*) pcWriteBuffer, 
                       "\r\nERROR: SET CART POSITION SETPOINT SOURCE TO CLI "
                       "WITH COMMAND: spcli\r\n");
            }
        } else {
            strcpy( 
                (char*) pcWriteBuffer, 
                "\r\nERROR: CAN'T TURN ON DPC, CART TOO CLOSE TO TRACK "
                "LIMITS\r\n");
        }
    } else {
        // command parameters were neither "on", "1", "off" or "0"
        strcpy( ( char * ) pcWriteBuffer, 
                "ERROR: INVALID PARAMETER VALUE, SHOULD BE: on, 1, off, 0\r\n");
    }

    return pdFALSE;
}

// command: upc
static portBASE_TYPE upc_command(int8_t *pcWriteBuffer, size_t xWriteBufferLen, 
                                 const int8_t *pcCommandString)
{
    ( void ) pcCommandString;
    ( void ) xWriteBufferLen;
    configASSERT( pcWriteBuffer );

    int8_t *pcParameter1;
    BaseType_t xParameter1StringLength;

    // get first command arguemnt
    pcParameter1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
                                                      &xParameter1StringLength);

    // terminate arguemnt string
    pcParameter1[ xParameter1StringLength ] = 0x00;

    const char *param = (const char *) pcParameter1;

    if (!strcmp(param, "off") || !strcmp(param, "0" )) {
        // turn off controller, "upc off" / "upc 0" are both valid commands
        vTaskSuspend( ctrl_upposition_task_handle );
        dcm_set_output_volatage( 0.0f );

        // change current app state back to DEFAULT
        app_current_state = DEFAULT;
    } else if (!strcmp(param, "on") || !strcmp(param, "1" ) ) {
        
        uint8_t is_in_freezing_zone = cart_current_zone != FREEZING_ZONE_L ||
                                      cart_current_zone != FREEZING_ZONE_R;

        if (is_in_freezing_zone) {
            // controller turn on case. Even if controller is turned on, it 
            // will only work if the pendulum angle is in range 
            // [switch_angle_low, switch_angle_high].

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
                    vTaskResume( ctrl_upposition_task_handle );

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
        vTaskSuspend( ctrl_upposition_task_handle );
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
                    vTaskResume( ctrl_upposition_task_handle );

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
        vTaskSuspend( ctrl_downposition_task_handle );
        vTaskSuspend( ctrl_upposition_task_handle );
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

/* command: tcc */
static portBASE_TYPE tcc_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    /* tcc - time constant cart. */
    // ( void ) pcCommandString;
    // ( void ) xWriteBufferLen;
    // configASSERT( pcWriteBuffer );
    int8_t *command_param_1;
    int8_t *command_param_2;
    BaseType_t command_param_str_len_1;
    BaseType_t command_param_str_len_2;
    char* errCheck;

    float new_time_constant;

    /* Get command arguemnts. */
    command_param_1 = ( int8_t * ) FreeRTOS_CLIGetParameter( pcCommandString,           /* The command string itself. */
                                                             1,                         /* Which parameter to return. */
                                                             &command_param_str_len_1); /* Store the parameter string length. */
    command_param_2 = ( int8_t * ) FreeRTOS_CLIGetParameter( pcCommandString,
                                                             2,
                                                             &command_param_str_len_2);

    /* Terminate arguemnt string. */
    command_param_1[ command_param_str_len_1 ] = 0x00;
    command_param_2[ command_param_str_len_2 ] = 0x00;

    new_time_constant = strtof( ( const char * )command_param_1, &errCheck );
    if( ( int8_t * ) errCheck == command_param_1 )
    {
        strcpy( ( char * ) pcWriteBuffer, ( const char * ) "\r\nERROR: parameter has to be a number\r\n" );
    }
    else
    {
        LP_update_time_Constant( &LP_filter_cart, new_time_constant );
    }

    return pdFALSE;
}

/* command: tcp */
static portBASE_TYPE tcp_command( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
    /* tcp - time constant pendulum. */
    // ( void ) pcCommandString;
    // ( void ) xWriteBufferLen;
    // configASSERT( pcWriteBuffer );
    int8_t *command_param_1;
    int8_t *command_param_2;
    BaseType_t command_param_str_len_1;
    BaseType_t command_param_str_len_2;
    char* errCheck;

    float new_time_constant;

    /* Get command arguemnts. */
    command_param_1 = ( int8_t * ) FreeRTOS_CLIGetParameter( pcCommandString,           /* The command string itself. */
                                                             1,                         /* Which parameter to return. */
                                                             &command_param_str_len_1); /* Store the parameter string length. */
    command_param_2 = ( int8_t * ) FreeRTOS_CLIGetParameter( pcCommandString,
                                                             2,
                                                             &command_param_str_len_2);

    /* Terminate arguemnt string. */
    command_param_1[ command_param_str_len_1 ] = 0x00;
    command_param_2[ command_param_str_len_2 ] = 0x00;

    new_time_constant = strtof( ( const char * )command_param_1, &errCheck );
    if( ( int8_t * ) errCheck == command_param_1 )
    {
        strcpy( ( char * ) pcWriteBuffer, ( const char * ) "\r\nERROR: parameter has to be a number\r\n" );
    }
    else
    {
        LP_update_time_Constant( &LP_filter_pendulum, new_time_constant );
    }

    return pdFALSE;
}
