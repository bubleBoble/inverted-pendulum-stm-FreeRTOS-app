#define TASKSTATS_HELPSTR                                                       \
        "task-stats  : Displays a table showing the state of each FreeRTOS\r\n" \
        "              task. (B)locked, (R)eady, (D)eleted, (S)uspended or\r\n" \
        "              blocked w/o timeout\r\n"

#define COMONOFF_HELPSTR "<enter_key> : Start / stop data streaming\r\n"

#define COMONOFFQ_HELPSTR "q           : Start / stop data streaming\r\n"

#define HELP_HELPSTR                                                        \
        "home        : Go to home cart position - center of the track,\r\n" \
        "              no controller used\r\n"

#define DPC_HELPSTR                                                            \
        "dpc         : Turn on/off down position controller. Default cart\r\n" \
        "              position setpoint is current cart position.\r\n"        \
        "              Available only in DEFAULT state: dpc on/off or dpc 1/0\r\n"

#define DPCI_HELPSTR                                                         \
        "dpci        : Down position controller with integral action on\r\n" \
        "              cart position error.\r\n"                             \
        "              Available only in DEFAULT state: dpci on/off or dpci 1/0\r\n"

#define UPC_HELPSTR                                                          \
        "upc         : Turn on/off up position controller. Default cart\r\n" \
        "              position setpoint is current cart position.\r\n"      \
        "              Available only in DEFAULT state: upc on/off or upc 1/0\r\n"

#define UPCI_HELPSTR                                                       \
        "upci        : Up position controller with integral action on\r\n" \
        "              cart position error.\r\n"                           \
        "              Available only in DEFAULT state: upci on/off or upci 1/0\r\n"

#define CLC_HELPSTR "clc         : Clears console screen\r\n"

#define SPPOT_HELPSTR \
        "sppot       : Set cart position setpoint source to potentiometer\r\n"

#define SPCLI_HELPSTR \
        "spcli       : Set cart position setpoint source to CLI\r\n"

#define SP_HELPSTR                                                    \
        "sp          : Set cart position setpoint source to CLI.\r\n" \
        "              No argument: display current cart position setpoint\r\n"

#define SWINGUP_HELPSTR "swingup     : Start pendulum swing-up routine\r\n"

#define SWU_HELPSTR "swu         : Alias for swingup command\r\n"

#define SWINGDOWN_HELPSTR "swingdown   : Start pendulum swing-down routine\r\n"

#define SWD_HELPSTR "swd         : Alias for swingdown command\r\n"

#define RESET_HELPSTR "reset       : Resets microcontroller\r\n"

#define RR_HELPSTR "rr          : Alias for \"reset\" command\r\n"

#define VOL_HELPSTR "vol         : Manually set DC motor voltage\r\n"

#define V_HELPSTR "v           : Alias for \"vol\" command\r\n"

#define BR_HELPSTR                                                              \
        "br          : Brake. Sets output voltage to zero and suspends any\r\n" \
        "              active control task\r\n"

#define BO_HELPSTR "bo          : Toggle cart min/max bounce-off protection\r\n"

#define TEST_HELPSTR "test        : Start a test procedure\r\n"

#define TCC_HELPSTR "tcc         : Start a test procedure\r\n"

#define TCP_HELPSTR "tcp         : Start a test procedure\r\n"
