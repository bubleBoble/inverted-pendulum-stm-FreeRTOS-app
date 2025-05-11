/* =============================================================================
 * This file contains task that implements freeRTOS cli.
 * With prompt and backspace support.
 * Awesome tutorial on how to make freeRTOS console better:
 *     https://www.edwinfairchild.com/p/making-freertos-cli-more-cli-ish_14.html
 * =============================================================================
 */
#include "LIP_tasks_common.h"

// =============================================================================
// App globals defined in LIP_tasks_common.c
// =============================================================================
extern uint8_t cRxedChar;
extern enum lip_app_states app_current_state;

// =============================================================================
// Prompt
// =============================================================================
char msg[100];
typedef struct {
        // Main prompt string
        char *promptStr;
        // Preprompt string while in alternate prompt mode
        char *prePromptStr;
        // altPromptActive can be changed in command callback function or inside
        // resumed task to indicate current program state, eg. controller used
        uint8_t altPromptActive;
        uint8_t pausePrompt;
} prompt_t;

// Prompt instance
prompt_t prompt;
// Main prompt
char promptStr[] = ">>> ";
// Each state has its own preprompt
char no_prePrompt[] = "[       ?       ]";
char uninitState_prePrompt[] = "[ uninitialzied ]";
char defaultState_prePrompt[] = "[    default    ]";
char dpcState_prePrompt[] = "[      dpc      ]";
char upcState_prePrompt[] = "[      upc      ]";
char swingupState_prePrompt[] = "[    swingup    ]";
char testState_prePrompt[] = "[     tests     ]";

// Function to print full prompt with preprompt string which indicates current
// app state
void show_prompt(void)
{
        switch (app_current_state) {
        case UNINITIALIZED:
                prompt.prePromptStr = uninitState_prePrompt;
                break;
        case DEFAULT:
                prompt.prePromptStr = defaultState_prePrompt;
                break;
        case DPC:
                prompt.prePromptStr = dpcState_prePrompt;
                break;
        case UPC:
                prompt.prePromptStr = upcState_prePrompt;
                break;
        case SWINGUP:
                prompt.prePromptStr = swingupState_prePrompt;
                break;
        case TEST:
                prompt.prePromptStr = testState_prePrompt;
                break;
        default:
                prompt.prePromptStr = no_prePrompt;
                break;
        }

        sprintf(msg, "\r\n%s %s", prompt.prePromptStr, prompt.promptStr);
        com_send(msg, strlen(msg));
        // printf( "\r\n%s %s", prompt.prePromptStr, prompt.promptStr );
        // fflush( stdout );
}

// CLI escape sequences to perform backspace operation in console. backspace,
// print blank and backspace again
// uint8_t backspace[] = "\x08 \x08";
uint8_t backspaceDeleteAction[] = "\b \b";

void console_task(void *pvParameters)
{
        // Keeps track of the number of input characters
        int8_t cInputIndex = 0;
        BaseType_t xMoreDataToFollow;

        // The input and output buffers are declared static to keep
        // them off the stack
        static int8_t pcOutputString[MAX_OUTPUT_LENGTH],
                pcInputString[MAX_INPUT_LENGTH];

        TickType_t xLastWakeTime = xTaskGetTickCount();

        vRegisterCLICommands();

        vTaskDelay(1000);

        // Clear screen before cli start
        com_send("\e[1;1H\e[2J", 10);
        sprintf(msg, "\r\n******************************************\r\n");
        com_send(msg, strlen(msg));
        sprintf(msg, "*********** FreeRTOS based CLI ***********\r\n");
        com_send(msg, strlen(msg));
        sprintf(msg, "******************************************\r\n");
        com_send(msg, strlen(msg));
        // printf( "\r\n******************************************\r\n" );
        // printf(     "*********** FreeRTOS based CLI ***********\r\n" );
        // printf(     "******************************************\r\n" );

        prompt.promptStr = promptStr;
        show_prompt();

        for (;;) {
                if (cRxedChar != 0x00) // better to use notification here
                {
                        if (cRxedChar == '\n' || cRxedChar == '\r') {
                                com_send("\r\n", 2);
                                // printf("\r\n");
                                // fflush(stdout);

                                // The command interpreter is called repeatedly
                                // until it returns pdFALSE.  See the
                                // "Implementing a command" documentation for an
                                // exaplanation of why this is
                                do {
                                        // Send the command string to the
                                        // command interpreter. Any output
                                        // generated by the command interpreter
                                        // will be placed in the pcOutputString
                                        // buffer
                                        xMoreDataToFollow =
                                                FreeRTOS_CLIProcessCommand(
                                                        // The command string
                                                        pcInputString,
                                                        // The output buffer
                                                        pcOutputString,
                                                        // The size of the
                                                        // output buffer
                                                        MAX_OUTPUT_LENGTH);

                                        int limit;
                                        if (xMoreDataToFollow == pdTRUE)
                                                limit = MAX_OUTPUT_LENGTH;
                                        else
                                                limit = strlen(
                                                        (const char *)
                                                                pcOutputString);

                                        for (int i = 0; i < limit; i++) {
                                                sprintf(msg, "%c",
                                                        *(pcOutputString + i));
                                                com_send(msg, strlen(msg));

                                                // printf( "%c", *(pcOutputString + i) );
                                                // fflush(stdout);
                                        }

                                } while (xMoreDataToFollow != pdFALSE);

                                cInputIndex = 0;
                                memset(pcInputString, 0x00, MAX_INPUT_LENGTH);
                                memset(pcOutputString, 0x00, MAX_INPUT_LENGTH);
                                show_prompt();
                        } else {
                                // The if() clause performs the processing after
                                // a newline character is received.  This else
                                // clause performs the processing if any other
                                // character is received
                                if (cRxedChar == '\b') {
                                        //  Backspace was pressed. Erase the
                                        // last character in the input buffer
                                        // - if there are any
                                        if (cInputIndex > 0) {
                                                cInputIndex--;
                                                // pcInputString[ cInputIndex ] = '\0';
                                                // The deleted character is set
                                                // to zero in case the user
                                                // won't type anything else
                                                // after backspace
                                                memset(&pcInputString
                                                               [cInputIndex],
                                                       0x00, 1);

                                                sprintf(msg, "%s",
                                                        (const uint8_t *)
                                                                backspaceDeleteAction);
                                                com_send(msg, strlen(msg));

                                                // printf("%s", (const uint8_t *) backspaceDeleteAction );
                                        }
                                        // fflush(stdout);
                                } else {
                                        // A character was entered.  It was not
                                        // a new line, backspace or carriage
                                        // return, so it is accepted as part of
                                        // the input and placed into the input
                                        // buffer. When a n is entered the
                                        // complete string will be passed to the
                                        // command interpreter
                                        if (cInputIndex < MAX_INPUT_LENGTH) {
                                                pcInputString[cInputIndex] =
                                                        cRxedChar;
                                                cInputIndex++;

                                                sprintf(msg, "%c", cRxedChar);
                                                com_send(msg, strlen(msg));
                                                // printf( "%c", cRxedChar );
                                        }
                                }
                        }
                        cRxedChar = 0x00;
                        // fflush( stdout );
                } // if (cRxedChar != 0x00)
                vTaskDelayUntil(&xLastWakeTime, dt_console);
        } // for( ;; )
} // console_task
