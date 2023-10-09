/*
*   Opis.
*/

#ifndef LIP_MAIN
#define LIP_MAIN

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "list.h"
#include "stream_buffer.h"
#include "task.h"
#include "timers.h"

#include "stm32f4xx_hal.h"
#include "main.h"

#include "stdarg.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

#include "motor_driver.h"
#include "dcm_encoder_driver.h"
#include "com_driver.h"
#include "main_LIP.h"
#include "pend_enc_driver.h"

void main_LIP_init(void);
void main_LIP_run(void);

#endif /* LIP_MAIN */
