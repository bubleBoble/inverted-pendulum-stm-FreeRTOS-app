/*
 * This file provides task that pendulum swingdown routine. This task should 
 * run with sampling period of 10ms.
 */
#include "LIP_tasks_common.h"
#include <math.h>

/*
 * App globals defined in LIP_tasks_common.c
 */
extern float                    pend_angle[2];
extern float                    pend_speed[2];
extern float                    cart_pos[2];
extern float                    cart_speed[2];
extern float                   *cart_pos_setp_cm;
extern float                    pendulum_arm_angle_setpoint_rad;
extern enum cart_position_zones cart_current_zone;
extern uint32_t                 reset_swingdown;
extern float                    cart_pos_setp_cm_cli_raw;
extern float                    pendulum_angle_in_base_range_upc;
extern enum lip_app_states      app_current_state;
extern uint32_t                 swingup_task_resumed;
extern TaskHandle_t             ctrl_downpos_task_h;
extern TaskHandle_t             ctrl_uppos_task_h;

void swingdown_task(void *pvParameters)
{
        /* For RTOS vTaskDelayUntil() */
        TickType_t last_wake_time = xTaskGetTickCount();

        // char msg[128];

        for (;;) {
                last_wake_time = xTaskGetTickCount();

                if (reset_swingdown) {
                        if (pendulum_angle_in_base_range_upc < 0.0f) {
                                cart_pos_setp_cm_cli_raw =
                                        TRACK_LEN_MAX_CM / 2.0f;

                                /* Wait for the cart to reach setpoint */
                                vTaskDelay(1000);
                                vTaskSuspend(ctrl_uppos_task_h);

                                /* Help pendulum swing freely in CCW
                                direction */
                                dcm_set_output_volatage(2.0f);
                                vTaskDelay(100);
                                dcm_set_output_volatage(0.0f);

                                vTaskResume(ctrl_downpos_task_h);
                                app_current_state = DPC;

                                reset_swingdown = 0;
                                vTaskSuspend(NULL);
                        } else if (pendulum_angle_in_base_range_upc > 0.0f) {
                                cart_pos_setp_cm_cli_raw =
                                        TRACK_LEN_MAX_CM / 2.0f;

                                /* Wait for the cart to reach setpoint */
                                vTaskDelay(1000);
                                vTaskSuspend(ctrl_uppos_task_h);

                                /* Help pendulum swing freely in CW direction */
                                dcm_set_output_volatage(-2.0f);
                                vTaskDelay(100);
                                dcm_set_output_volatage(0.0f);

                                vTaskResume(ctrl_downpos_task_h);
                                app_current_state = DPC;

                                reset_swingdown = 0;
                                vTaskSuspend(NULL);
                        }
                }

                vTaskDelayUntil(&last_wake_time, dt);
        }
}
