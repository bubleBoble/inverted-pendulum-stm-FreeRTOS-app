/*
 * This file provides task that is only active when command "zero" or "home" is
 * called. Its purpose is to move the cart without any controller.
 */
#include "LIP_tasks_common.h"
#include "limits.h"

/* voltage value used to move cart without any controller */
#define IDLE_MOVEMENTS_VOLTAGE 1.65f

/*
 * App globals defined in LIP_tasks_common.c
 */
extern enum lip_app_states app_current_state;
extern float               cart_pos[2];
extern float              *cart_pos_setp_cm;
extern float               cart_pos_setp_cm_cli_raw;
extern uint32_t            reset_home;

extern TaskHandle_t cartworker_task_h;

void set_output_voltage_nobounce(float voltage);

void cart_worker_task(void *pvParameters)
{
        /* holds value retrieved from task notification */
        uint32_t notif_val_rcv;

        for (;;) {
                /* wait for notification at index 0 */
                xTaskNotifyWaitIndexed(0, 0x00, ULONG_MAX, &notif_val_rcv,
                                       portMAX_DELAY);
                /* 
                 * notif_val_rcv possible values: GO_RIGHT,
                 * GO_RIGHT+GO_LEFT, GO_LEFT, SP_HOME cart_pos_calibrated
                 */
                if (notif_val_rcv == GO_RIGHT) {
                        /* 
                         * app is uninitialized state and cart is already in
                         * zero position OR app is in default state
                         * (cart position calibrated). Move cart to the right
                         * until track center reached
                         */
                        /* debouncing */
                        set_output_voltage_nobounce(IDLE_MOVEMENTS_VOLTAGE);

                        while (cart_pos[0] < TRACK_LEN_MAX_CM / 2 * 0.98f) {
                                vTaskDelay(dt_cartworker);
                        }
                        dcm_set_output_volatage(0.0f);

                        /* 
                         * change app state from UNINITIALIZED to DEFAULT right
                         * after zero position is reached
                         */
                        app_current_state = DEFAULT;
                } else if (notif_val_rcv == GO_LEFT) {
                        /* 
                         * App is in default state(cart position calibrated).
                         * Move cart to the left until center position reached.
                         */
                        /* debouncing */
                        set_output_voltage_nobounce(-IDLE_MOVEMENTS_VOLTAGE);

                        while (cart_pos[0] > TRACK_LEN_MAX_CM / 2 * 1.02f) {
                                vTaskDelay(dt_cartworker);
                        }
                        dcm_set_output_volatage(0.0f);
                } else if (notif_val_rcv == GO_LEFT + GO_RIGHT) {
                        /* 
                         * app is in uninitialized state and cart position has
                         * to be calibrated.
                         */
                        /* debouncing */
                        set_output_voltage_nobounce(-IDLE_MOVEMENTS_VOLTAGE);

                        while (!READ_ZERO_POSITION_REACHED) {
                                /* go left until zero position reached */
                                vTaskDelay(dt_cartworker);
                        }
                        app_current_state = DEFAULT;

                        /* Debouncing */
                        set_output_voltage_nobounce(IDLE_MOVEMENTS_VOLTAGE);

                        while (cart_pos[0] < TRACK_LEN_MAX_CM / 2.0f) {
                                /* go to track center */
                                vTaskDelay(dt_cartworker);
                                // vTaskDelay( 10 );
                        }
                        dcm_set_output_volatage(0.0f);
                } else if (notif_val_rcv == SP_HOME) {
                        /* 
                         * App is in UPC or DPC state. Change setpoint to home
                         * postion. Write new setpoint to *_raw cli setpoint
                         * (unfiltered). cart_pos_setp_cm_cli_raw acts
                         * as input to cart_pos_setp_cm_cli low-pass
                         * filter. Low-pass filter is used for both setpoint
                         * sources to smooth out discontinous input
                         */
                        cart_pos_setp_cm_cli_raw = TRACK_LEN_MAX_CM / 2.0f;
                }

                vTaskDelay(dt_cartworker);
        }
}

void set_output_voltage_nobounce(float voltage)
{
        /* Not elegant way of doing cart debouncing but for now it's alright */
        dcm_set_output_volatage(voltage);
        vTaskDelay(10);
        dcm_set_output_volatage(voltage);
        vTaskDelay(10);
        dcm_set_output_volatage(voltage);
        vTaskDelay(10);
        dcm_set_output_volatage(voltage);
        vTaskDelay(10);
        dcm_set_output_volatage(voltage);
        vTaskDelay(10);
        dcm_set_output_volatage(voltage);
}