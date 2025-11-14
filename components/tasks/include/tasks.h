#ifndef __TASKS_H__
#define __TASKS_H__

#include <stdint.h>
#include <stdbool.h>
#include "driver/pulse_cnt.h"
#include "driver/gptimer_types.h"
#include "freertos/idf_additions.h"
#include "motor.h"
#include "pid_control.h"
#include "low_pass_filter.h"
#include "wifi.h"

#define CONTROL_TASK_STACK_SIZE 2048U
#define CONTROL_TASK_PRIORITY 5U

#define CONTROL_TASK_NAME "control_motor_task"

#define UDP_TASK_STACK_SIZE (2048U * 2U)
#define UDP_TASK_PRIORITY 2U

#define UDP_TASK_NAME "udp_comm_task"

#define BUTTON_TASK_STACK_SIZE 2048U
#define BUTTON_TASK_PRIORITY 1U

#define BUTTON_TASK_NAME "button_task"

typedef struct {
    pid_control_handle pid;
    low_pass_filter_handle filter;
    motor_cmpr_reg* motor_cmpr_reg;
    pcnt_unit_handle_t pcnt_unit;
    float setpoint;
    uint16_t pwm_max_ticks;
    portMUX_TYPE mutex;
} control_task_ctx;

typedef struct {
    wifi_ap_event_handler_ctx* ehandler_ctx;
    bool first_exchange;
    control_task_ctx* ctrl_task_ctx;
} udp_comm_task_ctx;

typedef struct {
    QueueHandle_t button_queue;
} button_task_ctx;

void control_task(void* pvParameters);
bool control_task_notify_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx);

void udp_comm_task(void* pvParameters);

void button_task(void* pvParameters);

#endif // __TASKS_H__
