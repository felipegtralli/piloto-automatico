#ifndef TASKS_H
#define TASKS_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "driver/gptimer_types.h"
#include "motor.h"
#include "pid_control.h"
#include "low_pass_filter.h"

#define CONTROL_TASK_STACK_SIZE 2048
#define CONTROL_TASK_PRIORITY 5

#define CONTROL_TASK_A_NAME "control_motor_a_task"
#define CONTROL_TASK_B_NAME "control_motor_b_task"

#define UDP_TASK_STACK_SIZE 2048 * 2
#define UDP_TASK_PRIORITY 1

#define UDP_TASK_NAME "udp_comm_task"

typedef struct {
    pid_control_handle pid;
    low_pass_filter_handle filter;
    motor_cmpr_reg* motor_cmpr_reg;
    float setpoint;
    uint16_t pwm_max_ticks;
    portMUX_TYPE mutex;
} control_task_ctx;

typedef struct {
    bool first_exchange;
    control_task_ctx* control_ctx_motor_a;
    control_task_ctx* control_ctx_motor_b;
} udp_comm_task_ctx;

void control_task(void* pvParameters);
bool control_task_notify_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx);

void udp_comm_task(void* pvParameters);

#endif // TASKS_H
