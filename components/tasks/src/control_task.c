#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "freertos/projdefs.h"
#include "driver/pulse_cnt.h"
#include "esp_attr.h"
#include "tasks.h"
#include "pid_control.h"
#include "low_pass_filter.h"

extern TaskHandle_t ctrl_task_handle;

void control_task(void* pvParameters) {
    control_task_ctx* ctx = pvParameters;

    if(ctx == NULL || ctx->pid == NULL || ctx->filter == NULL || ctx->motor_cmpr_reg == NULL || ctx->pcnt_unit == NULL) {
        // todo: error handling
        vTaskDelete(NULL);
        return;
    }
    
    ESP_ERROR_CHECK(pcnt_unit_start(ctx->pcnt_unit));
    int pulse_count = 0;
    float filtered_pulse_count = 0.0f;
    float output = 0.0f;
    uint16_t pwm_value = 0;
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        (void)pcnt_unit_get_count(ctx->pcnt_unit, &pulse_count);
        (void)pcnt_unit_clear_count(ctx->pcnt_unit);

        taskENTER_CRITICAL(&ctx->mutex);

        (void)low_pass_filter_apply(ctx->filter, (float)pulse_count, &filtered_pulse_count);

        (void)pid_control_update(ctx->pid, ctx->setpoint, filtered_pulse_count, &output);

        taskEXIT_CRITICAL(&ctx->mutex);

        // todo: map output to pwm value
        pwm_value = ctx->pwm_max_ticks;
        ctx->motor_cmpr_reg->cmpr_a_gen_reg->gen = pwm_value;
        ctx->motor_cmpr_reg->cmpr_b_gen_reg->gen = 0;
    }
}

bool IRAM_ATTR control_task_notify_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(ctrl_task_handle, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
        return true;
    }
    return false;
}
