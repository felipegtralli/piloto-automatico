#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "freertos/projdefs.h"
#include "esp_attr.h"
#include "tasks.h"
#include "pid_control.h"
#include "low_pass_filter.h"
#include <stdint.h>

extern TaskHandle_t control_motor_a_task_handle;
extern TaskHandle_t control_motor_b_task_handle;

void control_task(void* pvParameters) {
    control_task_ctx* ctx = pvParameters;

    if(ctx == NULL || ctx->pid == NULL || ctx->filter == NULL || ctx->motor_cmpr_reg == NULL) {
        // error handling
        vTaskDelete(NULL);
        return;
    }

    uint16_t pwm_value = 0;
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        float measurement = 0.0f; // simulate sensor_read()
        float filtered_measurement = 0.0f;

        taskENTER_CRITICAL(&ctx->mutex);
        
        (void)low_pass_filter_apply(ctx->filter, measurement, &filtered_measurement); // ignore return for performance

        float output = 0.0f;
        (void)pid_control_update(ctx->pid, ctx->setpoint, filtered_measurement, &output); // ignore return for performance

        taskEXIT_CRITICAL(&ctx->mutex);

        // simulate actuator_write(output);
        if(++pwm_value >= ctx->pwm_max_ticks) {
            pwm_value = 0;
        }
        ctx->motor_cmpr_reg->cmpr_a_gen_reg->gen = pwm_value;
        ctx->motor_cmpr_reg->cmpr_b_gen_reg->gen = pwm_value;
    }
}

bool IRAM_ATTR control_task_notify_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(control_motor_a_task_handle, &xHigherPriorityTaskWoken);
    vTaskNotifyGiveFromISR(control_motor_b_task_handle, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
        return true;
    }
    return false;
}
