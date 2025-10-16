#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/projdefs.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "tasks.h"
#include "pid_control.h"
#include "low_pass_filter.h"

extern TaskHandle_t control_motor_a_task_handle;
extern TaskHandle_t control_motor_b_task_handle;

void control_task(void* pvParameters) {
    control_task_ctx* ctx = pvParameters;

    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI("control_task", "Task notified");

        float measurement = 0.0f; // simulate sensor_read()
        float filtered_measurement = 0.0f;
        esp_err_t err = low_pass_filter_apply(ctx->filter, measurement, &filtered_measurement); // will fail handle is NULL
        if(err != ESP_OK) {
            // handle error
        }

        float output = 0.0f;
        err = pid_control_update(ctx->pid, ctx->setpoint, filtered_measurement, &output); // will fail handle is NULL
        if(err != ESP_OK) {
            // handle error
        }

        // simulate actuator_write(output);
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
