#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "tasks.h"
#include "button.h"
#include <stdint.h>

void button_task(void* pvParameters) {
    button_task_ctx* ctx = pvParameters;

    for(;;) {
        button_event event = {0};

        if(xQueueReceive(ctx->button_queue, &event, portMAX_DELAY)) {
            button_handle handle = event.handle;

            esp_err_t status = ESP_OK;
            if(handle == NULL) {
                status = ESP_ERR_INVALID_ARG;
            }

            if(status == ESP_OK) {
                uint32_t debounce_ticks = 0;
                uint32_t last_interrupt_time_ticks = 0;

                status = button_get_debounce(handle, &debounce_ticks, &last_interrupt_time_ticks);
                if(status == ESP_OK) {
                    if((event.event_time_ticks - last_interrupt_time_ticks) < debounce_ticks) {
                        status = ESP_ERR_INVALID_STATE; // debounce: ignore event
                    } else {
                        (void)button_update_last_interrupt_time(event.handle, event.event_time_ticks);
                    }
                }
            }

            if(status == ESP_OK) {
                button_callback callback = NULL;
                void* callback_ctx = NULL;

                status = button_get_callback(handle, &callback, &callback_ctx);
                if(status == ESP_OK && callback != NULL) {
                    callback(handle, callback_ctx);
                }
            }
        }
    }
}
