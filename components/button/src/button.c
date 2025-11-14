#include "button.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include <stdbool.h>
#include <stdint.h>

static const char TAG[] = "button";

static bool isr_installed = false;

struct button {
    gpio_num_t gpio_num;
    uint32_t last_interrupt_time_ticks;
    TickType_t debounce_ticks;
    QueueHandle_t queue;
    button_callback callback;
    void* callback_ctx;
};

static void IRAM_ATTR button_isr_handler(void* arg) {
    struct button* btn = arg;
    if(btn == NULL) {
        return;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;

    button_event event = {
        .handle = btn,
        .event_time_ticks = xTaskGetTickCountFromISR(),
    };

    (void)xQueueSendFromISR(btn->queue, &event, &higher_priority_task_woken);
}

// MISRA C:2012 R11.4 deviation – pointer to integer cast for alignment check (no dereference).
static inline bool is_aligned(const void* ptr, size_t align) {
    const uintptr_t mask = (uintptr_t)align - (uintptr_t)1u;
    return ((uintptr_t)ptr & mask) == (uintptr_t)0;
}

static esp_err_t button_validate_storage(const void* storage, size_t storage_size, size_t req_size, size_t req_align) {
    if(storage_size < req_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    if(!is_aligned(storage, req_align)) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t button_validate_config(const button_config* config) {
    if(config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if((config->gpio_num < 0) || (config->gpio_num >= GPIO_NUM_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }

    if(config->queue == NULL && config->callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t button_config_gpio(const button_config* config) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    return gpio_config(&io_conf);
}

static void button_apply_config(struct button* button, const button_config* config) {
    button->gpio_num = config->gpio_num;
    button->debounce_ticks = pdMS_TO_TICKS(config->debounce_ms);
    button->queue = config->queue;
    button->callback = config->callback;
    button->callback_ctx = config->callback_ctx;
}

static esp_err_t button_setup_isr(struct button* button) {
    esp_err_t status = ESP_OK;
    if(!isr_installed) {
        status = gpio_install_isr_service(0);
        if(status == ESP_OK || status == ESP_ERR_INVALID_STATE) {
            isr_installed = true;
            status = ESP_OK;
        }
    }

    if(status == ESP_OK) {
        status = gpio_isr_handler_add(button->gpio_num, button_isr_handler, (void*)button);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add ISR handler for GPIO %d", button->gpio_num);
        }
    }

    return status;
}

esp_err_t button_init(void* storage, size_t storage_size, const button_config* config, button_handle* handle) {
    esp_err_t status = ESP_OK;
    if(storage == NULL || config == NULL || handle == NULL) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(TAG, "Invalid argument");
    } else {
        *handle = NULL;
    }   

    if(status == ESP_OK) {
        status = button_validate_storage(storage, storage_size, BUTTON_STORAGE_SIZE, BUTTON_STORAGE_ALIGNMENT);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Invalid storage (size or alignment)");
        }
    }

    if(status == ESP_OK) {
        status = button_validate_config(config);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Invalid button configuration");
        }
    }

    if(status == ESP_OK) {
        status = button_config_gpio(config);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure GPIO");
        }
    }

    struct button* button = NULL;
    if(status == ESP_OK) {
        // MISRA C:2012 R11.5 deviation – converting void* storage to object pointer after validation.
        button = storage;

        button_apply_config(button, config);
    }

    if(status == ESP_OK) {
        status = button_setup_isr(button);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to setup ISR");
        }
    }

    if(status == ESP_OK) {
        *handle = button;
        ESP_LOGI(TAG, "Button initialized on GPIO %d", button->gpio_num);
    } else {
        (void)gpio_isr_handler_remove(button->gpio_num);
        *handle = NULL;
    }

    return status;
}

esp_err_t button_get_debounce(button_handle handle, uint32_t* debounce_ticks, uint32_t* last_interrupt_time_ticks) {
    esp_err_t status = ESP_OK;
    if(handle == NULL || debounce_ticks == NULL || last_interrupt_time_ticks == NULL) {
        ESP_LOGE(TAG, "Invalid argument");
        status = ESP_ERR_INVALID_ARG;
    }

    if(status == ESP_OK) {
        struct button* button = handle;
        *debounce_ticks = button->debounce_ticks;
        *last_interrupt_time_ticks = button->last_interrupt_time_ticks;
    }

    return status;
}

esp_err_t button_get_callback(button_handle handle, button_callback* callback, void** callback_ctx) {
    esp_err_t status = ESP_OK;
    if(handle == NULL || callback == NULL || callback_ctx == NULL) {
        ESP_LOGE(TAG, "Invalid argument");
        status = ESP_ERR_INVALID_ARG;
    }

    if(status == ESP_OK) {
        struct button* button = handle;
        *callback = button->callback;
        *callback_ctx = button->callback_ctx;
    }

    return status;
}

esp_err_t button_update_last_interrupt_time(button_handle handle, uint32_t interrupt_time_ticks) {
    esp_err_t status = ESP_OK;
    if(handle == NULL) {
        ESP_LOGE(TAG, "Invalid argument");
        status = ESP_ERR_INVALID_ARG;
    }

    if(status == ESP_OK) {
        struct button* button = handle;
        button->last_interrupt_time_ticks = interrupt_time_ticks;
    }

    return status;
}
