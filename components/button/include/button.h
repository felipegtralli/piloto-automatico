#ifndef __BUTTON_H__
#define __BUTTON_H__

#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "soc/gpio_num.h"
#include <stdint.h>

#define BUTTON_STORAGE_SIZE 24U
#define BUTTON_STORAGE_ALIGNMENT 4U

struct button;
typedef struct button* button_handle;

typedef void (*button_callback)(button_handle, void* ctx);

typedef struct {
    button_handle handle;
    uint32_t event_time_ticks;
} button_event;

typedef struct {
    gpio_num_t gpio_num;
    uint32_t debounce_ms;
    QueueHandle_t queue;
    button_callback callback;
    void* callback_ctx;
} button_config;

esp_err_t button_init(void* storage, size_t storage_size, const button_config* config, button_handle* handle);
esp_err_t button_get_debounce(button_handle handle, uint32_t* debounce_ticks, uint32_t* last_interrupt_time_ticks);
esp_err_t button_get_callback(button_handle handle, button_callback* callback, void** callback_ctx);
esp_err_t button_update_last_interrupt_time(button_handle handle, uint32_t interrupt_time_ticks);

#endif // __BUTTON_H__
