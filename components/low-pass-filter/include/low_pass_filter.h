#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#define LOW_PASS_FILTER_STORAGE_SIZE 24u
#define LOW_PASS_FILTER_STORAGE_ALIGNMENT 4u

#include <stddef.h>
#include "esp_err.h"

struct low_pass_filter;
typedef struct low_pass_filter* low_pass_filter_handle;

typedef struct {
    float cutoff_freq;
    float sampling_freq;
} low_pass_filter_config;

size_t low_pass_filter_get_storage_size(void);
size_t low_pass_filter_get_storage_alignment(void);

esp_err_t low_pass_filter_init(void* storage, size_t storage_size, const low_pass_filter_config* config, low_pass_filter_handle* handle);
esp_err_t low_pass_filter_apply(low_pass_filter_handle handle, float input, float* output);

esp_err_t low_pass_filter_reset_state(low_pass_filter_handle handle);
esp_err_t low_pass_filter_get_state(low_pass_filter_handle handle, float* prev_output, float* prev_read);

#endif // LOW_PASS_FILTER_H