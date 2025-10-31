#include "low_pass_filter.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

#if CONFIG_LOW_PASS_FILTER_LOGGING
    #include "esp_log.h"
    static const char TAG[] = "low-pass-filter";
#endif

#define BILINEAR_FACTOR 2.0f
#define NYQUIST_FACTOR 2.0f

struct low_pass_filter {
    float alpha;
    float beta;
    float prev_output;
    float prev_read;
};

_Static_assert(LOW_PASS_FILTER_STORAGE_SIZE >= sizeof(struct low_pass_filter), "LOW_PASS_FILTER_STORAGE_SIZE is too small");
_Static_assert(LOW_PASS_FILTER_STORAGE_ALIGNMENT >= _Alignof(struct low_pass_filter), "LOW_PASS_FILTER_STORAGE_ALIGNMENT is too small");
_Static_assert((LOW_PASS_FILTER_STORAGE_ALIGNMENT & (LOW_PASS_FILTER_STORAGE_ALIGNMENT - 1U)) == 0U, "LOW_PASS_FILTER_STORAGE_ALIGNMENT must be a power of two");

// global helpers
// MISRA C:2012 R11.4 deviation – pointer to integer cast for alignment check (no dereference).
static inline bool is_aligned(const void* ptr, size_t align) {
    const uintptr_t mask = (uintptr_t)align - (uintptr_t)1u;
    return ((uintptr_t)ptr & mask) == (uintptr_t)0;
}

static inline bool is_finite(float x) {
    const float t = x;
    return (isfinite(t) != 0);
}

// init helpers
static esp_err_t filter_validate_storage(const void* storage, size_t storage_size, size_t req_size, size_t req_align) {
    if(storage_size < req_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    if(!is_aligned(storage, req_align)) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t filter_validate_config(const low_pass_filter_config* config) {
    if(!is_finite(config->cutoff_freq) || !is_finite(config->sampling_freq)) {
        return ESP_ERR_INVALID_ARG;
    }

    if(config->cutoff_freq < 0.0f || config->sampling_freq <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    bool nyquist_crit = (config->sampling_freq > NYQUIST_FACTOR * config->cutoff_freq);
    if(!nyquist_crit) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void filter_apply_config(struct low_pass_filter* filter, const low_pass_filter_config* config) {
    const float sampling_period = 1.0f / config->sampling_freq;
    const float wc = BILINEAR_FACTOR * (float)M_PI * config->cutoff_freq;

    // prewarp arg: (wc * T / 2)
    const float prewarp_arg = wc * (sampling_period / BILINEAR_FACTOR);
    const float wc_prewarp = (BILINEAR_FACTOR / sampling_period) * tanf(prewarp_arg);

    // alpha = (2 - T*wc) / (2 + T*wc)
    filter->alpha = (BILINEAR_FACTOR - (sampling_period * wc_prewarp)) / (BILINEAR_FACTOR + (sampling_period * wc_prewarp));

    // beta = (T*wc) / (2 + T*wc)
    filter->beta = (sampling_period * wc_prewarp) / (BILINEAR_FACTOR + (sampling_period * wc_prewarp));

    filter->prev_output = 0.0f;
    filter->prev_read = 0.0f;
}

// apply helpers
#if !CONFIG_LOW_PASS_FILTER_IGNORE_APPLY_CHECKS
static esp_err_t filter_check_apply_input(float input) {
    if(!is_finite(input)) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}
#endif // CONFIG_LOW_PASS_FILTER_IGNORE_APPLY_CHECKS

// y[n] = alpha * y[n-1] + beta * (x[n] + x[n-1])
static float filter_compute(const struct low_pass_filter* filter, float input) {
    const float a = filter->alpha * filter->prev_output;
    const float b = filter->beta * (input + filter->prev_read);

    const float output = a + b;
    return output;
}

static void filter_commit(struct low_pass_filter* filter, float input, float output) {
    filter->prev_read = input;
    filter->prev_output = output;
}

size_t low_pass_filter_get_storage_size(void) {
    return sizeof(struct low_pass_filter);
}

size_t low_pass_filter_get_storage_alignment(void) {
    return LOW_PASS_FILTER_STORAGE_ALIGNMENT;
}

esp_err_t low_pass_filter_init(void* storage, size_t storage_size, const low_pass_filter_config* config, low_pass_filter_handle* handle) {
    esp_err_t status = ESP_OK;
    if(storage == NULL || config == NULL || handle == NULL) {
        #if CONFIG_LOW_PASS_FILTER_LOGGING
            ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        #endif
        status = ESP_ERR_INVALID_ARG;
    } else {
        *handle = NULL;
    }

    if(status == ESP_OK) {
        const size_t req_size = low_pass_filter_get_storage_size();
        const size_t req_align = low_pass_filter_get_storage_alignment();

        status = filter_validate_storage(storage, storage_size, req_size, req_align);
        if(status != ESP_OK) {
            #if CONFIG_LOW_PASS_FILTER_LOGGING
                ESP_LOGE(TAG, "Invalid argument: invalid storage (size or alignment)");
            #endif
        }
    }

    if(status == ESP_OK) {
        status = filter_validate_config(config);
        if(status != ESP_OK) {
            if(status == ESP_ERR_INVALID_ARG) {
                #if CONFIG_LOW_PASS_FILTER_LOGGING
                    ESP_LOGE(TAG, "Invalid argument: invalid config (negative or non-finite frequency)");
                #endif
            }
            if(status == ESP_FAIL) {
                 #if CONFIG_LOW_PASS_FILTER_LOGGING
                    ESP_LOGE(TAG, "Configuration error: Nyquist criterion not met (sampling frequency must be > 2x cutoff frequency)");
                #endif
            }
        }
    }

    if(status == ESP_OK) {
        // MISRA C:2012 R11.5 deviation – converting void* storage to object pointer after validation.
        struct low_pass_filter* filter = storage;
        filter_apply_config(filter, config);

        *handle = filter;
    }

    return status;
}

/*
 In continuious time, the frequency response of a low pass filter is:
  G(s) = wc / (s + wc)
 Using Tustin's method and approximating s into G(s):
  G(z) = (T*wc * (1 + z^-1)) / ((T*wc + 2) + (T*wc - 2)*z^-1)
*/
esp_err_t low_pass_filter_apply(low_pass_filter_handle handle, float input, float* output) {
    esp_err_t status = ESP_OK;

    #if !CONFIG_LOW_PASS_FILTER_IGNORE_APPLY_CHECKS

        if(handle == NULL || output == NULL) {
            #if CONFIG_LOW_PASS_FILTER_LOGGING
                ESP_LOGE(TAG, "Invalid argument: NULL pointer");
            #endif
            status = ESP_ERR_INVALID_ARG;
        }

        if(status == ESP_OK) {
            status = filter_check_apply_input(input);
            if(status != ESP_OK) {
                #if CONFIG_LOW_PASS_FILTER_LOGGING
                    ESP_LOGE(TAG, "Invalid argument: non-finite input");
                #endif
            }
        }

    #endif // CONFIG_LOW_PASS_FILTER_LOGGING

    if(status == ESP_OK) {
        struct low_pass_filter* filter = handle;

        *output = filter_compute(filter, input);
        filter_commit(filter, input, *output);
    }

    return status;
}

esp_err_t low_pass_filter_reset_state(low_pass_filter_handle handle) {
    esp_err_t status = ESP_OK;
    if(handle == NULL) {
        #if CONFIG_LOW_PASS_FILTER_LOGGING
            ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        #endif
        status = ESP_ERR_INVALID_ARG;
    }

    if(status == ESP_OK) {
        struct low_pass_filter* filter = handle;

        filter->prev_output = 0.0f;
        filter->prev_read = 0.0f;
    }

    return status;
}

esp_err_t low_pass_filter_get_state(low_pass_filter_handle handle, float* prev_output, float* prev_read) {
    esp_err_t status = ESP_OK;
    if(handle == NULL || prev_read == NULL || prev_output == NULL) {
        #if CONFIG_LOW_PASS_FILTER_LOGGING
            ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        #endif
        status = ESP_ERR_INVALID_ARG;
    }

    if(status == ESP_OK) {
        struct low_pass_filter* filter = handle;

        *prev_output = filter->prev_output;
        *prev_read = filter->prev_read;
    }

    return status;
}
