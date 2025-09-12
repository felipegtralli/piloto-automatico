#include "pid_control.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#if CONFIG_PID_CONTROL_LOGGING
    #include "esp_log.h"
    static const char TAG[] = "pid_control";
#endif

struct clamp_range {
    float min;
    float max;
};

struct pid_req {
    size_t size;
    size_t align;
};

struct pid_control {
    float kp;
    float ki;
    float kd;
    float kaw;
    float integral_term;
    float prev_err;
    float prev_err2;
    float last_u;
    float u_min;
    float u_max;
};

_Static_assert(PID_CONTROL_STORAGE_SIZE >= sizeof(struct pid_control), "PID_CONTROL_STORAGE_SIZE is too small");
_Static_assert(PID_CONTROL_STORAGE_ALIGNMENT >= _Alignof(struct pid_control), "PID_CONTROL_STORAGE_ALIGNMENT is too small");
_Static_assert((PID_CONTROL_STORAGE_ALIGNMENT & (PID_CONTROL_STORAGE_ALIGNMENT - 1u)) == 0u, "PID_CONTROL_STORAGE_ALIGNMENT must be power of two");

static inline bool is_finite(float x) {
    const float t = x;
    return isfinite(t);
}

// must be power of two
static inline bool is_aligned(const void* ptr, size_t align) {
    return ((uintptr_t)ptr & (uintptr_t)(align - 1u)) == (uintptr_t)0;
}

static inline float clampf(float x, struct clamp_range range) {
    const float t = (x < range.min) ? range.min : x;
    return (t > range.max) ? range.max : t;
}

static esp_err_t pid_validate_storage(const void* storage, size_t storage_size, struct pid_req req) {
    if(storage_size < req.size) {
        return ESP_ERR_INVALID_SIZE;
    }
    if(!is_aligned(storage, req.align)) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t pid_validate_config(const pid_control_config* config) {
    if(!is_finite(config->kp) || !is_finite(config->ki) || !is_finite(config->kd) || !is_finite(config->kaw) ||
       !is_finite(config->u_min) || !is_finite(config->u_max)) {
        return ESP_ERR_INVALID_ARG;
    }
    if(config->kp < 0.0f || config->ki < 0.0f || config->kd < 0.0f || config->kaw < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if(config->u_min >= config->u_max) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static void pid_apply_config(struct pid_control* ctrl, const pid_control_config* config) {
    ctrl->kp = config->kp;
    ctrl->ki = config->ki;
    ctrl->kd = config->kd;
    ctrl->kaw = config->kaw;
    ctrl->integral_term = 0.0f;
    ctrl->prev_err = 0.0f;
    ctrl->prev_err2 = 0.0f;
    const struct clamp_range range = { .min = config->u_min, .max = config->u_max };
    ctrl->last_u = clampf(0.0f, range);
    ctrl->u_min = config->u_min;
    ctrl->u_max = config->u_max;
}

size_t pid_control_storage_size(void) {
    return sizeof(struct pid_control);
}

size_t pid_control_storage_alignment(void) {
    return (size_t)_Alignof(struct pid_control);
}

esp_err_t pid_control_init(void* storage, size_t storage_size, pid_control_handle* handle, const pid_control_config* config) {
    if(!storage || !handle || !config) {
        #if CONFIG_PID_CONTROL_LOGGING
            ESP_LOGE(TAG, "Invalid argument: NULL pointer");
        #endif
        return ESP_ERR_INVALID_ARG;
    }
    *handle = NULL;

    const struct pid_req req = {
        .size = pid_control_storage_size(),
        .align = pid_control_storage_alignment()
    };
    esp_err_t err = pid_validate_storage(storage, storage_size, req);
    if(err != ESP_OK) {
        switch(err) {
            case ESP_ERR_INVALID_SIZE:
                #if CONFIG_PID_CONTROL_LOGGING
                    ESP_LOGE(TAG, "Insufficient storage size: required %zu, provided %zu", req.size, storage_size);
                #endif
                return ESP_ERR_INVALID_SIZE;
            case ESP_ERR_INVALID_ARG:
                #if CONFIG_PID_CONTROL_LOGGING
                    ESP_LOGE(TAG, "Storage pointer is not properly aligned: required alignment %zu", req.align);
                #endif
                return ESP_ERR_INVALID_ARG;
            default:
                #if CONFIG_PID_CONTROL_LOGGING
                    ESP_LOGE(TAG, "Unexpected error in storage validation: %s", esp_err_to_name(err));
                #endif
                return err;
        }
    }

    err = pid_validate_config(config);
    if(err != ESP_OK) {
        #if CONFIG_PID_CONTROL_LOGGING
            ESP_LOGE(TAG, "Invalid argument: invalid PID configuration");
        #endif
        return err;
    }

    struct pid_control* ctrl = (struct pid_control*)storage;
    pid_apply_config(ctrl, config);

    *handle = ctrl;
    return ESP_OK;
}
