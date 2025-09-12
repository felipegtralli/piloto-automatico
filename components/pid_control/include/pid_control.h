#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stddef.h>
#include "esp_err.h"

#define PID_CONTROL_STORAGE_SIZE 64u
#define PID_CONTROL_STORAGE_ALIGNMENT 4u

struct pid_control;
typedef struct pid_control* pid_control_handle;

typedef struct {
    float kp;
    float ki;
    float kd;
    float kaw; // anti-windup gain
    float u_min; // minimum output
    float u_max; // maximum output
} pid_control_config;

size_t pid_control_storage_size(void);
size_t pid_control_storage_alignment(void);
esp_err_t pid_control_init(void* storage, size_t storage_size, pid_control_handle* handle, const pid_control_config* config);

#endif // PID_CONTROL_H
