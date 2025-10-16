#ifndef NVS_HELPER_H
#define NVS_HELPER_H

#include "esp_err.h"
#include "pid_control.h"
#include "low_pass_filter.h"

#define NVS_PID_NAMESPACE "pid_cfg_strg"
#define NVS_PID_CONFIG_KEY "pid_config"

#define NVS_FILTER_NAMESPACE "filter_cfg_strg"
#define NVS_FILTER_CONFIG_KEY "filter_config"

esp_err_t nvs_read_pid_config(const char* key, pid_control_config* config);
esp_err_t nvs_read_filter_config(const char* key, low_pass_filter_config* config);

#endif // NVS_HELPER_H