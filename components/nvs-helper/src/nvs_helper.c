#include <stddef.h>
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_helper.h"
#include "pid_control.h"
#include "low_pass_filter.h"

static const char TAG[] = "nvs-helper";

#define DEFAULT_PID_KP 1.0f
#define DEFAULT_PID_KI 0.1f
#define DEFAULT_PID_KD 0.01f
#define DEFAULT_PID_KAW 0.0f
#define DEFAULT_PID_UMAX 100.0f
#define DEFAULT_PID_UMIN (-100.0f)

#define DEFAULT_FILTER_CUTOFF_FREQ 10.0f
#define DEFAULT_FILTER_SAMPLING_FREQ_HZ 100.0f

static void default_pid_config(pid_control_config* config) {
    config->kp = DEFAULT_PID_KP;
    config->ki = DEFAULT_PID_KI;
    config->kd = DEFAULT_PID_KD;
    config->kaw = DEFAULT_PID_KAW;
    config->u_max = DEFAULT_PID_UMAX;
    config->u_min = DEFAULT_PID_UMIN;
}

static void default_filter_config(low_pass_filter_config* config) {
    config->cutoff_freq = DEFAULT_FILTER_CUTOFF_FREQ;
    config->sampling_freq = DEFAULT_FILTER_SAMPLING_FREQ_HZ;
}

esp_err_t nvs_read_pid_config(const char* key, pid_control_config* config) {
    esp_err_t status = ESP_OK;
    if(config == NULL) {
        ESP_LOGI(TAG, "Invalid argument: config is NULL");
        status = ESP_ERR_INVALID_ARG;
    }
 
    nvs_handle_t nvs_handle = 0;
    if(status == ESP_OK) {
        status = nvs_open(NVS_PID_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Error opening NVS namespace (%s): %s", NVS_PID_NAMESPACE, esp_err_to_name(status));
        }
    }

    if(status == ESP_OK) {
        size_t required_size = sizeof(pid_control_config);
        status = nvs_get_blob(nvs_handle, key, config, &required_size);
        switch(status) {
            case ESP_OK:
                ESP_LOGI(TAG, "Successfully read PID config from NVS");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGW(TAG, "PID config not found in NVS for key: %s. Using default values.", key);
                default_pid_config(config);
                break;
            default:
                ESP_LOGE(TAG, "Error reading PID config from NVS: %s", esp_err_to_name(status));
                break;
        }
    }

    if(nvs_handle != 0) {
        nvs_close(nvs_handle);
    }

    return status;
}

esp_err_t nvs_read_filter_config(const char* key, low_pass_filter_config* config) {
    esp_err_t status = ESP_OK;
    if(config == NULL) {
        ESP_LOGI(TAG, "Invalid argument: config is NULL");
        status = ESP_ERR_INVALID_ARG;
    }
 
    nvs_handle_t nvs_handle = 0;
    if(status == ESP_OK) {
        status = nvs_open(NVS_FILTER_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Error opening NVS namespace (%s): %s", NVS_FILTER_NAMESPACE, esp_err_to_name(status));
        }
    }

    if(status == ESP_OK) {
        size_t required_size = sizeof(low_pass_filter_config);
        status = nvs_get_blob(nvs_handle, key, config, &required_size);
        switch(status) {
            case ESP_OK:
                ESP_LOGI(TAG, "Successfully read filter config from NVS");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGW(TAG, "Filter config not found in NVS for key: %s. Using default values.", key);
                default_filter_config(config);
                break;
            default:
                ESP_LOGE(TAG, "Error reading filter config from NVS: %s", esp_err_to_name(status));
                break;
        }
    }

    if(nvs_handle != 0) {
        nvs_close(nvs_handle);
    }

    return status;
}
