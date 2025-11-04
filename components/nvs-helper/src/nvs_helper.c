#include <stddef.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_helper.h"
#include "pid_control.h"

static const char TAG[] = "nvs-helper";

#define NVS_PID_NAMESPACE "pid_cfg_strg"
#define NVS_MOTOR_NAMESPACE "motor_cfg_strg"
#define NVS_SETPOINT_NAMESPACE "setpoint_strg"

#define DEFAULT_PID_KP 1.0f
#define DEFAULT_PID_KI 0.1f
#define DEFAULT_PID_KD 0.01f
#define DEFAULT_PID_KAW 0.0f
#define DEFAULT_PID_UMAX 100.0f
#define DEFAULT_PID_UMIN (-100.0f)

#define DEFAULT_MOTOR_GPIO ((motor_gpio_config){ .pwma_gpio = GPIO_NUM_40, .pwmb_gpio = GPIO_NUM_41 })

#define DEFAULT_SETPOINT 50.0f

static void default_pid_config(pid_control_config* config) {
    config->kp = DEFAULT_PID_KP;
    config->ki = DEFAULT_PID_KI;
    config->kd = DEFAULT_PID_KD;
    config->kaw = DEFAULT_PID_KAW;
    config->u_max = DEFAULT_PID_UMAX;
    config->u_min = DEFAULT_PID_UMIN;
}

static void default_motor_gpio(motor_gpio_config* config) {
    *config = DEFAULT_MOTOR_GPIO;
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

esp_err_t nvs_read_motor_gpio(const char* key, motor_gpio_config* config) {
    esp_err_t status = ESP_OK;
    if(config == NULL) {
        ESP_LOGI(TAG, "Invalid argument: config is NULL");
        status = ESP_ERR_INVALID_ARG;
    }
 
    nvs_handle_t nvs_handle = 0;
    if(status == ESP_OK) {
        status = nvs_open(NVS_MOTOR_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Error opening NVS namespace (%s): %s", NVS_MOTOR_NAMESPACE, esp_err_to_name(status));
        }
    }

    if(status == ESP_OK) {
        size_t required_size = sizeof(motor_gpio_config);
        status = nvs_get_blob(nvs_handle, key, config, &required_size);
        switch(status) {
            case ESP_OK:
                ESP_LOGI(TAG, "Successfully read motor GPIO config from NVS");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGW(TAG, "Motor GPIO config not found in NVS for key: %s. Using default values.", key);
                default_motor_gpio(config);
                break;
            default:
                ESP_LOGE(TAG, "Error reading motor GPIO config from NVS: %s", esp_err_to_name(status));
                break;
        }
    }

    if(nvs_handle != 0) {
        nvs_close(nvs_handle);
    }

    return status;
}

esp_err_t nvs_read_setpoint(const char* key, float* setpoint) {
    esp_err_t status = ESP_OK;
    if(setpoint == NULL) {
        ESP_LOGI(TAG, "Invalid argument: setpoint is NULL");
        status = ESP_ERR_INVALID_ARG;
    }
 
    nvs_handle_t nvs_handle = 0;
    if(status == ESP_OK) {
        status = nvs_open(NVS_SETPOINT_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Error opening NVS namespace (%s): %s", NVS_SETPOINT_NAMESPACE, esp_err_to_name(status));
        }
    }

    if(status == ESP_OK) {
        size_t required_size = sizeof(float);
        status = nvs_get_blob(nvs_handle, key, setpoint, &required_size);
        switch(status) {
            case ESP_OK:
                ESP_LOGI(TAG, "Successfully read setpoint from NVS");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGW(TAG, "Setpoint not found in NVS for key: %s. Using default value 50.0f.", key);
                *setpoint = DEFAULT_SETPOINT;
                status = ESP_OK;
                break;
            default:
                ESP_LOGE(TAG, "Error reading setpoint from NVS: %s", esp_err_to_name(status));
                break;
        }
    }

    if(nvs_handle != 0) {
        nvs_close(nvs_handle);
    }

    return status;
}

esp_err_t nvs_write_pid_config(const char* key, const pid_control_config* config) {
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
        status = nvs_set_blob(nvs_handle, key, config, sizeof(pid_control_config));
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Error writing PID config to NVS: %s", esp_err_to_name(status));
        }
    }

    if(status == ESP_OK) {
        status = nvs_commit(nvs_handle);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Error committing PID config to NVS: %s", esp_err_to_name(status));
        }
    }

    if(nvs_handle != 0) {
        nvs_close(nvs_handle);
    }

    return status;
}

esp_err_t nvs_write_motor_gpio(const char* key, const motor_gpio_config* config) {
    esp_err_t status = ESP_OK;
    if(config == NULL) {
        ESP_LOGI(TAG, "Invalid argument: config is NULL");
        status = ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle = 0;
    if(status == ESP_OK) {
        status = nvs_open(NVS_MOTOR_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Error opening NVS namespace (%s): %s", NVS_MOTOR_NAMESPACE, esp_err_to_name(status));
        }
    }

    if(status == ESP_OK) {
        status = nvs_set_blob(nvs_handle, key, config, sizeof(motor_gpio_config));
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Error writing motor GPIO config to NVS: %s", esp_err_to_name(status));
        }
    }

    if(status == ESP_OK) {
        status = nvs_commit(nvs_handle);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Error committing motor GPIO config to NVS: %s", esp_err_to_name(status));
        }
    }

    if(nvs_handle != 0) {
        nvs_close(nvs_handle);
    }

    return status;
}

esp_err_t nvs_write_setpoint(const char* key, const float* setpoint) {
    esp_err_t status = ESP_OK;
    if(setpoint == NULL) {
        ESP_LOGI(TAG, "Invalid argument: setpoint is NULL");
        status = ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle = 0;
    if(status == ESP_OK) {
        status = nvs_open(NVS_SETPOINT_NAMESPACE, NVS_READWRITE, &nvs_handle);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Error opening NVS namespace (%s): %s", NVS_SETPOINT_NAMESPACE, esp_err_to_name(status));
        }
    }

    if(status == ESP_OK) {
        status = nvs_set_blob(nvs_handle, key, setpoint, sizeof(float));
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Error writing setpoint to NVS: %s", esp_err_to_name(status));
        }
    }

    if(status == ESP_OK) {
        status = nvs_commit(nvs_handle);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Error committing setpoint to NVS: %s", esp_err_to_name(status));
        }
    }

    if(nvs_handle != 0) {
        nvs_close(nvs_handle);
    }

    return status;
}
