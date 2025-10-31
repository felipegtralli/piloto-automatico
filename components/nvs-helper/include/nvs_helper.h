#ifndef NVS_HELPER_H
#define NVS_HELPER_H

#include "esp_err.h"
#include "motor.h"
#include "pid_control.h"

#define NVS_PID_CONFIG_KEY "pid_config"

#define NVS_MOTOR_A_GPIO_KEY "motor_a_gpio_config"
#define NVS_MOTOR_B_GPIO_KEY "motor_b_gpio_config"

#define NVS_SETPOINT_KEY "setpoint"

esp_err_t nvs_read_pid_config(const char* key, pid_control_config* config);
esp_err_t nvs_read_motor_gpio(const char* key, motor_gpio_config* config);
esp_err_t nvs_read_setpoint(const char* key, float* setpoint);

esp_err_t nvs_write_pid_config(const char* key, const pid_control_config* config);
esp_err_t nvs_write_motor_gpio(const char* key, const motor_gpio_config* config);
esp_err_t nvs_write_setpoint(const char* key, const float* setpoint);

#endif // NVS_HELPER_H