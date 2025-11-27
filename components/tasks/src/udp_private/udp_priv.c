#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/idf_additions.h"
#include "esp_err.h"
#include "udp_priv.h"
#include "tasks.h"
#include "low_pass_filter.h"
#include "pid_control.h"
#include "nvs_helper.h"

#define CMD_LEN 1U

#define GET_CMD_OFFSET 1U
#define SET_CMD_OFFSET 2U

#define SET_SETPOINT_RX_BUF_LEN 6U
#define SET_PID_GAINS_RX_BUF_LEN 14U
#define SET_PID_AW_RX_BUF_LEN 6U
#define SET_PID_OUTPUT_LIMITS_RX_BUF_LEN 10U

/*
 tx ack format:
    [command (1 byte)] [ack (1 byte)]
*/
static void build_ack(uint8_t* tx_buffer, size_t* tx_len, udp_command cmd, bool success) {
    tx_buffer[0] = (uint8_t)cmd;
    tx_buffer[1] = success ? 0xFF : 0x00; // ack
    *tx_len = 2U;
}

/*
 tx get rpm format:
    [command (1 byte)] [rpm (4 bytes float)]
*/
static void handle_get_rpm(uint8_t* tx_buffer, size_t* tx_len) {
    tx_buffer[0] = CMD_GET_RPM;

    extern float filtered_pulse_count;
    memcpy(&tx_buffer[GET_CMD_OFFSET], &filtered_pulse_count, sizeof(float));
    *tx_len = CMD_LEN + sizeof(float);    
}

/*
 tx get setpoint format:
    [command (1 byte)] [setpoint (4 bytes float)]
*/
static void handle_get_setpoint(uint8_t* tx_buffer, size_t* tx_len, float setpoint) {
    tx_buffer[0] = CMD_GET_SETPOINT;

    memcpy(&tx_buffer[GET_CMD_OFFSET], &setpoint, sizeof(float));
    *tx_len = CMD_LEN + sizeof(float);
}

/*
 tx get pid gains format:
    [command (1 byte)] [kp (4 bytes float)] [ki (4 bytes float)] [kd (4 bytes float)]
*/
static void handle_get_pid_gains(uint8_t* tx_buffer, size_t* tx_len, float kp, float ki, float kd) {
    tx_buffer[0] = CMD_GET_PID_GAINS;

    memcpy(&tx_buffer[GET_CMD_OFFSET], &kp, sizeof(float));
    memcpy(&tx_buffer[GET_CMD_OFFSET + sizeof(float)], &ki, sizeof(float));
    memcpy(&tx_buffer[GET_CMD_OFFSET + 2U * sizeof(float)], &kd, sizeof(float));
    *tx_len = CMD_LEN + 3U * sizeof(float);
}

/*
 tx get pid anti-windup format:
    [command (1 byte)] [kaw (4 bytes float)]
*/
static void handle_get_pid_anti_windup(uint8_t* tx_buffer, size_t* tx_len, float kaw) {
    tx_buffer[0] = CMD_GET_PID_ANTI_WINDUP;

    memcpy(&tx_buffer[GET_CMD_OFFSET], &kaw, sizeof(float));
    *tx_len = CMD_LEN + sizeof(float);
}

/*
 tx get pid output limits format:
    [command (1 byte)] [u_min (4 bytes float)] [u_max (4 bytes float)]
*/
static void handle_get_pid_output_limits(uint8_t* tx_buffer, size_t* tx_len, float u_min, float u_max) {
    tx_buffer[0] = CMD_GET_PID_OUTPUT_LIMITS;

    memcpy(&tx_buffer[GET_CMD_OFFSET], &u_min, sizeof(float));
    memcpy(&tx_buffer[GET_CMD_OFFSET + sizeof(float)], &u_max, sizeof(float));
    *tx_len = CMD_LEN + 2U * sizeof(float);
}

/*
 tx get lpf state format:
    [command (1 byte)] [prev_output (4 bytes float)] [prev_read (4 bytes float)]
*/
static void handle_get_lpf_state(uint8_t* tx_buffer, size_t* tx_len, float prev_output, float prev_read) {
    tx_buffer[0] = CMD_GET_LPF_STATE;

    memcpy(&tx_buffer[GET_CMD_OFFSET], &prev_output, sizeof(float));
    memcpy(&tx_buffer[GET_CMD_OFFSET + sizeof(float)], &prev_read, sizeof(float));
    *tx_len = CMD_LEN + 2U * sizeof(float);
}

/*
 rx set setpoint format:
    [command (1 byte)] [flags (1 byte)] [setpoint (4 bytes float)]

 tx set setpoint format:
    [command (1 byte)] [ack (1 byte)]
*/
static void handle_set_setpoint(const uint8_t* rx_buffer, size_t rx_len, uint8_t* tx_buffer, size_t* tx_len, control_task_ctx* ctrl_ctx) {
    esp_err_t status = ESP_OK;
    if(rx_len < SET_SETPOINT_RX_BUF_LEN) {
        status = ESP_ERR_INVALID_SIZE;
    }

    float new_setpoint = 0.0f;
    if(status == ESP_OK) {
        memcpy(&new_setpoint, &rx_buffer[SET_CMD_OFFSET], sizeof(float));
        
        taskENTER_CRITICAL(&ctrl_ctx->mutex);
        ctrl_ctx->setpoint = new_setpoint;
        taskEXIT_CRITICAL(&ctrl_ctx->mutex);
    }

    if(status == ESP_OK) {
        status = nvs_write_setpoint(NVS_SETPOINT_KEY, &new_setpoint);
    }

    bool ack_success = (status == ESP_OK);
    build_ack(tx_buffer, tx_len, CMD_SET_SETPOINT, ack_success);
}

/*
 rx set pid gains format:
    [command (1 byte)] [flags (1 byte)] [kp (4 bytes float)] [ki (4 bytes float)] [kd (4 bytes float)]

 tx set pid gains format:
    [command (1 byte)] [ack (1 byte)]
*/
static void handle_set_pid_gains(const uint8_t* rx_buffer, size_t rx_len, uint8_t* tx_buffer, size_t* tx_len, control_task_ctx* ctrl_ctx) {
    esp_err_t status = ESP_OK;
    if(rx_len < SET_PID_GAINS_RX_BUF_LEN) {
        status = ESP_ERR_INVALID_SIZE;
    }

    float new_kp = 0.0f;
    float new_ki = 0.0f;
    float new_kd = 0.0f;
    if(status == ESP_OK) {
        memcpy(&new_kp, &rx_buffer[SET_CMD_OFFSET], sizeof(float));
        memcpy(&new_ki, &rx_buffer[SET_CMD_OFFSET + sizeof(float)], sizeof(float));
        memcpy(&new_kd, &rx_buffer[SET_CMD_OFFSET + 2U * sizeof(float)], sizeof(float));

        taskENTER_CRITICAL(&ctrl_ctx->mutex);
        status = pid_control_set_gains(ctrl_ctx->pid, true, new_kp, new_ki, new_kd);
        taskEXIT_CRITICAL(&ctrl_ctx->mutex);
    }

    if(status == ESP_OK) {
        pid_control_config config = {0};
        status = pid_control_get_config(ctrl_ctx->pid, &config);

        if(status == ESP_OK) {
            config.kp = new_kp;
            config.ki = new_ki;
            config.kd = new_kd;

            status = nvs_write_pid_config(NVS_PID_CONFIG_KEY, &config);
        }
    }

    bool ack_success = (status == ESP_OK);
    build_ack(tx_buffer, tx_len, CMD_SET_PID_GAINS, ack_success);
}

/*
 rx set pid anti-windup format:
    [command (1 byte)] [flags (1 byte)] [kaw (4 bytes float)]

 tx set pid anti-windup format:
    [command (1 byte)] [ack (1 byte)]
*/
static void handle_set_pid_aw(const uint8_t* rx_buffer, size_t rx_len, uint8_t* tx_buffer, size_t* tx_len, control_task_ctx* ctrl_ctx) {
    esp_err_t status = ESP_OK;
    if(rx_len < SET_PID_AW_RX_BUF_LEN) {
        status = ESP_ERR_INVALID_SIZE;
    }

    float new_kaw = 0.0f;
    if(status == ESP_OK) {
        memcpy(&new_kaw, &rx_buffer[SET_CMD_OFFSET], sizeof(float));

        taskENTER_CRITICAL(&ctrl_ctx->mutex);
        status = pid_control_set_anti_windup(ctrl_ctx->pid, new_kaw);
        taskEXIT_CRITICAL(&ctrl_ctx->mutex);
    }

    if(status == ESP_OK) {
        pid_control_config config = {0};
        status = pid_control_get_config(ctrl_ctx->pid, &config);

        if(status == ESP_OK) {
            config.kaw = new_kaw;

            status = nvs_write_pid_config(NVS_PID_CONFIG_KEY, &config);
        }
    }

    bool ack_success = (status == ESP_OK);
    build_ack(tx_buffer, tx_len, CMD_SET_PID_ANTI_WINDUP, ack_success);
}

/*
 rx set pid output limits format:
    [command (1 byte)] [flags (1 byte)] [u_min (4 bytes float)] [u_max (4 bytes float)]

 tx set pid output limits format:
    [command (1 byte)] [ack (1 byte)]
*/
static void handle_set_pid_output_limits(const uint8_t* rx_buffer, size_t rx_len, uint8_t* tx_buffer, size_t* tx_len, control_task_ctx* ctrl_ctx) {
    esp_err_t status = ESP_OK;
    if(rx_len < SET_PID_OUTPUT_LIMITS_RX_BUF_LEN) {
        status = ESP_ERR_INVALID_SIZE;
    }

    float new_u_min = 0.0f;
    float new_u_max = 0.0f;
    if(status == ESP_OK) {
        memcpy(&new_u_min, &rx_buffer[SET_CMD_OFFSET], sizeof(float));
        memcpy(&new_u_max, &rx_buffer[SET_CMD_OFFSET + sizeof(float)], sizeof(float));

        taskENTER_CRITICAL(&ctrl_ctx->mutex);
        status = pid_control_set_output_limits(ctrl_ctx->pid, new_u_min, new_u_max);
        taskEXIT_CRITICAL(&ctrl_ctx->mutex);
    }

    if(status == ESP_OK) {
        pid_control_config config = {0};
        status = pid_control_get_config(ctrl_ctx->pid, &config);

        if(status == ESP_OK) {
            config.u_min = new_u_min;
            config.u_max = new_u_max;

            status = nvs_write_pid_config(NVS_PID_CONFIG_KEY, &config);
        }
    }

    bool ack_success = (status == ESP_OK);
    build_ack(tx_buffer, tx_len, CMD_SET_PID_OUTPUT_LIMITS, ack_success);
}

size_t udp_handle_command(const uint8_t* rx_buffer, size_t len, uint8_t* tx_buffer, size_t tx_max_len, udp_comm_task_ctx* udp_ctx) {
    udp_command cmd = (udp_command)rx_buffer[0];

    size_t tx_len = 0;
    switch(cmd) {
        case CMD_GET_RPM:
            handle_get_rpm(tx_buffer, &tx_len);
            break;
        
        case CMD_GET_SETPOINT:
            handle_get_setpoint(tx_buffer, &tx_len, udp_ctx->ctrl_task_ctx->setpoint);
            break;
        
        case CMD_GET_PID_GAINS: {
            float kp = 0.0f;
            float ki = 0.0f;
            float kd = 0.0f;

            if(pid_control_get_gains(udp_ctx->ctrl_task_ctx->pid, &kp, &ki, &kd) == ESP_OK) {
                handle_get_pid_gains(tx_buffer, &tx_len, kp, ki, kd);
            }
            break;
        }

        case CMD_GET_PID_ANTI_WINDUP: {
            float kaw = 0.0f;

            if(pid_control_get_anti_windup(udp_ctx->ctrl_task_ctx->pid, &kaw) == ESP_OK) {
                handle_get_pid_anti_windup(tx_buffer, &tx_len, kaw);
            }
            break;
        }

        case CMD_GET_PID_OUTPUT_LIMITS: {
            float u_min = 0.0f;
            float u_max = 0.0f;

            if(pid_control_get_output_limits(udp_ctx->ctrl_task_ctx->pid, &u_min, &u_max) == ESP_OK) {
                handle_get_pid_output_limits(tx_buffer, &tx_len, u_min, u_max);
            }
            break;
        }

        case CMD_GET_LPF_STATE: {
            float prev_output = 0.0f;
            float prev_read = 0.0f;

            if(low_pass_filter_get_state(udp_ctx->ctrl_task_ctx->filter, &prev_output, &prev_read) == ESP_OK) {
                handle_get_lpf_state(tx_buffer, &tx_len, prev_output, prev_read);
            }
            break;
        }

        case CMD_SET_SETPOINT:
            handle_set_setpoint(rx_buffer, len, tx_buffer, &tx_len, udp_ctx->ctrl_task_ctx);
            break;
        
        case CMD_SET_PID_GAINS:
            handle_set_pid_gains(rx_buffer, len, tx_buffer, &tx_len, udp_ctx->ctrl_task_ctx);
            break;

        case CMD_SET_PID_ANTI_WINDUP:
            handle_set_pid_aw(rx_buffer, len, tx_buffer, &tx_len, udp_ctx->ctrl_task_ctx);
            break;
            
        case CMD_SET_PID_OUTPUT_LIMITS:
            handle_set_pid_output_limits(rx_buffer, len, tx_buffer, &tx_len, udp_ctx->ctrl_task_ctx);
            break;

        default:
            build_ack(tx_buffer, &tx_len, CMD_UNKNOWN, false);
            break;
    }

    return tx_len;
}
