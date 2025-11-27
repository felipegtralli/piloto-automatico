#include "error_handling.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "lrgb.h"
#include "lwip/sockets.h"
#include "tasks.h"
#include <stdint.h>

#define MAX_RETRY_ATTEMPTS 3U
#define MAX_LED_BLINK 5U

#define LED_BLINK_DELAY_MS 100U

extern TaskHandle_t ctrl_task_handle;
extern TaskHandle_t udp_task_handle;
extern TaskHandle_t button_task_handle;

extern rgb_channel_config lrgb_channels;

static const char TAG[] = "error-handling";

static uint8_t udp_task_fail_count = 0;

static void handle_control_task_running(void) {
    ESP_LOGI(TAG, "Control Task is running.");

    lrgb_set_duty_update(lrgb_channels, GREEN_R, GREEN_G, GREEN_B);
}

static void handle_control_task_paused(void) {
    ESP_LOGI(TAG, "Control Task is paused.");

    lrgb_set_duty_update(lrgb_channels, YELLOW_R, YELLOW_G, YELLOW_B);
}

static void handle_control_task_fail(control_task_ctx* ctx) {
    ESP_LOGE(TAG, "CRITICAL ERROR: Control Task failure detected. Suspending button/UDP tasks. Deleting control task. Restarting System...");
    lrgb_set_duty_update(lrgb_channels, RED_R, RED_G, RED_B);

    if(ctrl_task_handle != NULL) {
        if(ctx != NULL && ctx->motor_cmpr_reg != NULL) {
            taskENTER_CRITICAL(&ctx->mutex);

            ctx->motor_cmpr_reg->cmpr_a_gen_reg->gen = 0;
            ctx->motor_cmpr_reg->cmpr_b_gen_reg->gen = 0;

            taskEXIT_CRITICAL(&ctx->mutex);
        }
        vTaskDelete(ctrl_task_handle);
    }

    if(udp_task_handle != NULL) {
        vTaskSuspend(udp_task_handle);
    }
    if(button_task_handle != NULL) {
        vTaskSuspend(button_task_handle);
    }

    for(int i = 0; i < MAX_LED_BLINK; i++) {
        lrgb_set_duty_update(lrgb_channels, RED_R, RED_G, RED_B);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DELAY_MS));
        lrgb_set_duty_update(lrgb_channels, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DELAY_MS));
    }

    esp_restart();
}

static void handle_comm_task_running(void) {
    ESP_LOGI(TAG, "Communication Task is running.");
    udp_task_fail_count = 0;
}

static void handle_comm_task_paused(void) {
    ESP_LOGI(TAG, "Communication Task is paused.");
}

static void handle_comm_task_fail(udp_comm_task_ctx* ctx, int sock) {
    ESP_LOGE(TAG, "CRITICAL ERROR: Communication Task failure detected. Trying to restart UDP task...");

    closesocket(sock);
    udp_task_fail_count++;

    if(udp_task_handle != NULL) {
        vTaskDelete(udp_task_handle);
    }

    extern StackType_t udp_task_stack[UDP_TASK_STACK_SIZE];
    extern StaticTask_t udp_task_buffer;
    udp_task_handle = xTaskCreateStatic(udp_comm_task, UDP_TASK_NAME, UDP_TASK_STACK_SIZE, ctx, UDP_TASK_PRIORITY, udp_task_stack, &udp_task_buffer);
    if(udp_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to recreate UDP task.");
    } else {
        ESP_LOGI(TAG, "UDP task recreated successfully.");
    }

    if(udp_task_fail_count >= MAX_RETRY_ATTEMPTS) {
        ESP_LOGE(TAG, "Maximum UDP task restart attempts reached. Restarting system...");

        for(int i = 0; i < MAX_LED_BLINK; i++) {
            lrgb_set_duty_update(lrgb_channels, RED_R, RED_G, RED_B);
            vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DELAY_MS));
            lrgb_set_duty_update(lrgb_channels, 0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DELAY_MS));
        }

        esp_restart();
    }
}

void handle_control_task_state(system_state state, control_task_ctx* ctx) {
    switch(state) {
        case SYSTEM_STATE_CONTROL_TASK_RUNNNING:
            handle_control_task_running();
            break;
        case SYSTEM_STATE_CONTROL_TASK_PAUSED:
            handle_control_task_paused();
            break;
        case SYSTEM_STATE_CONTROL_TASK_FAIL:
            handle_control_task_fail(ctx);
            break;
        default:
            ESP_LOGW(TAG, "Unknown control task state: %d", state);
            break;
        }
}       

void handle_comm_task_state(system_state state, udp_comm_task_ctx* ctx, int sock) {
    switch(state) {
        case SYSTEM_STATE_COMM_TASK_RUNNING:
            handle_comm_task_running();
            break;
        case SYSTEM_STATE_COMM_TASK_PAUSED:
            handle_comm_task_paused();
            break;
        case SYSTEM_STATE_COMM_TASK_FAIL:
            handle_comm_task_fail(ctx, sock);
            break;
        default:
            ESP_LOGW(TAG, "Unknown communication task state: %d", state);
            break;
    }
}
