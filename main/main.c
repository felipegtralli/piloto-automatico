/* includes */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "driver/pulse_cnt.h"
#include "driver/gptimer.h"
#include "driver/gptimer_types.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "app_main.h"
#include "low_pass_filter.h"
#include "pid_control.h"
#include "tasks.h"
#include "nvs_helper.h"
#include "motor.h"
#include "wifi.h"
#include "button.h"

/* types */
struct button_start_stop_ctx {
    gptimer_handle_t timer;
    motor_cmpr_reg* motor_reg;
    portMUX_TYPE* ctrl_mutex;
    bool timer_started;
};

/* defines */
#define TIMER_RESOLUTION_HZ (uint32_t)CONFIG_TIMER_RESOLUTION_HZ
#define SAMPLING_RATE_HZ (uint32_t)CONFIG_SAMPLING_RATE_HZ
#define TIMER_ALARM_COUNT (uint64_t)(TIMER_RESOLUTION_HZ / SAMPLING_RATE_HZ)

#define FILTER_CUTOFF_FREQ_HZ (strtof(CONFIG_FILTER_CUTOFF_FREQ_HZ, NULL))
#define FILTER_SAMPLING_FREQ_HZ (float)SAMPLING_RATE_HZ

#define PWM_GROUP_ID CONFIG_PWM_GROUP_ID
#define PWM_RESOLUTION_HZ (uint32_t)CONFIG_PWM_TIMER_RESOLUTION_HZ
#define PWM_FREQUENCY_HZ (uint32_t)CONFIG_PWM_FREQUENCY_HZ

#define MOTOR_GPIO ((motor_gpio_config){ .pwma_gpio = CONFIG_MOTOR_GPIO_A, .pwmb_gpio = CONFIG_MOTOR_GPIO_B })

#define ENCODER_GPIO_A CONFIG_ENCODER_GPIO_A
#define ENCODER_GPIO_B CONFIG_ENCODER_GPIO_B

#define ENCODER_HIGH_LIMIT 1000
#define ENCODER_LOW_LIMIT (-1000)

#define BUTTON_DEBOUNCE_MS (uint32_t)CONFIG_BUTTON_DEBOUNCE_MS

#define BUTTON_START_STOP_GPIO CONFIG_BUTTON_START_STOP_GPIO

#define BUTTON_QUEUE_LENGHT 10
#define BUTTON_QUEUE_ITEM_SIZE sizeof(button_event)

/* variables */
TaskHandle_t ctrl_task_handle = NULL;
TaskHandle_t udp_task_handle = NULL;
TaskHandle_t button_task_handle = NULL;

QueueHandle_t button_queue = NULL;

/* function prototypes */
static void nvs_init(void);
static void pid_init(pid_control_handle* handle, uint8_t* storage);
static void lpf_init(low_pass_filter_handle* handle, uint8_t* storage);
static void pwm_init_start(motor_handle* motor, uint8_t* storage);
static void pcnt_init(pcnt_unit_handle_t* pcnt_unit);
static void ctrl_ctx_init(pid_control_handle pid, low_pass_filter_handle filter, motor_cmpr_reg* cmpr_reg, pcnt_unit_handle_t pcnt_unit, float setpoint, control_task_ctx* ctx);
static void wifi_ap_init(wifi_ap_event_handler_ctx* event_handler_ctx);
static void udp_ctx_init(udp_comm_task_ctx* ctx, wifi_ap_event_handler_ctx* ehandler_ctx, control_task_ctx* ctrl_ctx);
static void gptimer_init(gptimer_handle_t* timer);
static void button_start_stop_callback(button_handle handle, void* ctx);
static void btn_init(button_handle* handle, void* callback_ctx, uint8_t* storage);

void app_main(void) {
    // nvs setup
    nvs_init();

    // pid controller setup
    _Alignas(PID_CONTROL_STORAGE_ALIGNMENT) static uint8_t pid_storage[PID_CONTROL_STORAGE_SIZE] = {0};

    pid_control_handle pid = NULL;
    pid_init(&pid, pid_storage);

    // low pass filter setup
    _Alignas(LOW_PASS_FILTER_STORAGE_ALIGNMENT) static uint8_t filter_storage[LOW_PASS_FILTER_STORAGE_SIZE] = {0};

    low_pass_filter_handle filter = NULL;
    lpf_init(&filter, filter_storage);

    // pwm setup
    _Alignas(MOTOR_STORAGE_ALIGNMENT) static uint8_t motor_storage[MOTOR_STORAGE_SIZE] = {0};

    motor_handle motor = NULL;
    pwm_init_start(&motor, motor_storage);

    static motor_cmpr_reg motor_reg = {0};
    ESP_ERROR_CHECK(motor_get_cmpr_reg(motor, &motor_reg));

    // pcnt setup
    pcnt_unit_handle_t pcnt_unit = NULL;
    pcnt_init(&pcnt_unit);

    // control task setup
    static float setpoint = 0.0f;
    nvs_read_setpoint(NVS_SETPOINT_KEY, &setpoint);

    static control_task_ctx ctrl_task_ctx = {0};
    ctrl_ctx_init(pid, filter, &motor_reg, pcnt_unit, setpoint, &ctrl_task_ctx);

    static StackType_t ctrl_task_stack[CONTROL_TASK_STACK_SIZE];
    static StaticTask_t ctrl_task_buffer;
    ctrl_task_handle = xTaskCreateStatic(control_task, CONTROL_TASK_NAME, CONTROL_TASK_STACK_SIZE, &ctrl_task_ctx, CONTROL_TASK_PRIORITY, ctrl_task_stack, &ctrl_task_buffer);

    // wifi setup
    static wifi_ap_event_handler_ctx wifi_ap_ehandler_ctx = {
        .station_connected = false,
    };
    wifi_ap_init(&wifi_ap_ehandler_ctx);

    // udp task setup
    static udp_comm_task_ctx udp_task_ctx = {0};
    udp_ctx_init(&udp_task_ctx, &wifi_ap_ehandler_ctx, &ctrl_task_ctx);

    static StackType_t udp_task_stack[UDP_TASK_STACK_SIZE];
    static StaticTask_t udp_task_buffer;
    udp_task_handle = xTaskCreateStatic(udp_comm_task, UDP_TASK_NAME, UDP_TASK_STACK_SIZE, &udp_task_ctx, UDP_TASK_PRIORITY, udp_task_stack, &udp_task_buffer);
    if(!wifi_ap_ehandler_ctx.station_connected) {
        vTaskSuspend(udp_task_handle);
    }

    // gptimer setup
    gptimer_handle_t timer = NULL;
    gptimer_init(&timer);
    
    // button setup
    static uint8_t button_queue_storage[BUTTON_QUEUE_LENGHT * BUTTON_QUEUE_ITEM_SIZE];
    static StaticQueue_t button_queue_struct;

    button_queue = xQueueCreateStatic(BUTTON_QUEUE_LENGHT, BUTTON_QUEUE_ITEM_SIZE, button_queue_storage, &button_queue_struct);

    _Alignas(BUTTON_STORAGE_ALIGNMENT) static uint8_t button_start_stop_storage[BUTTON_STORAGE_SIZE] = {0};
    
    static struct button_start_stop_ctx btn_start_stop_ctx = {0};
    btn_start_stop_ctx.timer = timer;
    btn_start_stop_ctx.motor_reg = &motor_reg;
    btn_start_stop_ctx.ctrl_mutex = &ctrl_task_ctx.mutex;
    btn_start_stop_ctx.timer_started = false;

    button_handle button_start_stop = NULL;
    btn_init(&button_start_stop, (void*)&btn_start_stop_ctx, button_start_stop_storage);

    // button task setup
    static button_task_ctx btn_task_ctx = {0};
    btn_task_ctx.button_queue = button_queue;

    static StackType_t button_task_stack[BUTTON_TASK_STACK_SIZE];
    static StaticTask_t button_task_buffer;
    button_task_handle = xTaskCreateStatic(button_task, BUTTON_TASK_NAME, BUTTON_TASK_STACK_SIZE, &btn_task_ctx, BUTTON_TASK_PRIORITY, button_task_stack, &button_task_buffer);
}

static void nvs_init(void) {
    esp_err_t err = nvs_flash_init();
    if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void pid_init(pid_control_handle* handle, uint8_t* storage) {
    pid_control_config pid_config = {0};
    esp_err_t err = nvs_read_pid_config(NVS_PID_CONFIG_KEY, &pid_config);
    if(err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(pid_control_init(storage, PID_CONTROL_STORAGE_SIZE, handle, &pid_config));
}

static void lpf_init(low_pass_filter_handle* handle, uint8_t* storage) {
    const low_pass_filter_config filter_config = {
        .cutoff_freq = FILTER_CUTOFF_FREQ_HZ,
        .sampling_freq = FILTER_SAMPLING_FREQ_HZ,
    };

    ESP_ERROR_CHECK(low_pass_filter_init(storage, LOW_PASS_FILTER_STORAGE_SIZE, &filter_config, handle));
}

static void pwm_init_start(motor_handle* motor, uint8_t* storage) {
    const motor_config motor_cfg = {
        .gpio_config = MOTOR_GPIO,
        .pwm_config = {
            .group_id = PWM_GROUP_ID,
            .resolution_hz = PWM_RESOLUTION_HZ,
            .pwm_frequency_hz = PWM_FREQUENCY_HZ,
        },
    };

    ESP_ERROR_CHECK(motor_init(storage, MOTOR_STORAGE_SIZE, &motor_cfg, motor));
    ESP_ERROR_CHECK(motor_pwm_enable_start(*motor));
}

static void ctrl_ctx_init(pid_control_handle pid, low_pass_filter_handle filter, motor_cmpr_reg* cmpr_reg, pcnt_unit_handle_t pcnt_unit, float setpoint, control_task_ctx* ctx) {
    ctx->pid = pid;
    ctx->filter = filter;
    ctx->motor_cmpr_reg = cmpr_reg;
    ctx->pcnt_unit = pcnt_unit;
    ctx->setpoint = setpoint;
    ctx->pwm_max_ticks = (PWM_RESOLUTION_HZ / PWM_FREQUENCY_HZ) - 1U;
    (void)memcpy(&ctx->mutex, &(portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED, sizeof(portMUX_TYPE));
}

static void pcnt_init(pcnt_unit_handle_t* pcnt_unit) {
    const pcnt_unit_config_t pcnt_config = {
        .high_limit = ENCODER_HIGH_LIMIT,
        .low_limit = ENCODER_LOW_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_config, pcnt_unit));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_GPIO_A,
        .level_gpio_num = ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*pcnt_unit, &chan_a_config, &chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_GPIO_B,
        .level_gpio_num = ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*pcnt_unit, &chan_b_config, &chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(*pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*pcnt_unit));
}

static void wifi_ap_init(wifi_ap_event_handler_ctx* event_handler_ctx) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    (void)esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, event_handler_ctx, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .password = WIFI_AP_PASSWORD,
            .channel = WIFI_AP_CHANNEL,
            .max_connection = WIFI_AP_MAX_CONNECTIONS,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = true,
            }
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void udp_ctx_init(udp_comm_task_ctx* ctx, wifi_ap_event_handler_ctx* ehandler_ctx, control_task_ctx* ctrl_ctx) {
    ctx->ehandler_ctx = ehandler_ctx;
    ctx->first_exchange = true;
    ctx->ctrl_task_ctx = ctrl_ctx;
}

static void gptimer_init(gptimer_handle_t* timer) {
    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, timer));

    const gptimer_alarm_config_t alarm_config = {
        .alarm_count = TIMER_ALARM_COUNT,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(*timer, &alarm_config));

    const gptimer_event_callbacks_t timer_callback = {
        .on_alarm = control_task_notify_isr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(*timer, &timer_callback, NULL));

    ESP_ERROR_CHECK(gptimer_enable(*timer));
}

static void button_start_stop_callback(button_handle handle, void* ctx) {
    if(handle == NULL || ctx == NULL) {
        return;
    }

    struct button_start_stop_ctx* btn_ctx = ctx;
    if(btn_ctx->timer_started) {
        ESP_ERROR_CHECK(gptimer_stop(btn_ctx->timer));
        btn_ctx->timer_started = false;

        taskENTER_CRITICAL(btn_ctx->ctrl_mutex);
        btn_ctx->motor_reg->cmpr_a_gen_reg->gen = 0;
        btn_ctx->motor_reg->cmpr_b_gen_reg->gen = 0;
        taskEXIT_CRITICAL(btn_ctx->ctrl_mutex);
    } else {
        ESP_ERROR_CHECK(gptimer_start(btn_ctx->timer));
        btn_ctx->timer_started = true;
    }
}

static void btn_init(button_handle* handle, void* callback_ctx, uint8_t* storage) {
    const button_config btn_config = {
        .gpio_num = BUTTON_START_STOP_GPIO,
        .debounce_ms = BUTTON_DEBOUNCE_MS,
        .queue = button_queue,
        .callback = button_start_stop_callback,
        .callback_ctx = callback_ctx,
    };
    ESP_ERROR_CHECK(button_init(storage, BUTTON_STORAGE_SIZE, &btn_config, handle));
}
