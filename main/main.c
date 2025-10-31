/* includes */
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

/* defines */
#define TIMER_RESOLUTION_HZ 1000000U // 1 MHz
#define SAMPLING_RATE_HZ 100U
#define TIMER_ALARM_COUNT (TIMER_RESOLUTION_HZ / SAMPLING_RATE_HZ)

#define FILTER_CUTOFF_FREQ_HZ 10.0f
#define FILTER_SAMPLING_FREQ_HZ (float)SAMPLING_RATE_HZ

#define PWM_GROUP_ID 0
#define PWM_RESOLUTION_HZ 10000000U // 10 MHz
#define PWM_FREQUENCY_HZ 25000U      // 25 kHz

/* variables */
TaskHandle_t control_motor_a_task_handle = NULL;
TaskHandle_t control_motor_b_task_handle = NULL;

static control_task_ctx control_task_ctx_motor_a;
static control_task_ctx control_task_ctx_motor_b;

static StackType_t control_motor_a_task_stack[CONTROL_TASK_STACK_SIZE];
static StaticTask_t control_motor_a_task_buffer;

static StackType_t control_motor_b_task_stack[CONTROL_TASK_STACK_SIZE];
static StaticTask_t control_motor_b_task_buffer;

TaskHandle_t udp_task_handle = NULL;

static udp_comm_task_ctx udp_task_ctx;

static StackType_t udp_task_stack[UDP_TASK_STACK_SIZE];
static StaticTask_t udp_task_buffer;

/* function prototypes */
static void nvs_init(void);
static void pid_init(pid_control_handle* handle, uint8_t* storage);
static void lpf_init(low_pass_filter_handle* handle, uint8_t* storage);
static void pwm_init_start(motor_handle* motor, uint8_t* storage, const char* nvs_gpio_key);
static void ctrl_ctx_init(pid_control_handle pid, low_pass_filter_handle filter, motor_cmpr_reg* cmpr_reg, float setpoint, control_task_ctx* ctx);
static void wifi_ap_init(wifi_config_t* wifi_config, wifi_ap_event_handler_ctx* event_handler_ctx);
static void udp_ctx_init(udp_comm_task_ctx* ctx, control_task_ctx* ctrl_ctx_motor_a, control_task_ctx* ctrl_ctx_motor_b);
static void gptimer_init(void);

void app_main(void) {
    // nvs setup
    nvs_init();

    // pid controller setup
    _Alignas(PID_CONTROL_STORAGE_ALIGNMENT) static uint8_t pid_storage_motor_a[PID_CONTROL_STORAGE_SIZE];
    _Alignas(PID_CONTROL_STORAGE_ALIGNMENT) static uint8_t pid_storage_motor_b[PID_CONTROL_STORAGE_SIZE];

    pid_control_handle pid_motor_a = NULL;
    pid_init(&pid_motor_a, pid_storage_motor_a);

    pid_control_handle pid_motor_b = NULL;
    pid_init(&pid_motor_b, pid_storage_motor_b);

    // low pass filter setup
    _Alignas(LOW_PASS_FILTER_STORAGE_ALIGNMENT) static uint8_t filter_storage_motor_a[LOW_PASS_FILTER_STORAGE_SIZE];
    _Alignas(LOW_PASS_FILTER_STORAGE_ALIGNMENT) static uint8_t filter_storage_motor_b[LOW_PASS_FILTER_STORAGE_SIZE];

    low_pass_filter_handle filter_motor_a = NULL;
    lpf_init(&filter_motor_a, filter_storage_motor_a);

    low_pass_filter_handle filter_motor_b = NULL;
    lpf_init(&filter_motor_b, filter_storage_motor_b);

    // pwm setup
    _Alignas(MOTOR_STORAGE_ALIGNMENT) static uint8_t motor_storage_a[MOTOR_STORAGE_SIZE];
    _Alignas(MOTOR_STORAGE_ALIGNMENT) static uint8_t motor_storage_b[MOTOR_STORAGE_SIZE];

    motor_handle motor_a = NULL;
    pwm_init_start(&motor_a, motor_storage_a, NVS_MOTOR_A_GPIO_KEY);

    motor_handle motor_b = NULL;
    pwm_init_start(&motor_b, motor_storage_b, NVS_MOTOR_B_GPIO_KEY);

    static motor_cmpr_reg motor_a_cmpr_reg = {0};
    ESP_ERROR_CHECK(motor_get_cmpr_reg(motor_a, &motor_a_cmpr_reg));

    static motor_cmpr_reg motor_b_cmpr_reg = {0};
    ESP_ERROR_CHECK(motor_get_cmpr_reg(motor_b, &motor_b_cmpr_reg));

    // control task setup
    float setpoint = 0.0f;
    nvs_read_setpoint(NVS_SETPOINT_KEY, &setpoint);

    ctrl_ctx_init(pid_motor_a, filter_motor_a, &motor_a_cmpr_reg, setpoint, &control_task_ctx_motor_a);
    ctrl_ctx_init(pid_motor_b, filter_motor_b, &motor_b_cmpr_reg, setpoint, &control_task_ctx_motor_b);

    control_motor_a_task_handle = xTaskCreateStatic(control_task, CONTROL_TASK_A_NAME, CONTROL_TASK_STACK_SIZE, &control_task_ctx_motor_a, CONTROL_TASK_PRIORITY, control_motor_a_task_stack, &control_motor_a_task_buffer);
    control_motor_b_task_handle = xTaskCreateStatic(control_task, CONTROL_TASK_B_NAME, CONTROL_TASK_STACK_SIZE, &control_task_ctx_motor_b, CONTROL_TASK_PRIORITY, control_motor_b_task_stack, &control_motor_b_task_buffer);

    // wifi setup
    static wifi_ap_event_handler_ctx wifi_ap_ehandler_ctx = {
        .station_connected = false,
    };
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
    wifi_ap_init(&wifi_config, &wifi_ap_ehandler_ctx);

    // udp task setup
    udp_ctx_init(&udp_task_ctx, &control_task_ctx_motor_a, &control_task_ctx_motor_b);

    udp_task_handle = xTaskCreateStatic(udp_comm_task, UDP_TASK_NAME, UDP_TASK_STACK_SIZE, &udp_task_ctx, UDP_TASK_PRIORITY, udp_task_stack, &udp_task_buffer);
    if(!wifi_ap_ehandler_ctx.station_connected) {
        vTaskSuspend(udp_task_handle);
    }

    // gptimer setup
    gptimer_init();
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
    nvs_read_pid_config(NVS_PID_CONFIG_KEY, &pid_config);

    ESP_ERROR_CHECK(pid_control_init(storage, PID_CONTROL_STORAGE_SIZE, handle, &pid_config));
}

static void lpf_init(low_pass_filter_handle* handle, uint8_t* storage) {
    low_pass_filter_config filter_config = {
        .cutoff_freq = FILTER_CUTOFF_FREQ_HZ,
        .sampling_freq = FILTER_SAMPLING_FREQ_HZ,
    };

    ESP_ERROR_CHECK(low_pass_filter_init(storage, LOW_PASS_FILTER_STORAGE_SIZE, &filter_config, handle));
}

static void pwm_init_start(motor_handle* motor, uint8_t* storage, const char* nvs_gpio_key) {
    motor_gpio_config gpio_config = {0};
    nvs_read_motor_gpio(nvs_gpio_key, &gpio_config);

    motor_config motor_cfg = {
        .gpio_config = gpio_config,
        .pwm_config = {
            .group_id = PWM_GROUP_ID,
            .resolution_hz = PWM_RESOLUTION_HZ,
            .pwm_frequency_hz = PWM_FREQUENCY_HZ,
        },
    };

    ESP_ERROR_CHECK(motor_init(storage, MOTOR_STORAGE_SIZE, &motor_cfg, motor));
    ESP_ERROR_CHECK(motor_pwm_enable_start(*motor));
}

static void ctrl_ctx_init(pid_control_handle pid, low_pass_filter_handle filter, motor_cmpr_reg* cmpr_reg, float setpoint, control_task_ctx* ctx) {
    ctx->pid = pid;
    ctx->filter = filter;
    ctx->motor_cmpr_reg = cmpr_reg;
    ctx->setpoint = setpoint;
    ctx->pwm_max_ticks = (PWM_RESOLUTION_HZ / PWM_FREQUENCY_HZ) - 1U;
    memcpy(&ctx->mutex, &(portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED, sizeof(portMUX_TYPE));
}

static void wifi_ap_init(wifi_config_t* wifi_config, wifi_ap_event_handler_ctx* event_handler_ctx) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    (void)esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, event_handler_ctx, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void udp_ctx_init(udp_comm_task_ctx* ctx, control_task_ctx* ctrl_ctx_motor_a, control_task_ctx* ctrl_ctx_motor_b) {
    ctx->first_exchange = true;
    ctx->control_ctx_motor_a = ctrl_ctx_motor_a;
    ctx->control_ctx_motor_b = ctrl_ctx_motor_b;
}

static void gptimer_init(void) {
    gptimer_handle_t timer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = TIMER_ALARM_COUNT,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));

    gptimer_event_callbacks_t timer_callback = {
        .on_alarm = control_task_notify_isr,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &timer_callback, NULL));

    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
}
