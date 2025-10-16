/* includes */
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "driver/gptimer_types.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "app_main.h"
#include "low_pass_filter.h"
#include "pid_control.h"
#include "tasks.h"
#include "nvs_helper.h"

/* defines */
#define TIMER_RESOLUTION_HZ 1000000U // 1 MHz
#define SAMPLING_RATE_HZ 10U
#define TIMER_ALARM_COUNT (TIMER_RESOLUTION_HZ / SAMPLING_RATE_HZ)

/* variables */
TaskHandle_t control_motor_a_task_handle = NULL;
TaskHandle_t control_motor_b_task_handle = NULL;

static control_task_ctx control_task_ctx_motor_a;
static control_task_ctx control_task_ctx_motor_b;

static StackType_t control_motor_a_task_stack[CONTROL_TASK_STACK_SIZE];
static StaticTask_t control_motor_a_task_buffer;

static StackType_t control_motor_b_task_stack[CONTROL_TASK_STACK_SIZE];
static StaticTask_t control_motor_b_task_buffer;

/* function prototypes */
static void nvs_init(void);
static void pid_init(pid_control_handle* handle, uint8_t* storage);
static void lpf_init(low_pass_filter_handle* handle, uint8_t* storage);
static void ctrl_ctx_init(pid_control_handle pid, low_pass_filter_handle filter, control_task_ctx* ctx);
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

    // control task setup
    ctrl_ctx_init(pid_motor_a, filter_motor_a, &control_task_ctx_motor_a);
    ctrl_ctx_init(pid_motor_b, filter_motor_b, &control_task_ctx_motor_b);

    control_motor_a_task_handle = xTaskCreateStatic(control_task, CONTROL_TASK_A_NAME, CONTROL_TASK_STACK_SIZE, &control_task_ctx_motor_a, CONTROL_TASK_PRIORITY, control_motor_a_task_stack, &control_motor_a_task_buffer);
    control_motor_b_task_handle = xTaskCreateStatic(control_task, CONTROL_TASK_B_NAME, CONTROL_TASK_STACK_SIZE, &control_task_ctx_motor_b, CONTROL_TASK_PRIORITY, control_motor_b_task_stack, &control_motor_b_task_buffer);

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
    low_pass_filter_config filter_config = {0};
    nvs_read_filter_config(NVS_FILTER_CONFIG_KEY, &filter_config);

    ESP_ERROR_CHECK(low_pass_filter_init(storage, LOW_PASS_FILTER_STORAGE_SIZE, &filter_config, handle));
}

static void ctrl_ctx_init(pid_control_handle pid, low_pass_filter_handle filter, control_task_ctx* ctx) {
    ctx->pid = pid;
    ctx->filter = filter;
    ctx->setpoint = 50.0f; // hardcoded value for testing. replace with nvs read
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
