#ifndef MOTOR_H
#define MOTOR_H

#include "esp_err.h"
#include "soc/gpio_num.h"
#include "soc/mcpwm_struct.h"
#include <stdint.h>
#include <stddef.h>

#define MOTOR_STORAGE_SIZE 32u
#define MOTOR_STORAGE_ALIGNMENT 4u

struct motor;
typedef struct motor* motor_handle;

typedef struct motor_gpio_config {
    gpio_num_t pwma_gpio;
    gpio_num_t pwmb_gpio;
} motor_gpio_config;

typedef struct motor_pwm_config {
    int group_id;
    uint32_t resolution_hz;
    uint32_t pwm_frequency_hz;
} motor_pwm_config;

typedef struct {
    motor_gpio_config gpio_config;
    motor_pwm_config pwm_config;
} motor_config;

typedef struct{
    volatile mcpwm_gen_tstmp_reg_t* cmpr_a_gen_reg;
    volatile mcpwm_gen_tstmp_reg_t* cmpr_b_gen_reg;
} motor_cmpr_reg;

esp_err_t motor_init(void* storage, size_t storage_size, const motor_config* config, motor_handle* handle);
esp_err_t motor_free(motor_handle handle);
esp_err_t motor_pwm_enable_start(motor_handle handle);
esp_err_t motor_pwm_disable_stop(motor_handle handle);
esp_err_t motor_get_cmpr_reg(motor_handle handle, motor_cmpr_reg* cmpr_reg);

#endif // MOTOR_H
