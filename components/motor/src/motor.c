#include "motor.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/mcpwm_types.h"
#include "soc/clk_tree_defs.h"
#include "soc/gpio_num.h"
#include "soc/mcpwm_struct.h"
#include <stdint.h>

static const char TAG[] = "motor";

struct motor {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t comparator_a;
    mcpwm_cmpr_handle_t comparator_b;
    mcpwm_gen_handle_t generator_a;
    mcpwm_gen_handle_t generator_b;
    volatile mcpwm_gen_tstmp_reg_t* cmpr_a_reg;
    volatile mcpwm_gen_tstmp_reg_t* cmpr_b_reg;
};

_Static_assert(MOTOR_STORAGE_SIZE >= sizeof(struct motor), "MOTOR_STORAGE_SIZE is too small");
_Static_assert(MOTOR_STORAGE_ALIGNMENT >= _Alignof(struct motor), "MOTOR_STORAGE_ALIGNMENT is too small");
_Static_assert((MOTOR_STORAGE_ALIGNMENT & (MOTOR_STORAGE_ALIGNMENT - 1U)) == 0U, "MOTOR_STORAGE_ALIGNMENT must be a power of two");

extern int mcpwm_get_operator_id(mcpwm_cmpr_handle_t cmpr);
extern int mcpwm_get_comparator_id(mcpwm_cmpr_handle_t cmpr);

// global helpers
// MISRA C:2012 R11.4 deviation – pointer to integer cast for alignment check (no dereference).
static inline bool is_aligned(const void* ptr, size_t align) {
    const uintptr_t mask = (uintptr_t)align - (uintptr_t)1u;
    return ((uintptr_t)ptr & mask) == (uintptr_t)0;
}

// init helpers
static esp_err_t motor_validate_storage(const void* storage, size_t storage_size, size_t req_size, size_t req_align) {
    if(storage_size < req_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    if(!is_aligned(storage, req_align)) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t motor_validate_config(const motor_config* config) {
    if(config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if((config->gpio_config.pwma_gpio < 0 || config->gpio_config.pwma_gpio >= GPIO_NUM_MAX) || (config->gpio_config.pwmb_gpio < 0 || config->gpio_config.pwmb_gpio >= GPIO_NUM_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }

    if(config->pwm_config.resolution_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t pwm_timer_init(const motor_pwm_config* config, struct motor* motor) {
    const mcpwm_timer_config_t timer_config = {
        .group_id = config->group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = config->resolution_hz,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = (config->resolution_hz / config->pwm_frequency_hz),
    }; 
    
    return mcpwm_new_timer(&timer_config, &motor->timer);
}

static esp_err_t pwm_operator_init(const motor_pwm_config* config, struct motor* motor) {
    const mcpwm_operator_config_t operator_config = {
        .group_id = config->group_id,
    };
    esp_err_t err = mcpwm_new_operator(&operator_config, &motor->operator);
    if(err != ESP_OK) {
        return err;
    }

    return mcpwm_operator_connect_timer(motor->operator, motor->timer);
}

static esp_err_t pwm_comparator_init(const motor_config* config, struct motor* motor) {
    const mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    
    esp_err_t err = mcpwm_new_comparator(motor->operator, &comparator_config, &motor->comparator_a);
    if(err != ESP_OK) {
        return err;
    }
    err = mcpwm_new_comparator(motor->operator, &comparator_config, &motor->comparator_b);
    if(err != ESP_OK) {
        return err;
    }

    (void)mcpwm_comparator_set_compare_value(motor->comparator_a, 0);
    (void)mcpwm_comparator_set_compare_value(motor->comparator_b, 0);

    return ESP_OK;
}

static esp_err_t pwm_generator_init(const motor_config* config, struct motor* motor) {
    const mcpwm_generator_config_t generator_config_a = {
        .gen_gpio_num = config->gpio_config.pwma_gpio,
    };
    esp_err_t err = mcpwm_new_generator(motor->operator, &generator_config_a, &motor->generator_a);
    if(err != ESP_OK) {
        return err;
    }

    const mcpwm_generator_config_t generator_config_b = {
        .gen_gpio_num = config->gpio_config.pwmb_gpio,
    };
    err = mcpwm_new_generator(motor->operator, &generator_config_b, &motor->generator_b);
    if(err != ESP_OK) {
        return err;
    }

    mcpwm_gen_timer_event_action_t timer_action = MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH);

    mcpwm_gen_compare_event_action_t compare_action_a = MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->comparator_a, MCPWM_GEN_ACTION_LOW);
    mcpwm_gen_compare_event_action_t compare_action_b = MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->comparator_b, MCPWM_GEN_ACTION_LOW);

    (void)mcpwm_generator_set_action_on_timer_event(motor->generator_a, timer_action);
    (void)mcpwm_generator_set_action_on_compare_event(motor->generator_a, compare_action_a);

    (void)mcpwm_generator_set_action_on_timer_event(motor->generator_b, timer_action);
    (void)mcpwm_generator_set_action_on_compare_event(motor->generator_b, compare_action_b);

    return ESP_OK;
}

static void motor_set_cmpr_reg(struct motor* motor, const motor_config* config) {
    if(motor == NULL || config == NULL) {
        return;
    }

    mcpwm_dev_t* mcpwm = (config->pwm_config.group_id == 0) ? &MCPWM0 : &MCPWM1;

    int operator_id = mcpwm_get_operator_id(motor->comparator_a);
    int comparator_a_id = mcpwm_get_comparator_id(motor->comparator_a);
    int comparator_b_id = mcpwm_get_comparator_id(motor->comparator_b);

    motor->cmpr_a_reg = &mcpwm->operators[operator_id].timestamp[comparator_a_id];
    motor->cmpr_b_reg = &mcpwm->operators[operator_id].timestamp[comparator_b_id];
}

static void motor_cleanup(struct motor* motor) {
    if(motor != NULL) {
        if(motor->generator_a != NULL) {
            mcpwm_del_generator(motor->generator_a);
            motor->generator_a = NULL;
        }
        if(motor->generator_b != NULL) {
            mcpwm_del_generator(motor->generator_b);
            motor->generator_b = NULL;
        }
        if(motor->comparator_a != NULL) {
            mcpwm_del_comparator(motor->comparator_a);
            motor->comparator_a = NULL;
        }
        if(motor->comparator_b != NULL) {
            mcpwm_del_comparator(motor->comparator_b);
            motor->comparator_b = NULL;
        }
        if(motor->operator != NULL) {
            mcpwm_del_operator(motor->operator);
            motor->operator = NULL;
        }
        if(motor->timer != NULL) {
            mcpwm_del_timer(motor->timer);
            motor->timer = NULL;
        }
    }
}

esp_err_t motor_init(void* storage, size_t storage_size, const motor_config* config, motor_handle* handle) {
    esp_err_t status = ESP_OK;
    if(storage == NULL || config == NULL || handle == NULL) {
        status = ESP_ERR_INVALID_ARG;
        ESP_LOGE(TAG, "Invalid argument");
    } else {
        *handle = NULL;
    }

    if(status == ESP_OK) {
        status = motor_validate_storage(storage, storage_size, MOTOR_STORAGE_SIZE, MOTOR_STORAGE_ALIGNMENT);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Invalid storage (size or alignment)");
        }
    }

    if(status == ESP_OK) {
        status = motor_validate_config(config);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Invalid motor configuration");
        }
    }
    
    struct motor* motor = NULL;
    if(status == ESP_OK) {
        // MISRA C:2012 R11.5 deviation – converting void* storage to object pointer after validation.
        motor = storage;

        status = pwm_timer_init(&config->pwm_config, motor);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize PWM timer");
        }
    }

    if(status == ESP_OK) {
        status = pwm_operator_init(&config->pwm_config, motor);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize PWM operator");
        }
    }

    if(status == ESP_OK) {
        status = pwm_comparator_init(config, motor);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize PWM comparator");
        }
    }

    if(status == ESP_OK) {
        status = pwm_generator_init(config, motor);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize PWM generator");
        }
    }

    if(status == ESP_OK) {
        motor_set_cmpr_reg(motor, config);
        
        ESP_LOGI(TAG, "Motor initialized successfully");
        *handle = motor;
    } else {
        motor_cleanup(motor);
        *handle = NULL;
    }

    return status;
}

esp_err_t motor_free(motor_handle handle) {
    esp_err_t status = ESP_OK;

    struct motor* motor = NULL;
    if(handle == NULL) {
        status = ESP_ERR_INVALID_ARG;
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Invalid argument");
        }
    } else {
        motor = handle;
    }

    if(status == ESP_OK) {
        motor_cleanup(motor);
        handle = NULL;
        
        ESP_LOGI(TAG, "Motor resources freed");
    }

    return status;
}

esp_err_t motor_pwm_enable_start(motor_handle handle) {
    esp_err_t status = ESP_OK;

    struct motor* motor = NULL;
    if(handle == NULL) {
        status = ESP_ERR_INVALID_ARG;
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Invalid argument");
        }
    } else {
        motor = handle;
    }

    if(status == ESP_OK) {
        status = mcpwm_timer_enable(motor->timer);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable PWM timer");
        }
    }

    if(status == ESP_OK) {
        status = mcpwm_timer_start_stop(motor->timer, MCPWM_TIMER_START_NO_STOP);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start PWM timer");
        } else {
            ESP_LOGI(TAG, "PWM timer started");
        }
    }

    return status;
}

esp_err_t motor_pwm_disable_stop(motor_handle handle) {
    esp_err_t status = ESP_OK;

    struct motor* motor = NULL;
    if(handle == NULL) {
        status = ESP_ERR_INVALID_ARG;
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Invalid argument");
        }
    } else {
        motor = handle;
    }

    if(status == ESP_OK) {
        status = mcpwm_timer_disable(motor->timer);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to disable PWM timer");
        }
    }

    if(status == ESP_OK) {
        status = mcpwm_timer_start_stop(motor->timer, MCPWM_TIMER_STOP_EMPTY);
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop PWM timer");
        } else {
            ESP_LOGI(TAG, "PWM timer stopped");
        }
    }

    return status;
}

esp_err_t motor_get_cmpr_reg(motor_handle handle, motor_cmpr_reg* cmpr_reg) {
    esp_err_t status = ESP_OK;
    if(handle == NULL || cmpr_reg == NULL) {
        status = ESP_ERR_INVALID_ARG;
        if(status != ESP_OK) {
            ESP_LOGE(TAG, "Invalid argument");
        }
    }

    if(status == ESP_OK) {
        struct motor* motor = handle;

        cmpr_reg->cmpr_a_gen_reg = motor->cmpr_a_reg;
        cmpr_reg->cmpr_b_gen_reg = motor->cmpr_b_reg;
    }

    return status;
}
