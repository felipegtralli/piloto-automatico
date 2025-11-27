#ifndef __LED_RGB_H__
#define __LED_RGB_H__

#include <stdint.h>
#include "driver/ledc.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE

#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY_HZ 4000

#define RGB_TO_DUTY(x) (x * (1 << LEDC_DUTY_RES) / 255)

// RED       - R: 255, G: 0  , B: 0
#define RED_R           RGB_TO_DUTY(255)
#define RED_G           RGB_TO_DUTY(0)
#define RED_B           RGB_TO_DUTY(0)

// YELLOW    - R: 255, G: 255, B: 0
#define YELLOW_R        RGB_TO_DUTY(255)
#define YELLOW_G        RGB_TO_DUTY(255)
#define YELLOW_B        RGB_TO_DUTY(0)

// GREEN    - R: 0  , G: 255, B: 0
#define GREEN_R         RGB_TO_DUTY(0)
#define GREEN_G         RGB_TO_DUTY(255)
#define GREEN_B         RGB_TO_DUTY(0)

// CYAN      - R: 0  , G: 255, B: 255
#define CYAN_R          RGB_TO_DUTY(0)
#define CYAN_G          RGB_TO_DUTY(255)
#define CYAN_B          RGB_TO_DUTY(255)

// BLUE     - R: 0  , G: 0   , B: 255
#define BLUE_R          RGB_TO_DUTY(0)
#define BLUE_G          RGB_TO_DUTY(0)
#define BLUE_B          RGB_TO_DUTY(255)

// MAGENTA   - R: 255, G: 0  , B: 255
#define MAGENTA_R       RGB_TO_DUTY(255)
#define MAGENTA_G       RGB_TO_DUTY(0)
#define MAGENTA_B       RGB_TO_DUTY(255)

// ORANGE   - R: 255, G: 128, B: 0
#define ORANGE_R        RGB_TO_DUTY(255)
#define ORANGE_G        RGB_TO_DUTY(128)
#define ORANGE_B        RGB_TO_DUTY(0)

// ROSE PINK - R: 255, G: 0  , B: 128
#define ROSE_R          RGB_TO_DUTY(255)
#define ROSE_G          RGB_TO_DUTY(0)
#define ROSE_B          RGB_TO_DUTY(128)

// BLUEISH PURPLE - R: 178, G: 102, B: 255
#define BLUEISH_PURPLE_R      RGB_TO_DUTY(178)
#define BLUEISH_PURPLE_G      RGB_TO_DUTY(102)
#define BLUEISH_PURPLE_B      RGB_TO_DUTY(255)

typedef struct {
    uint32_t red_channel;
    uint32_t green_channel;
    uint32_t blue_channel;
} rgb_channel_config;

void lrgb_set_duty_update(rgb_channel_config channels, uint32_t red_duty, uint32_t green_duty, uint32_t blue_duty);

#endif // __LED_RGB_H__
