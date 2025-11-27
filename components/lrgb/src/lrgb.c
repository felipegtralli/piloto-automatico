#include "lrgb.h"
#include "driver/ledc.h"

void lrgb_set_duty_update(rgb_channel_config channels, uint32_t red_duty, uint32_t green_duty, uint32_t blue_duty) {
    (void)ledc_set_duty(LEDC_MODE, channels.red_channel, red_duty);
    (void)ledc_set_duty(LEDC_MODE, channels.green_channel, green_duty);
    (void)ledc_set_duty(LEDC_MODE, channels.blue_channel, blue_duty);

    (void)ledc_update_duty(LEDC_MODE, channels.red_channel);
    (void)ledc_update_duty(LEDC_MODE, channels.green_channel);
    (void)ledc_update_duty(LEDC_MODE, channels.blue_channel);
}
