#pragma once
#include "driver/ledc.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY (4000)

#define MOTOR_MAX_DUTY (1 << LEDC_DUTY_RES) - 1

void motor_pwm_init(void);
void motor_gpio_init(void);
void motor_set_duty(int32_t duty);
void motor_stop();
