#include <stdio.h>
#include "Motor.h"
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_err.h"

#define MOTOR_ENA_IO 5
#define MOTOR_IN1_IO 16
#define MOTOR_IN2_IO 17

void motor_pwm_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .gpio_num = MOTOR_ENA_IO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void motor_gpio_init(void)
{
    gpio_reset_pin(MOTOR_IN1_IO);
    gpio_set_direction(MOTOR_IN1_IO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(MOTOR_IN2_IO);
    gpio_set_direction(MOTOR_IN2_IO, GPIO_MODE_OUTPUT);

    gpio_set_level(MOTOR_IN1_IO, 0);
    gpio_set_level(MOTOR_IN2_IO, 0);
}

static void motor_forward()
{
    gpio_set_level(MOTOR_IN1_IO, 1);
    gpio_set_level(MOTOR_IN2_IO, 0);
}

static void motor_backward()
{
    gpio_set_level(MOTOR_IN1_IO, 0);
    gpio_set_level(MOTOR_IN2_IO, 1);
}

void motor_set_duty(int32_t duty)
{
    if (duty > 0)
    {
        motor_forward();
    }
    else
    {
        motor_backward();
        duty = -duty;
    }
    duty = duty > MOTOR_MAX_DUTY ? MOTOR_MAX_DUTY : duty;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

void motor_stop()
{
    gpio_set_level(MOTOR_IN1_IO, 0);
    gpio_set_level(MOTOR_IN2_IO, 0);
    motor_set_duty(0);
}

void motor_brake(int32_t duty)
{
    gpio_set_level(MOTOR_IN1_IO, 1);
    gpio_set_level(MOTOR_IN2_IO, 0);
    gpio_set_level(MOTOR_ENA_IO, 0);
    // motor_set_duty(1023);
}
