#include <stdio.h>
#include "Controller.h"
#include "Encoder.h"
#include "Motor.h"

#define MOTOR_MAX_RPM 110

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float I;
    float I_output_limit;
} pid_t;

pid_t pid = {.Kp = 12.0, .Ki = 0.4, .Kd = 0.005, .prev_error = 0, .I = 0, .I_output_limit = 50.0};

static int32_t pid_calculate(pid_t *pid, float setpoint, float measured_value, float dt)
{
    float error = setpoint - measured_value;
    float P_output = pid->Kp * error;

    pid->I += error;
    float I_limit = pid->I_output_limit / pid->Ki;
    pid->I = pid->I > I_limit ? I_limit : (pid->I < -I_limit ? -I_limit : pid->I);

    float I_output = pid->Ki * pid->I;

    float derivative = (error - pid->prev_error) / dt;
    float D_output = pid->Kd * derivative;

    float output = P_output + I_output + D_output;

    pid->prev_error = error;
    return (int32_t)output;
}

esp_err_t Controller_Init(void)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(Encoder_Init());
    motor_pwm_init();
    motor_gpio_init();

    return ESP_OK;
}

static int16_t Controler_PID(float target_speed, float dt)
{
    static int pre_encoder_count = 0;
    int encoder_count, diff_encoder_count;
    Encoder_Read(&encoder_count);
    diff_encoder_count = (encoder_count - pre_encoder_count);
    if (diff_encoder_count > ENCODER_COUNT / 2)
    {
        diff_encoder_count -= ENCODER_COUNT;
    }
    else if (diff_encoder_count < -ENCODER_COUNT / 2)
    {
        diff_encoder_count += ENCODER_COUNT;
    }
    pre_encoder_count = encoder_count;
    float current_speed = ((float)diff_encoder_count / (ENCODER_PPR) / dt) * 60.0;
    int16_t control_signal = pid_calculate(&pid, (float)target_speed, current_speed, dt);
    printf(">speed:%.2f\n", current_speed);

    // printf("Target speed: %.2f, Current speed: %.2f, Control signal: %d ", target_speed, current_speed, control_signal);
    return control_signal;
}

void Controller_Update(float target_speed, float dt)
{
    uint16_t base_duty = target_speed / MOTOR_MAX_RPM * MOTOR_MAX_DUTY;
    int16_t duty = Controler_PID(target_speed, dt);
    duty = base_duty + duty;
    duty = duty > MOTOR_MAX_DUTY ? MOTOR_MAX_DUTY : duty;
    // printf("duty: %d\n", duty);
    motor_set_duty(duty);
}
