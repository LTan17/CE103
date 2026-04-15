#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "Controller.h"
#include "Encoder.h"
#include "Motor.h"

#define MOTOR_MAX_RPM 333

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float I;
    float I_output_limit;
} pid_t;

pid_t pid = {.Kp = 5.0, .Ki = 17.8, .Kd = 0.3, .prev_error = 0, .I = 0, .I_output_limit = 300.0};

float display_current_speed, display_target_speed = 0.0;

#define RELAY_STEP 150
#define HYSTERESIS 10

int tune_state = -1;
float tune_max = 0.0f, tune_min = 99999.0f;
int64_t t1 = 0, t2 = 0;
int crossing_count = 0;
bool relay_state = true;

static int32_t pid_calculate(pid_t *pid, float setpoint, float measured_value, float dt)
{
    static float alpha = 0.5;
    static float D_out_filter = 0;
    float error = setpoint - measured_value;
    float P_output = pid->Kp * error;

    pid->I += error * dt;
    float I_limit = pid->I_output_limit / pid->Ki;
    pid->I = pid->I > I_limit ? I_limit : (pid->I < -I_limit ? -I_limit : pid->I);

    float I_output = pid->Ki * pid->I;

    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    D_out_filter = alpha * derivative + (1 - alpha) * D_out_filter;

    float output = P_output + I_output + D_out_filter;
    // printf(">P: %.2f\n", P_output);
    // printf(">I: %.2f\n", I_output);
    // printf(">D: %.2f\n", D_out_filter);
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

static float Controller_Cal_RPM(float dt)
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
    return current_speed;
}

static int16_t Controller_PID(float target_speed, float dt)
{
    float current_speed = Controller_Cal_RPM(dt);
    display_current_speed = current_speed;
    display_target_speed = target_speed;

    int16_t control_signal = pid_calculate(&pid, (float)target_speed, current_speed, dt);
    printf(">speed:%.2f\n", current_speed);

    // printf("Target speed: %.2f, Current speed: %.2f, Control signal: %d ", target_speed, current_speed, control_signal);
    return control_signal;
}

void Controller_Update(float target_speed, float dt)
{
    uint16_t base_duty = target_speed / MOTOR_MAX_RPM * MOTOR_MAX_DUTY;
    int16_t duty = Controller_PID(target_speed, dt);
    duty = base_duty + duty;
    duty = duty > MOTOR_MAX_DUTY ? MOTOR_MAX_DUTY : duty;
    printf(">duty: %d\n", duty);
    motor_set_duty(duty);
}

int16_t Controller_AutoTune(float target_speed, int64_t time, float dt)
{
    static int tune_pwm = 800;
    float current = Controller_Cal_RPM(dt);
    // if (tune_state == -1)
    // {
    //     if (current < target_speed)
    //     {
    //         tune_pwm += 10;
    //         printf(">Initial PWM: %d\n>current: %.2f\n", tune_pwm, current);
    //         motor_set_duty(tune_pwm);
    //         vTaskDelay(pdMS_TO_TICKS(100));
    //         return tune_state;
    //     }
    //     else
    //     {
    //         tune_state = 0;
    //         return tune_state;
    //     }
    // }
    tune_state = 0;
    if (tune_state == 0)
    {
        if (crossing_count > 0)
        {
            if (current > tune_max)
                tune_max = current;
            if (current < tune_min)
                tune_min = current;
            printf(">current: %.2f\n", current);
        }

        if (relay_state && current > (target_speed + HYSTERESIS))
        {
            relay_state = false;

            // if (crossing_count == 0)
            // {
            //     t1 = time;
            //     tune_max = current;
            //     tune_min = current;
            //     printf("First crossing at t=%.2fs, current: %.2f\n", t1 / 1000000.0, current);
            // }
            // else if (crossing_count == 2)
            // {
            //     t2 = time;
            //     printf("Third crossing at t=%.2fs, current: %.2f\n", t2 / 1000000.0, current);
            // }
            crossing_count++;
        }
        else if (!relay_state && current < (target_speed - HYSTERESIS))
        {
            relay_state = true;
            // crossing_count++;
            // printf("Second crossing at t=%.2fs, current: %.2f\n", time / 1000000.0, current);
        }
        // if (crossing_count >= 3)
        // {
        //     float Amplitude = (tune_max - tune_min) / 2.0f;
        //     float Tu = (t2 - t1) / 1000000.0f;

        //     float Ku = (4.0f * RELAY_STEP) / (3.14159f * Amplitude);

        //     pid.Kp = 0.6f * Ku;
        //     pid.Ki = (1.2f * Ku) / Tu;
        //     pid.Kd = (0.075f * Ku) * Tu;

        //     printf("\n--- AUTOTUNE SUCCESS ---\n");
        //     printf("A: %.1f | Tu: %.3fs | Ku: %.3f\n", Amplitude, Tu, Ku);
        //     printf("Kp: %.3f | Ki: %.3f | Kd: %.3f\n", pid.Kp, pid.Ki, pid.Kd);

        //     tune_state = 1;
        //     return tune_state;
        // }
    }

    if (relay_state == true)
    {
        motor_set_duty(tune_pwm + (int16_t)RELAY_STEP);
        return tune_state;
    }
    else
    {
        motor_set_duty(tune_pwm - (int16_t)RELAY_STEP);
        return tune_state;
    }
}

void Controller_GetParams(float *current_speed, float *target_speed, float *kP, float *kI, float *kD)
{
    *current_speed = display_current_speed;
    *target_speed = display_target_speed;
    *kP = pid.Kp;
    *kI = pid.Ki;
    *kD = pid.Kd;
}
