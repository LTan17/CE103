#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "Controller.h"
#include "Encoder.h"
#include "Motor.h"

#define MOTOR_MAX_RPM 333

static const char *TAG = "Controller";

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float I;
    float I_output_limit;
} pid_t;

pid_t pid = {.Kp = 0.0f, .Ki = 0.0f, .Kd = 0.0f, .prev_error = 0, .I = 0, .I_output_limit = 300.0};

float display_current_speed, display_target_speed = 0.0;

#define RELAY_STEP 150
#define HYSTERESIS 1
#define MAX_SAMPLES 50

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
    printf(">P: %.2f\n", P_output);
    printf(">I: %.2f\n", I_output);
    printf(">D: %.2f\n", D_out_filter);
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
    duty = duty < 0 ? 0 : duty;
    duty = duty * abs(target_speed) / target_speed;
    printf(">duty: %d\n", duty);
    motor_set_duty(duty);
}

int16_t Controller_AutoTune(int tune_pwm, float dt)
{
    static int tune_state = 0;
    static int sample_count = 0;
    static bool relay_state = true;
    static float data[MAX_SAMPLES];
    static float time_period = 0, peak = 0, bottom = 0;
    static int start_up = 0;
    static float target_speed = 0.0f;
    float current = Controller_Cal_RPM(dt);

    if (start_up++ < 100)
    {
        target_speed = current;
        motor_set_duty(tune_pwm);
        printf("target: %.2f\n", target_speed);
        return 0;
    }
    if (tune_state == 0)
    {
        if (sample_count < MAX_SAMPLES)
            data[sample_count++] = current;
        else
            tune_state = 1;

        if (relay_state && current > (target_speed + HYSTERESIS))
            relay_state = false;
        else if (!relay_state && current < (target_speed - HYSTERESIS))
            relay_state = true;
        if (relay_state == true)
        {
            motor_set_duty(tune_pwm + (int16_t)RELAY_STEP);
            printf("tang\n");
            return 0;
        }
        else
        {
            motor_set_duty(tune_pwm - (int16_t)RELAY_STEP);
            printf("giam\n");
            return 0;
        }
    }

    if (tune_state == 1)
    {
        motor_set_duty(tune_pwm);

        float peak_array[MAX_SAMPLES / 2];
        float peak_index[MAX_SAMPLES / 2];
        float bottom_array[MAX_SAMPLES / 2];
        float bottom_index[MAX_SAMPLES / 2];

        int p_index = 0;
        int b_index = 0;

        for (int i = 1; i < MAX_SAMPLES - 1; i++)
        {
            printf("%.2f, ", data[i]);
            if (data[i] > (data[i - 1] + HYSTERESIS) && data[i] > (data[i + 1] + HYSTERESIS))
            {
                peak_array[p_index] = data[i];
                peak_index[p_index] = i;
                p_index++;
            }
            if (data[i] < (data[i - 1] - HYSTERESIS) && data[i] < (data[i + 1] - HYSTERESIS))
            {
                bottom_array[b_index] = data[i];
                bottom_index[b_index] = i;
                b_index++;
            }
        }

        if (p_index >= 3 && b_index >= 3)
        {
            printf("\nnum index: %d \n", p_index);
            for (int i = 1; i < p_index; i++)
            {
                printf("%.5f, ", peak_index[i]);
                peak += peak_array[i];
            }
            peak /= (p_index - 1);
            time_period = ((peak_index[p_index - 1] - peak_index[1]) / (p_index - 2)) * dt;
            for (int i = 1; i < b_index; i++)
            {
                bottom += bottom_array[i];
            }
            bottom /= (b_index - 1);
            tune_state = 2;
        }
        else
        {
            ESP_LOGE(TAG, "Loi: Khong tim du dinh/day! P_idx: %d, B_idx: %d", p_index, b_index);
            tune_state = -1;
        }
    }
    if (tune_state == 2)
    {
        float A = (peak - bottom) / 2.0f;
        float Ku = (4.0f * RELAY_STEP) / (3.14159f * A);
        float Tu = time_period;
        pid.Kp = 0.6 * Ku;
        pid.Ki = 1.2 * Ku / Tu;
        pid.Kd = 0.075 * Ku * Tu;
        printf("Auto-tune complete! Kp: %.2f, Ki: %.2f, Kd: %.2f\n", pid.Kp, pid.Ki, pid.Kd);
        tune_state = 3;
    }
    if (tune_state == 3)
    {
        ESP_LOGI(TAG, "Auto-tune complete! Kp: %.2f, Ki: %.2f, Kd: %.2f", pid.Kp, pid.Ki, pid.Kd);
        return 1;
    }
    return -1;
}

void Controller_GetParams(float *current_speed, float *kP, float *kI, float *kD)
{
    *current_speed = display_current_speed;
    *kP = pid.Kp;
    *kI = pid.Ki;
    *kD = pid.Kd;
}
