#pragma once
#include "esp_err.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float I;
    float I_output_limit;
} pid_t;

extern pid_t pid;

esp_err_t Controller_Init(void);
void Controller_Update(float speed, float dt);
void Controller_GetSpeed(float *current_speed);
int16_t Controller_AutoTune(int tune_pwm, float dt);

void Controller_SetPIDParams(pid_t *pid_params);
void Controller_GetPIDParams(pid_t *pid_params);
