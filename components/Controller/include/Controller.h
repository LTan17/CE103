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
void Controller_GetParams(float *current_speed, float *target_speed, float *kP, float *kI, float *kD);
int16_t Controller_AutoTune(float target_speed, int64_t time, float dt);
