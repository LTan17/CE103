#pragma once
#include "esp_err.h"

esp_err_t Controller_Init(void);
void Controller_Update(float speed, float dt);
void Controller_GetParams(float *current_speed, float *kP, float *kI, float *kD);
int16_t Controller_AutoTune(int tune_pwm, float dt);
