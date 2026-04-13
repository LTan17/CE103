#pragma once
#include "esp_err.h"

esp_err_t Controller_Init(void);
void Controller_Update(float speed, float dt);