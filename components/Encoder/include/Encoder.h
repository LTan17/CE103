#pragma once
#include "esp_err.h"

#define ENCODER_RESOLUTION 11
#define ENCODER_RATIO 30

#define ENCODER_PPR (ENCODER_RESOLUTION * ENCODER_RATIO) * 4

#define ENCODER_COUNT 30000

esp_err_t Encoder_Init(void);
void Encoder_Read(int *count);