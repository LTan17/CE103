#include <stdio.h>
#include "TFT_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

void TFT_gpio_init()
{
    gpio_reset_pin(TFT_RES);
    gpio_set_direction(TFT_RES, GPIO_MODE_OUTPUT);

    gpio_reset_pin(TFT_DC);
    gpio_set_direction(TFT_DC, GPIO_MODE_OUTPUT);

    //gpio_reset_pin(TFT_BL);
    //gpio_set_direction(TFT_BL, GPIO_MODE_OUTPUT);

    gpio_reset_pin(TFT_CS);
    gpio_set_direction(TFT_CS, GPIO_MODE_OUTPUT);
}
