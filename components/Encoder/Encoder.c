#include <stdio.h>
#include "Encoder.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"

#define MAX_GLITCH_NS 1000

#define GPIO_PHASE_A 14
#define GPIO_PHASE_B 13

static const char *TAG = "Encoder";

pcnt_unit_handle_t pcnt_unit = NULL;

esp_err_t Encoder_Init(void)
{
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_COUNT,
        .low_limit = -ENCODER_COUNT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = MAX_GLITCH_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = GPIO_PHASE_A,
        .level_gpio_num = GPIO_PHASE_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = GPIO_PHASE_B,
        .level_gpio_num = GPIO_PHASE_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK_WITHOUT_ABORT(pcnt_unit_start(pcnt_unit));
    ESP_LOGI(TAG, "Encoder initialized successfully");
    return ESP_OK;
}

void Encoder_Read(int *count)
{
    pcnt_unit_get_count(pcnt_unit, count);
}