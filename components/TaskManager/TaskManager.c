#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "TaskManager.h"
#include "Controller.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "esp_err.h"
#include "Encoder.h"

#define CONTROL_TIMER_ALARM_VALUE 50000

TaskHandle_t Controller_TaskHandle = NULL;

bool control_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    xTaskNotifyFromISR(Controller_TaskHandle, 0, eNoAction, NULL);
    return true;
}

static void control_timer_init(void)
{
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(gptimer_new_timer(&timer_config, &gptimer));
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = CONTROL_TIMER_ALARM_VALUE,
        .flags.auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = control_timer_callback,
    };
    ESP_ERROR_CHECK_WITHOUT_ABORT(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gptimer_enable(gptimer));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gptimer_start(gptimer));
}

void Controller_Task(void *pvParameters)
{
    int sp[] = {10, 60, 90, 40, 100};
    uint16_t cnt = 0;
    const float target_speed = 50.0;
    float dt = 0;
    static int64_t last_time = 0;
    static int64_t current_time = 0;
    int current_speed = -10;
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        cnt = (cnt + 1) % 500;
        current_time = esp_timer_get_time();
        dt = (current_time - last_time) / 1000000.0;
        last_time = current_time;
        // Controller_Update(sp[cnt / 100], dt);
        Controller_Update(70, dt);
    }
}

void CreateTasks(void)
{
    Controller_Init();
    control_timer_init();
    xTaskCreatePinnedToCore(
        Controller_Task,        /* Task function. */
        "ControllerTask",       /* name of task. */
        4096,                   /* Stack size of task */
        NULL,                   /* parameter of the task */
        1,                      /* priority of the task */
        &Controller_TaskHandle, /* Task handle to keep track of created task */
        0);                     /* pin task to core 0 */
}