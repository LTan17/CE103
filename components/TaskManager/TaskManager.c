#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "TaskManager.h"
#include "Controller.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "esp_err.h"
#include "Encoder.h"
#include "TFT.h"

#define CONTROL_TIMER_ALARM_VALUE 20000

TaskHandle_t Controller_TaskHandle = NULL;
TaskHandle_t TFT_TaskHandle = NULL;

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
    int sp[] = {50, 100, 150, 70, 200};
    uint16_t cnt = 0;
    const float target_speed = 50.0;
    float dt = 0;
    static int64_t last_time = 0;
    static int64_t current_time = 0;
    int current_speed = -10;
    Controller_Init();
    control_timer_init();
    // while (1)
    // {
    //     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    //     current_time = esp_timer_get_time();
    //     dt = (current_time - last_time) / 1000000.0;
    //     last_time = current_time;
    //     if (Controller_AutoTune(220, current_time, dt) == 1)
    //     {
    //         continue;
    //     }
    // }
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        cnt = (cnt + 1) % 5000;
        current_time = esp_timer_get_time();
        dt = (current_time - last_time) / 1000000.0;
        last_time = current_time;
        Controller_Update(sp[cnt / 1000], dt);
    }
}

void TFT_task(void *pvParameters)
{
    TFT_SPI_init();
    TFT_gpio_init();
    TFT_Init();
    TFT_InvertColors(false);
    float display_current_speed, display_target_speed, kP, kI, kD;
    TFT_FillScreen(TFT_WHITE);
    vTaskDelay(pdMS_TO_TICKS(2000));
    while (1)
    {
        printf("Updating display...\n");
        Controller_GetParams(&display_current_speed, &display_target_speed, &kP, &kI, &kD);
        printf("Current Speed: %.2f, Target Speed: %.2f, kP: %.3f, kI: %.3f, kD: %.3f\n", display_current_speed, display_target_speed, kP, kI, kD);
        update_display(display_current_speed, display_target_speed, kP, kI, kD);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void CreateTasks(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000)); // Delay to ensure all peripherals are initialized
    xTaskCreatePinnedToCore(
        Controller_Task,        /* Task function. */
        "ControllerTask",       /* name of task. */
        4096,                   /* Stack size of task */
        NULL,                   /* parameter of the task */
        1,                      /* priority of the task */
        &Controller_TaskHandle, /* Task handle to keep track of created task */
        0);                     /* pin task to core 0 */

    xTaskCreatePinnedToCore(
        TFT_task,        /* Task function. */
        "TFTTask",       /* name of task. */
        4096,            /* Stack size of task */
        NULL,            /* parameter of the task */
        1,               /* priority of the task */
        &TFT_TaskHandle, /* Task handle to keep track of created task */
        1);              /* pin task to core 0 */
}