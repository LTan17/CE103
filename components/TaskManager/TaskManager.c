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
#include "button.h"

#define CONTROL_TIMER_ALARM_VALUE 50000

TaskHandle_t Controller_TaskHandle = NULL;
TaskHandle_t TFT_TaskHandle = NULL;

SemaphoreHandle_t btn_semaphore;

float target_speed = 50.0;

void IRAM_ATTR isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(btn_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

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
    float dt = 0;
    static int64_t last_time = 0;
    static int64_t current_time = 0;
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
        Controller_Update(target_speed, dt);
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
        // printf("Updating display...\n");
        Controller_GetParams(&display_current_speed, &display_target_speed, &kP, &kI, &kD);
        // printf("Current Speed: %.2f, Target Speed: %.2f, kP: %.3f, kI: %.3f, kD: %.3f\n", display_current_speed, display_target_speed, kP, kI, kD);
        update_display(display_current_speed, display_target_speed, kP, kI, kD);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void button_task(void *arg)
{
    Button btn1;
    Button btn2;

    float setpoint = 0;

    uint32_t last_hold_1 = 0;
    uint32_t last_hold_2 = 0;

    button_init(&btn1, BTN1);
    button_init(&btn2, BTN2);
    while (1)
    {
        // chờ timer hoặc interrupt
        xSemaphoreTake(btn_semaphore, pdMS_TO_TICKS(50));

        // lấy thời gian hiện tại
        uint32_t now = xTaskGetTickCount();

        // FSM xử lý, update từng nút
        BtnEvent e1 = process_button(&btn1, gpio_get_level(BTN1), now);
        BtnEvent e2 = process_button(&btn2, gpio_get_level(BTN2), now);

        if (e1 == BTN_SINGLE)
        {
            // printf("BTN1 SINGLE\n");
            target_speed += 20;
        }
        if (e2 == BTN_SINGLE)
        {
            // printf("BTN2 SINGLE\n");
            target_speed -= 20;
        }
        if (e1 == BTN_DOUBLE || e2 == BTN_DOUBLE)
        {
            // printf("BTN DOUBLE\n");
            target_speed = 150;
        }
        if (btn1.state == HOLD)
        {
            // printf("BTN1 HOLD\n");
            // target_speed += 20;
            last_hold_1 = now;
        }
        if (btn2.state == HOLD)
        {
            // printf("BTN2 HOLD\n");
            // target_speed -= 20;
            last_hold_2 = now;
        }

        // if (setpoint > 100)
        //     setpoint = 100;
        // if (setpoint < 0)
        //     setpoint = 0;
        // static float last_sp = -1;
        // if (last_sp != setpoint)
        // {
        //     printf("Setpoint = %.2f\n", setpoint);
        //     last_sp = setpoint;
        // }
    }
}

void CreateTasks(void)
{
    btn_semaphore = xSemaphoreCreateBinary();
    gpio_set_direction(26, GPIO_MODE_INPUT);
    gpio_set_direction(27, GPIO_MODE_INPUT);
    gpio_set_pull_mode(26, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(27, GPIO_PULLUP_ONLY);

    gpio_set_intr_type(26, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(27, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(26, isr_handler, NULL);
    gpio_isr_handler_add(27, isr_handler, NULL);
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);

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