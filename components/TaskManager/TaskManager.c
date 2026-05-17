#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "TaskManager.h"
#include "Controller.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "esp_err.h"
#include "Encoder.h"
#include "TFT.h"
#include "button.h"
#include "driver/uart.h"
#include "string.h"
#define CONTROL_TIMER_ALARM_VALUE 50000

#define UART_PORT UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_RX_PIN UART_PIN_NO_CHANGE
#define UART_TX_PIN UART_PIN_NO_CHANGE

TaskHandle_t Controller_TaskHandle = NULL;
TaskHandle_t TFT_TaskHandle = NULL;

SemaphoreHandle_t btn_semaphore;
SemaphoreHandle_t param_mutex;

char rx_buf[UART_BUF_SIZE];
int idx = 0;

float temp_speed = 50.0f;
pid_t temp_pid = {.Kp = 0.0f, .Ki = 0.0f, .Kd = 0.0f, .prev_error = 0, .I = 0, .I_output_limit = 0.0f};
float setpoint = 100.0f;

bool btn1_trigger = false;
bool btn2_trigger = false;
bool btn3_trigger = false;
Button btn1, btn2, btn3;
Control_mode mode = SPEED;

bool uart_mode = false;
bool autotune_mode = false;

void IRAM_ATTR isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(btn_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_PORT, 256, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void uart_process_command(const char *cmd)
{
    char response[64];

    if (strncmp(cmd, "GET", 3) == 0)
    {
        float cs, ts, kp, ki, kd;
        Controller_GetSpeed(&cs);
        // snprintf(response, sizeof(response),
        //          "SPD:%.1f|KP:%.3f|KI:%.3f|KD:%.3f\n",
        //          cs, kp, ki, kd);
        // uart_write_bytes(UART_PORT, response, strlen(response));
        return;
    }

    if (strlen(cmd) < 3 || cmd[1] != ':')
    {
        uart_write_bytes(UART_PORT, "ERR:FORMAT\n", 11);
        return;
    }

    float value = atof(&cmd[2]);
    char type = cmd[0];

    switch (type)
    {
    case 'S':
        if (value < 0 || value > 333.0f)
        {
            uart_write_bytes(UART_PORT, "ERR:RANGE\n", 10);
            return;
        }
        setpoint = value;
        temp_speed = value;
        snprintf(response, sizeof(response), "OK:SPD=%.1f\n", setpoint);
        break;

    case 'P':
        if (value < 0)
        {
            uart_write_bytes(UART_PORT, "ERR:RANGE\n", 10);
            return;
        }
        pid.Kp = value;
        temp_pid.Kp = value;
        snprintf(response, sizeof(response), "OK:KP=%.3f\n", pid.Kp);
        break;

    case 'I':
        if (value < 0)
        {
            uart_write_bytes(UART_PORT, "ERR:RANGE\n", 10);
            return;
        }
        pid.Ki = value;
        temp_pid.Ki = value;
        snprintf(response, sizeof(response), "OK:KI=%.3f\n", pid.Ki);
        break;

    case 'D':
        if (value < 0)
        {
            uart_write_bytes(UART_PORT, "ERR:RANGE\n", 10);
            return;
        }
        pid.Kd = value;
        temp_pid.Kd = value;
        snprintf(response, sizeof(response), "OK:KD=%.3f\n", pid.Kd);
        break;

    default:
        uart_write_bytes(UART_PORT, "ERR:CMD\n", 8);
        return;
    }

    uart_write_bytes(UART_PORT, response, strlen(response));
}

void uart_task(void *pvParameters)
{

    uart_init();
    uart_write_bytes(UART_PORT, "UART_READY\n", 11);

    uint8_t byte;
    while (1)
    {
        // Khi không ở uart_mode thì vẫn drain buffer để tránh tràn
        if (!uart_mode)
        {
            uart_flush_input(UART_PORT);
            idx = 0;
            memset(rx_buf, 0, UART_BUF_SIZE);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int len = uart_read_bytes(UART_PORT, &byte, 1, pdMS_TO_TICKS(20));
        if (len <= 0)
            continue;

        if (byte == '\n' || byte == '\r')
        {
            if (idx > 0)
            {
                rx_buf[idx] = '\0';
                uart_process_command(rx_buf);
                idx = 0;
                memset(rx_buf, 0, UART_BUF_SIZE);
            }
        }
        else if (idx < UART_BUF_SIZE - 1)
        {
            rx_buf[idx++] = (char)byte;
        }
        else
        {
            // Buffer tràn, reset
            idx = 0;
            memset(rx_buf, 0, UART_BUF_SIZE);
            uart_write_bytes(UART_PORT, "ERR:OVERFLOW\n", 13);
        }
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
    float dt = 0;
    static int64_t last_time = 0;
    static int64_t current_time = 0;
    Controller_Init();
    control_timer_init();
    Controller_GetPIDParams(&temp_pid);
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        current_time = esp_timer_get_time();
        dt = (current_time - last_time) / 1000000.0;
        last_time = current_time;
        if (autotune_mode)
        {
            if (Controller_AutoTune(700, dt) == 1)
            {
                autotune_mode = false;
                xSemaphoreTake(param_mutex, portMAX_DELAY);
                Controller_GetPIDParams(&temp_pid);
                xSemaphoreGive(param_mutex);
            }
        }
        else
        {
            Controller_Update(setpoint, dt);
        }
    }
}

void limit_temp_values(void)
{
    xSemaphoreTake(param_mutex, portMAX_DELAY);
    if (temp_speed > 200.0f)
        temp_speed = 200.0f;
    if (temp_speed < 50.0f)
        temp_speed = 50.0f;
    if (temp_pid.Kp < 0.0f)
        temp_pid.Kp = 0.0f;
    if (temp_pid.Ki < 0.0f)
        temp_pid.Ki = 0.0f;
    if (temp_pid.Kd < 0.0f)
        temp_pid.Kd = 0.0f;
    xSemaphoreGive(param_mutex);
}

void button_task(void *arg)
{
    button_startup();
    button_init(&btn1, BTN1);
    button_init(&btn2, BTN2);
    button_init(&btn3, BTN3);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN1, isr_handler, NULL);
    gpio_isr_handler_add(BTN2, isr_handler, NULL);
    gpio_isr_handler_add(BTN3, isr_handler, NULL);
    while (1)
    {
        xSemaphoreTake(btn_semaphore, pdMS_TO_TICKS(50));

        uint32_t now = xTaskGetTickCount();

        BtnEvent e1 = process_button(&btn1, gpio_get_level(BTN1), now);
        BtnEvent e2 = process_button(&btn2, gpio_get_level(BTN2), now);
        BtnEvent e3 = process_button(&btn3, gpio_get_level(BTN3), now);

        if (e1 == BTN_SINGLE)
        {
            // printf("BTN1: SINGLE\n");
            if (mode == SPEED)
            {
                xSemaphoreTake(param_mutex, portMAX_DELAY);
                temp_speed += STEP_SPEED;
                xSemaphoreGive(param_mutex);
            }

            else if (mode == KP)
            {
                xSemaphoreTake(param_mutex, portMAX_DELAY);
                temp_pid.Kp += STEP_KP;
                xSemaphoreGive(param_mutex);
            }
            else if (mode == KI)
            {
                xSemaphoreTake(param_mutex, portMAX_DELAY);
                temp_pid.Ki += STEP_KI;
                xSemaphoreGive(param_mutex);
            }
            else if (mode == KD)
            {
                xSemaphoreTake(param_mutex, portMAX_DELAY);
                temp_pid.Kd += STEP_KD;
                xSemaphoreGive(param_mutex);
            }
            limit_temp_values();
        }

        if (e2 == BTN_SINGLE)
        {
            // printf("BTN2: SINGLE\n");
            if (mode == SPEED)
            {
                xSemaphoreTake(param_mutex, portMAX_DELAY);
                temp_speed -= STEP_SPEED;
                xSemaphoreGive(param_mutex);
            }
            else if (mode == KP)
            {
                xSemaphoreTake(param_mutex, portMAX_DELAY);
                temp_pid.Kp -= STEP_KP;
                xSemaphoreGive(param_mutex);
            }
            else if (mode == KI)
            {
                xSemaphoreTake(param_mutex, portMAX_DELAY);
                temp_pid.Ki -= STEP_KI;
                xSemaphoreGive(param_mutex);
            }
            else if (mode == KD)
            {
                xSemaphoreTake(param_mutex, portMAX_DELAY);
                temp_pid.Kd -= STEP_KD;
                xSemaphoreGive(param_mutex);
            }
            limit_temp_values();
        }
        if (e3 == BTN_SINGLE)
        {
            printf("BTN3: SINGLE\n");
            mode = (mode + 1) % MODE_COUNT;
        }
        if (e1 == BTN_HOLD && !btn1_trigger)
        {
            printf("BTN1: HOLD\n");
            btn1_trigger = true;
            uart_mode = !uart_mode;
        }
        if (e1 == BTN_NONE)
            btn1_trigger = false;

        if (e2 == BTN_HOLD && !btn2_trigger)
        {
            printf("BTN2: HOLD\n");
            btn2_trigger = true;
            autotune_mode = !autotune_mode;
        }
        if (e2 == BTN_NONE)
            btn2_trigger = false;

        if (e3 == BTN_HOLD && !btn3_trigger)
        {
            printf("BTN3: HOLD\n");
            btn3_trigger = true;
            setpoint = temp_speed;
            Controller_SetPIDParams(&temp_pid);
        }
        if (e3 == BTN_NONE)
        {
            btn3_trigger = false;
        }
    }
}

void TFT_task(void *pvParameters)
{
    TFT_SPI_init();
    TFT_gpio_init();
    TFT_Init();
    TFT_InvertColors(false);
    gpio_reset_pin(GPIO_NUM_19);
    gpio_set_direction(GPIO_NUM_19, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_19, 1);
    float local_current_speed, local_setpoint, local_kP, local_kI, local_kD;
    TFT_FillScreen(TFT_WHITE);
    vTaskDelay(pdMS_TO_TICKS(2000));
    while (1)
    {
        Controller_GetSpeed(&local_current_speed);
        xSemaphoreTake(param_mutex, portMAX_DELAY);
        local_kP = temp_pid.Kp;
        local_kI = temp_pid.Ki;
        local_kD = temp_pid.Kd;
        local_setpoint = temp_speed;
        xSemaphoreGive(param_mutex);
        // printf("Display mode: %d - Speed: %.1f, Setpoint: %.1f, Kp: %.3f, Ki: %.3f, Kd: %.3f\n",
        //        mode, local_current_speed, local_setpoint, local_kP, local_kI, local_kD);
        update_display(local_current_speed, local_setpoint, local_kP, local_kI, local_kD);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void CreateTasks(void)
{
    btn_semaphore = xSemaphoreCreateBinary();
    param_mutex = xSemaphoreCreateMutex();
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
    // xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
}