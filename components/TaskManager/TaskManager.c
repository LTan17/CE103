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
#include "driver/uart.h"
#include "string.h"
#define CONTROL_TIMER_ALARM_VALUE 50000

#define UART_PORT       UART_NUM_0
#define UART_BAUD_RATE  115200
#define UART_RX_PIN     UART_PIN_NO_CHANGE
#define UART_TX_PIN     UART_PIN_NO_CHANGE

TaskHandle_t Controller_TaskHandle = NULL;
TaskHandle_t TFT_TaskHandle = NULL;

SemaphoreHandle_t btn_semaphore;
char rx_buf[UART_BUF_SIZE];
int idx = 0;

float temp_speed = 0.0f;
float temp_kp = 5.0f;
float temp_ki = 17.8f;
float temp_kd = 0.3f;
float setpoint = 0.0f;

bool btn1_trigger = false;
bool btn2_trigger = false;
bool btn3_trigger = false;
Button btn1, btn2, btn3;
Control_mode mode = SPEED;

bool uart_mode = false;
bool autotune_mode = false;
//float target_speed = 50.0;

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
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
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
        Controller_GetParams(&cs, &ts, &kp, &ki, &kd);
        snprintf(response, sizeof(response),
                 "SPD:%.1f|SET:%.1f|KP:%.3f|KI:%.3f|KD:%.3f\n",
                 cs, ts, kp, ki, kd);
        uart_write_bytes(UART_PORT, response, strlen(response));
        return;
    }

    if (strlen(cmd) < 3 || cmd[1] != ':')
    {
        uart_write_bytes(UART_PORT, "ERR:FORMAT\n", 11);
        return;
    }

    float value = atof(&cmd[2]);
    char type   = cmd[0];

    switch (type)
    {
        case 'S':
            if (value < 0 || value > 333.0f) { uart_write_bytes(UART_PORT, "ERR:RANGE\n", 10); return; }
            setpoint   = value;
            temp_speed = value;
            snprintf(response, sizeof(response), "OK:SPD=%.1f\n", setpoint);
            break;

        case 'P':
            if (value < 0) { uart_write_bytes(UART_PORT, "ERR:RANGE\n", 10); return; }
            pid.Kp  = value;
            temp_kp = value;
            snprintf(response, sizeof(response), "OK:KP=%.3f\n", pid.Kp);
            break;

        case 'I':
            if (value < 0) { uart_write_bytes(UART_PORT, "ERR:RANGE\n", 10); return; }
            pid.Ki  = value;
            temp_ki = value;
            snprintf(response, sizeof(response), "OK:KI=%.3f\n", pid.Ki);
            break;

        case 'D':
            if (value < 0) { uart_write_bytes(UART_PORT, "ERR:RANGE\n", 10); return; }
            pid.Kd  = value;
            temp_kd = value;
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
        if (len <= 0) continue;

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
        if(autotune_mode){
            if(Controller_AutoTune(setpoint,current_time,dt) == 1){
                autotune_mode = false;
                // float current_speed; có thể để debug
                temp_kp = pid.Kp;
                temp_ki = pid.Ki;
                temp_kd = pid.Kd;
            }
        }
        else{
            Controller_Update(setpoint, dt);
        }
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

void limit_temp_values(void)
{
    if (temp_speed > 200.0f) temp_speed = 200.0f;
    if (temp_speed < 50.0f)   temp_speed = 50.0f;
    if (temp_kp < 0.0f)      temp_kp = 0.0f;
    if (temp_ki < 0.0f)      temp_ki = 0.0f;
    if (temp_kd < 0.0f)      temp_kd = 0.0f;
}

void button_task(void *arg){
    button_init(&btn1, BTN1);
    button_init(&btn2, BTN2);
    button_init(&btn3, BTN3);
    while(1){
        xSemaphoreTake(btn_semaphore, pdMS_TO_TICKS(50));

        uint32_t now = xTaskGetTickCount();

        BtnEvent e1 = process_button(&btn1, gpio_get_level(BTN1), now);
        BtnEvent e2 = process_button(&btn2, gpio_get_level(BTN2), now);
        BtnEvent e3 = process_button(&btn3, gpio_get_level(BTN3), now);

        if(e1 == BTN_SINGLE){
            if (mode == SPEED) temp_speed += STEP_SPEED;
            else if (mode == KP) temp_kp += STEP_KP;
            else if (mode == KI) temp_ki += STEP_KI;
            else if (mode == KD) temp_kd += STEP_KD;
            limit_temp_values();
        }

        if(e2 == BTN_SINGLE){
            if (mode == SPEED) temp_speed -= STEP_SPEED;
            else if (mode == KP) temp_kp -= STEP_KP;
            else if (mode == KI) temp_ki -= STEP_KI;
            else if (mode == KD) temp_kd -= STEP_KD;
            limit_temp_values();
        }
        if(e3 == BTN_SINGLE){
            mode = (mode + 1) % MODE_COUNT;
        }
        if(e1 == BTN_HOLD && !btn1_trigger){
            btn1_trigger = true;
            uart_mode = !uart_mode;
        }
        if(e1 == BTN_NONE) btn1_trigger = false;

        if(e2 == BTN_HOLD && !btn2_trigger){
            btn2_trigger = true;
            autotune_mode = !autotune_mode;
        }
        if(e2 == BTN_NONE) btn2_trigger = false;

        if(e3 == BTN_HOLD && !btn3_trigger){
            btn3_trigger = true;
            if (mode == SPEED) {
                setpoint = temp_speed;
            } 
            else if (mode == KP) {
                pid.Kp = temp_kp;
            } 
            else if (mode == KI) {
                pid.Ki = temp_ki;
            } 
            else if (mode == KD) {
                pid.Kd = temp_kd;
            }
        }
        if (e3 == BTN_NONE){
            btn3_trigger = false;       
        }
    }
}

/*
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
*/
void CreateTasks(void)
{
    btn_semaphore = xSemaphoreCreateBinary();
    gpio_set_direction(26, GPIO_MODE_INPUT);
    gpio_set_direction(27, GPIO_MODE_INPUT);
    gpio_set_direction(28, GPIO_MODE_INPUT);
    gpio_set_pull_mode(26, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(27, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(28, GPIO_PULLUP_ONLY);

    gpio_set_intr_type(26, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(27, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(28, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(26, isr_handler, NULL);
    gpio_isr_handler_add(27, isr_handler, NULL);
    gpio_isr_handler_add(28, isr_handler, NULL);
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
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
}