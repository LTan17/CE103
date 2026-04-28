#include "drivers/button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <stdio.h>


extern SemaphoreHandle_t btn_semaphore;
/* void button_task(void *arg){
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
            if (mode == MODE_SPEED) temp_speed += STEP_SPEED;
            else if (mode == MODE_KP) temp_kp += STEP_KP;
            else if (mode == MODE_KI) temp_ki += STEP_KI;
            else if (mode == MODE_KD) temp_kd += STEP_KD;
            continue
        }

        if(e2 == BTN_SINGLE){
            if (mode == MODE_SPEED) temp_speed -= STEP_SPEED;
            else if (mode == MODE_KP) temp_kp -= STEP_KP;
            else if (mode == MODE_KI) temp_ki -= STEP_KI;
            else if (mode == MODE_KD) temp_kd -= STEP_KD;
            continue;
        }
        if(e3 == BTN_SINGLE){
            mode = (mode + 1) % MODE_COUNT;
        }
        if(e1 == BTN_HOLD){
            uart_mode = true;
        }
        if(e2 == BTN_HOLD){
            autotune_mode = true;
        }
        if(e3 == BTN_HOLD && btn3_trigger == false){
            btn3_trigger = true;
            if (mode == MODE_SPEED) {
                speed = temp_speed;
            } 
            else if (mode == MODE_KP) {
                pid.Kp = temp_kp;
            } 
            else if (mode == MODE_KI) {
                pid.Ki = temp_ki;
            } 
            else if (mode == MODE_KD) {
                pid.Kd = temp_kd;
            }
        }
        if (e3 == BTN_NONE){
            btn3_trigger = false;       
        }

    if(setpoint > 200) speed = 200;
    if(setpoint < 50) speed = 50;
    static float last_sp = -1;
    if(last_sp != speed){
        printf("Setpoint = %.2f\n", setpoint);
        last_sp = speed;
        }
    }
}

void uart_parse(char *cmd)
{
    float val;

    if (sscanf(cmd, "SET %f", &val) == 1) {
        setpoint = val;
        printf("SET = %.1f\n", speed);
        return;
    }

    if (sscanf(cmd, "KP %f", &val) == 1) {
        pid.Kp = val;
        printf("Kp = %.2f\n", pid.Kp);
        return;
    }

    if (sscanf(cmd, "KI %f", &val) == 1) {
        pid.Ki = val;
        printf("Ki = %.2f\n", pid.Ki);
        return;
    }

    if (sscanf(cmd, "KD %f", &val) == 1) {
        pid.Kd = val;
        printf("Kd = %.2f\n", pid.Kd);
        return;
    }

}

void uart_task(void *arg)
{
    while (1)
    {
        if (uart_mode)   // chỉ chạy khi bật UART mode
        {
            while (Serial.available())
            {
                char c = Serial.read();

                if (c == '\n' || c == '\r')
                {
                    rx_buf[idx] = '\0';

                    if (idx > 0) {
                        uart_parse(rx_buf);
                        idx = 0;
                    }
                }
                else
                {
                    if (idx < UART_BUF_SIZE - 1)
                        rx_buf[idx++] = c;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
*/