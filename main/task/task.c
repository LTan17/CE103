#include "drivers/button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <stdio.h>


Button btn1;
Button btn2;

float setpoint = 0;

uint32_t last_hold_1 = 0;
uint32_t last_hold_2 = 0;

extern SemaphoreHandle_t btn_semaphore;

void button_task(void *arg){
    button_init(&btn1, BTN1);
    button_init(&btn2, BTN2);
    while(1){
        // chờ timer hoặc interrupt
        xSemaphoreTake(btn_semaphore, pdMS_TO_TICKS(50));

        // lấy thời gian hiện tại 
        uint32_t now = xTaskGetTickCount();

        // FSM xử lý, update từng nút
        BtnEvent e1 = process_button(&btn1, gpio_get_level(BTN1), now);
        BtnEvent e2 = process_button(&btn2, gpio_get_level(BTN2), now);

        if(e1 == BTN_SINGLE){
            setpoint += 10;
        }
        if(e2 == BTN_SINGLE){
            setpoint -= 10;
        }
        if(e1 == BTN_DOUBLE || e2 == BTN_DOUBLE){
            setpoint = 0;
        }
        if(btn1.state == HOLD){
            if(now - last_hold_1 > pdMS_TO_TICKS(100)){
                setpoint += 5;
                last_hold_1 = now;
            }
        }
        if(btn2.state == HOLD){
            if(now - last_hold_2 > pdMS_TO_TICKS(100)){
                setpoint -= 5;
                last_hold_2 = now;
            }
        }

    if(setpoint > 100) setpoint = 100;
    if(setpoint < 0) setpoint = 0;
    static float last_sp = -1;
    if(last_sp != setpoint){
         printf("Setpoint = %.2f\n", setpoint);
        last_sp = setpoint;
        }
    }
}