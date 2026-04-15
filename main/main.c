#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

// semaphore
SemaphoreHandle_t btn_semaphore;

// ISR
void IRAM_ATTR isr_handler(void *arg){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(btn_semaphore, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken){
        portYIELD_FROM_ISR();
    }
}

// task từ file khác
extern void button_task(void *arg);

void app_main(){

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
}