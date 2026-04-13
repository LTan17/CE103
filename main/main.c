    #include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdint.h>

#define LED1 2
#define LED2 15
#define BTN1 4
#define BTN2 5

#define DEBOUNCE_MS 20
#define DOUBLE_MS   300
#define HOLD_MS     800

static float setpoint = 0;
uint32_t last_hold_1 = 0;
uint32_t last_hold_2 = 0;

typedef enum {
    BTN_NONE = 0,
    BTN_SINGLE,
    BTN_DOUBLE,
    BTN_HOLD
} BtnEvent;

typedef enum {
    MODE_OFF,
    MODE_BLINK,
    MODE_ON
} Mode;

volatile Mode current_mode = MODE_OFF; // biến có thể thay đổi bởi hoạt động khác, khai báo để tránh việc mode bị thay đổi mà k nhận ra


SemaphoreHandle_t btn_semaphore = NULL; // thay cho while(1), đồng bộ giữa isr và task, báo hiệu interrupt 


void IRAM_ATTR isr_handler(void *arg){      // ràng buộc hàm ISR vào trong RAM 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;                          //cờ ngủ, RTOS sẽ gọi đậy lúc có task
    xSemaphoreGiveFromISR(btn_semaphore, &xHigherPriorityTaskWoken);        // gọi dậy nè
    if(xHigherPriorityTaskWoken){                                           // xác định sem ở trạng thái nào
        portYIELD_FROM_ISR();
    }
}

typedef enum {
    IDLE,
    PRESSED,
    WAIT_DOUBLE,
    HOLD
} ButtonState;

typedef struct {
    int pin;
    int stable;
    int last_raw;
    uint32_t last_change_time;
    uint32_t press_time;
    uint32_t release_time;
    ButtonState state;
} Button;

Button btn1 = {1,1,0,0,0,0,0};          // khởi tạo
Button btn2 = {1,1,0,0,0,0,0};


BtnEvent process_button(Button *btn, int raw, uint32_t now){
    // debounce
    if(raw != btn->last_raw){
        btn->last_change_time = now;
        btn->last_raw = raw;
    }
    if((now - btn->last_change_time) > pdMS_TO_TICKS(DEBOUNCE_MS)){
        if(btn->stable != raw){
            btn->stable = raw;

            // kiểm tra giống code polling
            if(btn->stable == 0){
                btn->press_time = now;

                if(btn->state == WAIT_DOUBLE){
                    btn->state = IDLE;
                    return BTN_DOUBLE;
                }
                btn->state = PRESSED;    
            }
  
            else {
                if(btn->state == HOLD){
                    btn->state = IDLE;
                    return BTN_NONE;
                }

                if(btn->state == PRESSED){
                    btn->release_time = now;
                    btn->state = WAIT_DOUBLE;
                }
            }
        }
    }

    // ===== HOLD =====
    if(btn->state == PRESSED){
        if((now - btn->press_time) > pdMS_TO_TICKS(HOLD_MS)){
            btn->state = HOLD;
            return BTN_HOLD;
        }
    }

    // ===== SINGLE =====
    if(btn->state == WAIT_DOUBLE){
        if((now - btn->release_time) > pdMS_TO_TICKS(DOUBLE_MS)){
            btn->state = IDLE;
            return BTN_SINGLE;
        }
    }

    return BTN_NONE;
}


void button_task(void *arg){

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
        if(btn1.state == BTN_HOLD){
            if(now - last_hold_1 > pdMS_TO_TICKS(200)){
                setpoint += 5;
                last_hold_1 = now;
            }
        }
        if(btn2.state == BTN_HOLD){
            if(now - last_hold_2 > pdMS_TO_TICKS(200)){
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
/*
void led_task(void *arg){
    uint32_t last = 0;
    int step = 0;

    while(1){
        uint32_t now = xTaskGetTickCount();

        if(current_mode == MODE_ON){
            gpio_set_level(LED1, 1);
            gpio_set_level(LED2, 1);
        }
        else if(current_mode == MODE_OFF){
            gpio_set_level(LED1, 0);
            gpio_set_level(LED2, 0);
        }
        else if(current_mode == MODE_BLINK){
            if(now - last >= pdMS_TO_TICKS(200)){
                step = !step;
                gpio_set_level(LED1, step);
                gpio_set_level(LED2, !step);
                last = now;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
*/

void app_main(){

    // Tạo semaphore
    btn_semaphore = xSemaphoreCreateBinary();

    gpio_reset_pin(LED1);
    gpio_reset_pin(LED2);
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);

    gpio_reset_pin(BTN1);
    gpio_reset_pin(BTN2);
    gpio_set_direction(BTN1, GPIO_MODE_INPUT);
    gpio_set_direction(BTN2, GPIO_MODE_INPUT);

    gpio_set_pull_mode(BTN1, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BTN2, GPIO_PULLUP_ONLY);

    // INTERRUPT - Bắt cả 2 cạnh để phản ứng nhanh
    gpio_set_intr_type(BTN1, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(BTN2, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN1, isr_handler, NULL);
    gpio_isr_handler_add(BTN2, isr_handler, NULL);

    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
    //xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
}
