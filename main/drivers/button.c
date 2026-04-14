#include "button.h"
#include "freertos/FreeRTOS.h"

void button_init(Button *btn, int pin){
    btn->pin = pin;
    btn->stable = 1;
    btn->last_raw = 1;
    btn->last_change_time = 0;
    btn->press_time = 0;
    btn->release_time = 0;
    btn->state = IDLE;
}

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
