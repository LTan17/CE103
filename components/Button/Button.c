#include "button.h"
#include "freertos/FreeRTOS.h"

void button_init(Button *btn, int pin)
{
    btn->pin = pin;
    btn->stable = 1;
    btn->last_raw = 1;
    btn->last_change_time = 0;
    btn->press_time = 0;
    btn->release_time = 0;
    btn->state = IDLE;
}
BtnEvent process_button(Button *btn, int raw, uint32_t now)
{
    // debounce
    if (raw != btn->last_raw)
    {
        btn->last_change_time = now;
        btn->last_raw = raw;
    }
    if ((now - btn->last_change_time) > pdMS_TO_TICKS(DEBOUNCE_MS))
    {
        if (btn->stable != raw)
        {
            btn->stable = raw;

            // kiểm tra giống code polling
            if (btn->stable == 0)   // nút được nhấn
            {
                btn->press_time = now;      // lưu thời điểm nhấn
                btn->state = PRESSED;       // gán state là được nhấn
            }
            else                            // khi thả ra
            {
                if (btn->state == HOLD)     // kiểm tra đang trong state HOLD không? lỡ trường hợp nhấn chưa đủ lâu ở để vào HOLD
                {
                    btn->state = IDLE;      // thả ra thì đưa trạng thái về IDLE
                    return BTN_NONE;        // trả về giá trị là đang không nhấn
                }

                if (btn->state == PRESSED)  // nếu thả ra mà ở trạng thái nhấn   
                {
                    btn->release_time = now;// gán thời gian
                    btn->state = IDLE;
                    return BTN_SINGLE;      // nếu không lớn hơn thì trả về SINGLE
                }
            }
        }
    }
    // kiểm tra nhấn
    if (btn->state == PRESSED)
    {
        if ((now - btn->press_time) > pdMS_TO_TICKS(HOLD_MS))   // nếu biến now (biến đém thời gian nhấn nút) > HOLD_MS 
        {
            btn->state = HOLD;          // trả về trạng thái là HOLD
            return BTN_HOLD;
        }
    }
    return BTN_NONE;
}
