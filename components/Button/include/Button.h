#pragma once
#include <stdint.h>

#define BTN1 26
#define BTN2 27

#define DEBOUNCE_MS 20
#define DOUBLE_MS 300
#define HOLD_MS 800

typedef enum
{
    BTN_NONE = 0,
    BTN_SINGLE,
    BTN_DOUBLE,
    BTN_HOLD
} BtnEvent;

typedef enum
{
    IDLE,
    PRESSED,
    WAIT_DOUBLE,
    HOLD
} ButtonState;

typedef struct
{
    int pin;
    int stable;
    int last_raw;
    uint32_t last_change_time;
    uint32_t press_time;
    uint32_t release_time;
    ButtonState state;
} Button;
BtnEvent process_button(Button *btn, int raw, uint32_t now);
void button_init(Button *btn, int pin);