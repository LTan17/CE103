#pragma once
#include <stdint.h>

#define BTN1 25
#define BTN2 26
#define BTN3 27

#define DEBOUNCE_MS 0
#define HOLD_MS 1000

typedef enum
{
    BTN_NONE = 0,
    BTN_SINGLE,
    BTN_HOLD
} BtnEvent;

typedef enum
{
    IDLE,
    PRESSED,
    HOLD
} ButtonState;

typedef enum
{
    SPEED,
    KP,
    KI,
    KD,
    MODE_COUNT
} Control_mode;

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
void button_startup();
