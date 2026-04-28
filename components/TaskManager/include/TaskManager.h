#pragma once
#include <stdbool.h>
#include "Controller.h"
#include "Button.h"

extern bool btn3_trigger;

extern Button btn1;
extern Button btn2;
extern Button btn3; 

#define UART_BUF_SIZE 64
extern char rx_buf[UART_BUF_SIZE];
extern int idx;

#define STEP_SPEED 10.0f
#define STEP_KP    0.1f
#define STEP_KI    0.1f
#define STEP_KD    0.01f

extern float temp_speed;
extern float temp_kp;
extern float temp_ki;
extern float temp_kd;

extern float setpoint;

void CreateTasks(void);
