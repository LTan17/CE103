#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "TaskManager.h"
#include "driver/gpio.h"
#include "Motor.h"

void app_main(void)
{
    CreateTasks();
}
