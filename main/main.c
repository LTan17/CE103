#include <stdio.h>
#include <string.h>
#include "TFT.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "myuit.h" 

float current_speed = 45.5;
float target_speed = 50.0;
float kP = 1.25;
float kI = 0.50;
float kD = 0.10;

void update_display()
{
    char buf[64]; 
    snprintf(buf, sizeof(buf), "Toc do hien tai: %.1f  ", current_speed);
    TFT_WriteString_Transparent(10, 40, buf, Font_11x18, TFT_BLACK, myuit); 

    snprintf(buf, sizeof(buf), "Toc do yeu cau: %.1f  ", target_speed);
    TFT_WriteString_Transparent(10, 70, buf, Font_11x18, TFT_RED, myuit);

    snprintf(buf, sizeof(buf), "kP: %.2f  ", kP); 
    TFT_WriteString_Transparent(10, 110, buf, Font_11x18, TFT_BLUE, myuit);

    snprintf(buf, sizeof(buf), "kI: %.2f  ", kI);
    TFT_WriteString_Transparent(10, 140, buf, Font_11x18, TFT_BLUE, myuit);

    snprintf(buf, sizeof(buf), "kD: %.2f  ", kD);
    TFT_WriteString_Transparent(10, 170, buf, Font_11x18, TFT_BLUE, myuit);
}

void app_main(void)
{
    TFT_SPI_init();
    TFT_gpio_init();
    TFT_Init();
    // Tắt chế độ Invert để màu sắc chuẩn xác
    TFT_InvertColors(false); 
    // Phủ trắng màn hình và vẽ ảnh đúng 1 lần duy nhất bằng hàm gốc
    TFT_DrawImage_Standard(0, 0, 320, 240, myuit);
    while (1)
    {
        update_display(); 
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}