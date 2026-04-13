#include <stdio.h>
#include <string.h>
#include "TFT.h"

static void TFT_Select()
{
    gpio_set_level(TFT_CS, 0);
}

void TFT_Unselect()
{
    gpio_set_level(TFT_CS, 1);
}

static void TFT_Reset()
{
    gpio_set_level(TFT_RES, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    gpio_set_level(TFT_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
}

static void TFT_WriteCommand(uint8_t cmd)
{
    gpio_set_level(TFT_DC, 0);
    TFT_SPI_transmit(&cmd, 1);
}

static void TFT_WriteData(uint8_t *buff, size_t buff_size)
{
    gpio_set_level(TFT_DC, 1);

    uint8_t *current_buff = buff;
    size_t bytes_remaining = buff_size;

    while (bytes_remaining > 0)
    {
        size_t chunk_size = (bytes_remaining > MAX_SPI_CHUNK_SIZE) ? MAX_SPI_CHUNK_SIZE : bytes_remaining;
        TFT_SPI_transmit(current_buff, chunk_size);

        bytes_remaining -= chunk_size;
        current_buff += chunk_size;
    }
}

static void TFT_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    // column address set
    TFT_WriteCommand(0x2A); // CASET
    {
        uint8_t data[] = {(x0 >> 8) & 0xFF, x0 & 0xFF, (x1 >> 8) & 0xFF, x1 & 0xFF};
        TFT_WriteData(data, sizeof(data));
    }

    // row address set
    TFT_WriteCommand(0x2B); // RASET
    {
        uint8_t data[] = {(y0 >> 8) & 0xFF, y0 & 0xFF, (y1 >> 8) & 0xFF, y1 & 0xFF};
        TFT_WriteData(data, sizeof(data));
    }

    // write to RAM
    TFT_WriteCommand(0x2C); // RAMWR
}

void TFT_Init()
{
    TFT_Select();
    TFT_Reset();

    // SOFTWARE RESET
    TFT_WriteCommand(0x01);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // POWER CONTROL A
    TFT_WriteCommand(0xCB);
    {
        uint8_t data[] = {0x39, 0x2C, 0x00, 0x34, 0x02};
        TFT_WriteData(data, sizeof(data));
    }

    // POWER CONTROL B
    TFT_WriteCommand(0xCF);
    {
        uint8_t data[] = {0x00, 0xC1, 0x30};
        TFT_WriteData(data, sizeof(data));
    }

    // DRIVER TIMING CONTROL A
    TFT_WriteCommand(0xE8);
    {
        uint8_t data[] = {0x85, 0x00, 0x78};
        TFT_WriteData(data, sizeof(data));
    }

    // DRIVER TIMING CONTROL B
    TFT_WriteCommand(0xEA);
    {
        uint8_t data[] = {0x00, 0x00};
        TFT_WriteData(data, sizeof(data));
    }

    // POWER ON SEQUENCE CONTROL
    TFT_WriteCommand(0xED);
    {
        uint8_t data[] = {0x64, 0x03, 0x12, 0x81};
        TFT_WriteData(data, sizeof(data));
    }

    // PUMP RATIO CONTROL
    TFT_WriteCommand(0xF7);
    {
        uint8_t data[] = {0x20};
        TFT_WriteData(data, sizeof(data));
    }

    // POWER CONTROL,VRH[5:0]
    TFT_WriteCommand(0xC0);
    {
        uint8_t data[] = {0x23};
        TFT_WriteData(data, sizeof(data));
    }

    // POWER CONTROL,SAP[2:0];BT[3:0]
    TFT_WriteCommand(0xC1);
    {
        uint8_t data[] = {0x10};
        TFT_WriteData(data, sizeof(data));
    }

    // VCM CONTROL
    TFT_WriteCommand(0xC5);
    {
        uint8_t data[] = {0x3E, 0x28};
        TFT_WriteData(data, sizeof(data));
    }

    // VCM CONTROL 2
    TFT_WriteCommand(0xC7);
    {
        uint8_t data[] = {0x86};
        TFT_WriteData(data, sizeof(data));
    }

    // MEMORY ACCESS CONTROL
    TFT_WriteCommand(0x36);
    {
        uint8_t data[] = {0x48};
        TFT_WriteData(data, sizeof(data));
    }

    // PIXEL FORMAT
    TFT_WriteCommand(0x3A);
    {
        uint8_t data[] = {0x55};
        TFT_WriteData(data, sizeof(data));
    }

    // FRAME RATIO CONTROL, STANDARD RGB COLOR
    TFT_WriteCommand(0xB1);
    {
        uint8_t data[] = {0x00, 0x18};
        TFT_WriteData(data, sizeof(data));
    }

    // DISPLAY FUNCTION CONTROL
    TFT_WriteCommand(0xB6);
    {
        uint8_t data[] = {0x08, 0x82, 0x27};
        TFT_WriteData(data, sizeof(data));
    }

    // 3GAMMA FUNCTION DISABLE
    TFT_WriteCommand(0xF2);
    {
        uint8_t data[] = {0x00};
        TFT_WriteData(data, sizeof(data));
    }

    // GAMMA CURVE SELECTED
    TFT_WriteCommand(0x26);
    {
        uint8_t data[] = {0x01};
        TFT_WriteData(data, sizeof(data));
    }

    // POSITIVE GAMMA CORRECTION
    TFT_WriteCommand(0xE0);
    {
        uint8_t data[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
                          0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
        TFT_WriteData(data, sizeof(data));
    }

    // NEGATIVE GAMMA CORRECTION
    TFT_WriteCommand(0xE1);
    {
        uint8_t data[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
                          0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
        TFT_WriteData(data, sizeof(data));
    }

    // EXIT SLEEP
    TFT_WriteCommand(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));

    // TURN ON DISPLAY
    TFT_WriteCommand(0x29);

    // MADCTL
    TFT_WriteCommand(0x36);
    {
        uint8_t data[] = {TFT_ROTATION};
        TFT_WriteData(data, sizeof(data));
    }

    TFT_Unselect();
}

void TFT_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if ((x >= TFT_WIDTH) || (y >= TFT_HEIGHT))
        return;

    TFT_Select();

    TFT_SetAddressWindow(x, y, x + 1, y + 1);
    uint8_t data[] = {color >> 8, color & 0xFF};
    TFT_WriteData(data, sizeof(data));

    TFT_Unselect();
}

void TFT_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // Kiểm tra w, h bằng 0
    if ((w == 0) || (h == 0))
        return;

    // ... (code clipping) ...
    if ((x + w - 1) >= TFT_WIDTH)
        w = TFT_WIDTH - x;
    if ((y + h - 1) >= TFT_HEIGHT)
        h = TFT_HEIGHT - y;

    TFT_Select();
    TFT_SetAddressWindow(x, y, x + w - 1, y + h - 1);

    uint8_t pixel[] = {color >> 8, color & 0xFF};
    uint16_t line_size_bytes = w * sizeof(pixel);

    uint8_t *line = malloc(line_size_bytes);
    if (line == NULL)
    {
        TFT_Unselect();
        return;
    }
    for (uint16_t i = 0; i < w; ++i)
        memcpy(line + i * sizeof(pixel), pixel, sizeof(pixel));

    gpio_set_level(TFT_DC, 1);
    for (uint16_t j = h; j > 0; j--)
    {
        TFT_SPI_transmit(line, line_size_bytes);
    }

    free(line);
    TFT_Unselect();
}

void TFT_FillScreen(uint16_t color)
{
    TFT_FillRectangle(0, 0, TFT_WIDTH, TFT_HEIGHT, color);
}

void TFT_InvertColors(bool invert)
{
    TFT_Select();
    TFT_WriteCommand(invert ? 0x21 /* INVON */ : 0x20 /* INVOFF */);
    TFT_Unselect();
}

void TFT_DrawImage_Standard(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
    TFT_Select();
    TFT_SetAddressWindow(x, y, x + w - 1, y + h - 1);
    gpio_set_level(TFT_DC, 1);
    
    // Gửi trực tiếp vào SPI
    TFT_WriteData((uint8_t *)data, sizeof(uint16_t) * w * h);
    
    TFT_Unselect();
}

// Hàm in chữ xuyên thấu 
static void TFT_WriteChar_Transparent(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, const uint16_t *bg_image)
{
    uint32_t i, b, j;
    TFT_SetAddressWindow(x, y, x + font.width - 1, y + font.height - 1);

    for (i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for (j = 0; j < font.width; j++) {
            if ((b << j) & 0x8000) {
                // In màu nét chữ
                uint8_t data[] = {color >> 8, color & 0xFF};
                TFT_WriteData(data, 2);
            } else {
                // Lấy màu nền khớp với ảnh đã Swap Endian
                uint16_t bg_color = bg_image[(y + i) * TFT_WIDTH + (x + j)];
                uint8_t data[] = {bg_color & 0xFF, bg_color >> 8};
                TFT_WriteData(data, 2);
            }
        }
    }
}

void TFT_WriteString_Transparent(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, const uint16_t *bg_image)
{
    TFT_Select();
    while (*str)
    {
        TFT_WriteChar_Transparent(x, y, *str, font, color, bg_image);
        x += font.width;
        str++;
    }
    TFT_Unselect();
}