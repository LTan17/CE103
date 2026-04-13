#ifndef TFT_H
#define TFT_H

#include "TFT_spi.h"
#include "TFT_gpio.h"
#include "fonts.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TFT_SCREEN_WIDTH 320
#define TFT_SCREEN_HEIGHT 240

#define TFT_MADCTL_MY 0x80
#define TFT_MADCTL_MX 0x40
#define TFT_MADCTL_MV 0x20
#define TFT_MADCTL_ML 0x10
#define TFT_MADCTL_RGB 0x00
#define TFT_MADCTL_BGR 0x08
#define TFT_MADCTL_MH 0x04

/*** Redefine if necessary ***/
#define TFT_SPI_PORT spi
extern spi_device_handle_t TFT_SPI_PORT;

// default orientation
/*
#define TFT_WIDTH 240
#define TFT_HEIGHT 320
#define TFT_ROTATION (TFT_MADCTL_MX | TFT_MADCTL_BGR)
*/
// rotate right

#define TFT_WIDTH 320
#define TFT_HEIGHT 240
#define TFT_ROTATION (TFT_MADCTL_MX | TFT_MADCTL_MY | TFT_MADCTL_MV | TFT_MADCTL_BGR)

// rotate left
/*
#define TFT_WIDTH  320
#define TFT_HEIGHT 240
#define TFT_ROTATION (TFT_MADCTL_MV | TFT_MADCTL_BGR)
*/

// upside down
/*
#define TFT_WIDTH  240
#define TFT_HEIGHT 320
#define TFT_ROTATION (TFT_MADCTL_MY | TFT_MADCTL_BGR)
*/

/****************************/

// Color definitions
#define TFT_BLACK 0x0000
#define TFT_BLUE 0x001F
#define TFT_RED 0xF800
#define TFT_GREEN 0x07E0
#define TFT_CYAN 0x07FF
#define TFT_MAGENTA 0xF81F
#define TFT_YELLOW 0xFFE0
#define TFT_WHITE 0xFFFF
#define TFT_COLOR565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

// call before initializing any SPI devices
void TFT_Unselect();
void TFT_Init(void);
void TFT_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void TFT_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void TFT_FillScreen(uint16_t color);
//void TFT_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *data);
void TFT_InvertColors(bool invert);
//void TFT_WriteString_BgImage(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, const uint16_t *bg_image);
//void TFT_DrawImage_FixColor(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);
void TFT_DrawImage_Standard(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);
void TFT_WriteString_Transparent(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, const uint16_t *bg_image);
#endif