#ifndef TFT_SPI_H
#define TFT_SPI_H

#include "driver/spi_master.h"
#include "esp_log.h"

#define TFT_MOSI 23
#define TFT_CLK 18

#define MAX_SPI_CHUNK_SIZE 52100 * 3

void TFT_SPI_init();
void TFT_SPI_transmit(const uint8_t *data, size_t len);
void TFT_SPI_transmit_DMA(const uint8_t *data, size_t len);
void post_transaction_callback(spi_transaction_t *trans);

#endif