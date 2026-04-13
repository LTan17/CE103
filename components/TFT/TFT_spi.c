#include <stdio.h>
#include "TFT_spi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SPI";

spi_device_handle_t spi;

void TFT_SPI_init()
{
    // spi = handle_dev;
    spi_bus_config_t buscfg = {
        .mosi_io_num = TFT_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = TFT_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_SPI_CHUNK_SIZE};
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 7,
        .post_cb = post_transaction_callback};
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH1);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi);
    ESP_LOGI(TAG, "Cau hinh TFT SPI thanh cong!");
}

void TFT_SPI_transmit(const uint8_t *data, size_t len)
{
    if (len == 0)
    {
        ESP_LOGI(TAG, " do dai data bang 0");
        return;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    t.user = (void *)1;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

void TFT_SPI_transmit_DMA(const uint8_t *data, size_t len)
{
    if (len == 0)
    {
        return;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    t.user = (void *)1;

    esp_err_t ret = spi_device_queue_trans(spi, &t, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
}

void IRAM_ATTR post_transaction_callback(spi_transaction_t *trans)
{
    //
}