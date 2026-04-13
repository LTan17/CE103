idf_component_register(SRCS "TFT.c" "TFT_spi.c" "TFT_gpio.c" "fonts.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_driver_spi esp_driver_gpio) 