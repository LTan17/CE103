idf_component_register(SRCS "TaskManager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES Controller esp_driver_gptimer esp_timer Encoder TFT Button)
