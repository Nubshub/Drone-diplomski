idf_component_register(SRCS "comm.c" "control.c" "i2c_config.c" "main.c" "soc.c"
                       PRIV_REQUIRES esp_timer esp_wifi spi_flash nvs_flash driver
                       INCLUDE_DIRS "")
