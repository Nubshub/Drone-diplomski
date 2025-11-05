idf_component_register(SRCS "wifi_config.c" "i2c_config.c" "main.c"
                       PRIV_REQUIRES esp_wifi spi_flash driver
                       INCLUDE_DIRS "")
