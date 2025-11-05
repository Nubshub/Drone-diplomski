#include "i2c_config.h"

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t mpu6050_handle;
i2c_master_dev_handle_t bmp280_handle;

esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    esp_err_t ret;
    uint8_t msg[2] = { reg_addr, data };
    ret = i2c_master_transmit(dev_handle, msg, sizeof(msg), -1);
    return ret;
}

void i2c_master_init(void)
{
    // Initialize I2C bus
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .intr_priority = 1,
        .flags.enable_internal_pullup = true,
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // Add MPU6050 device to I2C bus
    i2c_device_config_t mpu6050_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // Add BMP280 device to I2C bus
    i2c_device_config_t bmp280_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_BMP280_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu6050_cfg, &mpu6050_handle));
    printf("MPU6050 device added to I2C bus!\n");
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_add_device(bus_handle, &bmp280_cfg, &bmp280_handle));
    printf("BMP280 device added to I2C bus!\n");

    printf("I2C init done!\n");
}