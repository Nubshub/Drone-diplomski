#ifndef _I2C_CONFIG_H__
#define _I2C_CONFIG_H__

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO           22          /* gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           21          /* gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0   /* I2C port */
#define I2C_MASTER_FREQ_HZ          100000      /* I2C master clock frequency */
#define I2C_MPU6050_ADDR            0x68        /* slave address for MPU6050 sensor */
#define I2C_BMP280_ADDR             0x76        /* slave address for BMP280 sensor */

extern i2c_master_bus_handle_t bus_handle;
extern i2c_master_dev_handle_t mpu6050_handle;
extern i2c_master_dev_handle_t bmp280_handle;

esp_err_t i2c_write_reg(i2c_master_dev_handle_t, uint8_t, uint8_t);
void i2c_master_init(void);

#endif