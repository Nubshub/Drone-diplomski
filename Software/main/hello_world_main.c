/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO           22          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           21          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port */
#define I2C_MASTER_FREQ_HZ          100000      /*!< I2C master clock frequency */
#define I2C_MPU6050_ADDR            0x68        /*!< slave address for MPU6050 sensor */
#define MPU6050_REG_ACCEL_XOUT_H    0x3B

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

static void i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .glitch_ignore_cnt = 7,
        .intr_priority = 1,
        .flags.enable_internal_pullup = true,
        // .flags.allow_pd = true,
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    printf("I2C init done!\n");
}


void vTaskTest1 (void * pvParametars)
{
    for(;;)
    {
        printf("Enter task test 1\n");
        uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
        uint8_t data[14]; // accel(6) + temp(2) + gyro(6)

        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, data, sizeof(data), -1));

        // Parse values (16-bit signed big-endian)
        int16_t accel_x = (data[0] << 8) | data[1];
        int16_t accel_y = (data[2] << 8) | data[3];
        int16_t accel_z = (data[4] << 8) | data[5];
        int16_t temp_raw = (data[6] << 8) | data[7];
        int16_t gyro_x  = (data[8] << 8) | data[9];
        int16_t gyro_y  = (data[10] << 8) | data[11];
        int16_t gyro_z  = (data[12] << 8) | data[13];

        float temp_c = (temp_raw / 340.0f) + 36.53f;

        printf("Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d | Temp=%.2f C",
                 accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp_c);
        printf("\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void vTaskTest2 (void * pvParametars)
{
    for(;;)
    {
        printf("Enter task test 2\n");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    BaseType_t xReturned;
    
    uint8_t start_msg[2] = {0x6B, 0x00};

    printf("Hello world from Milos!\n");

    i2c_master_init();

    // Wake up MPU6050
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, start_msg, sizeof(start_msg), -1));
    printf("MPU6050 is started!\n");

    

    xReturned = xTaskCreate(vTaskTest1, "TaskTest1", 4096, NULL, 1, NULL);
    if(xReturned != pdPASS)
    {
        printf("Error: Failed to create task!\n");
    }

    xReturned = xTaskCreate(vTaskTest2, "TaskTest2", 1024, NULL, 1, NULL);
    if(xReturned != pdPASS)
    {
        printf("Error: Failed to create task!\n");
    }


    printf("End app_main.\n");

}
