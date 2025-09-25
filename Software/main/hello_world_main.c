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
#include "driver/gpio.h"

#define I2C_MASTER_SCL_IO           22          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           21          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port */
#define I2C_MASTER_FREQ_HZ          100000      /*!< I2C master clock frequency */
#define I2C_MPU6050_ADDR            0x68        /*!< slave address for MPU6050 sensor */
#define MPU6050_REG_ACCEL_XOUT_H    0x3B        /*!< MPU6050 register address of accelerometer X high byte */
#define GPIO_INPUT_PIN              GPIO_NUM_4  /*!< GPIO pin for input from MPU6050 sensor*/

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp_raw;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

}mpu6050_regs;

static mpu6050_regs mpu6050_data;
static TaskHandle_t mpu6050_task_handle = NULL;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    esp_err_t ret;
    uint8_t msg[2] = { reg_addr, data };
    ret = i2c_master_transmit(dev_handle, msg, sizeof(msg), -1);
    return ret;
}

static void IRAM_ATTR mpu6050_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(mpu6050_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void gpio_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_INPUT_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Install GPIO ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Attach the interrupt service routine to the GPIO pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_INPUT_PIN, mpu6050_isr_handler, NULL));
    
    printf("GPIO init done!\n");
}

static void i2c_master_init(void)
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
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    printf("I2C init done!\n");
}

static void configure_mpu6050(void)
{
    // Wake up MPU6050
    ESP_ERROR_CHECK(i2c_write_reg(dev_handle, 0x6B, 0x00));  // PWR_MGMT_1 = 0
    printf("MPU6050 is started!\n");

    // Set accelerometer to ±2g for max sensitivity
    ESP_ERROR_CHECK(i2c_write_reg(dev_handle, 0x1C, 0x00));

    // Motion interrupt
    ESP_ERROR_CHECK(i2c_write_reg(dev_handle, 0x38, 0x40));  // enable motion interrupt
    ESP_ERROR_CHECK(i2c_write_reg(dev_handle, 0x1F, 0x02));  // lower threshold → small motion
    ESP_ERROR_CHECK(i2c_write_reg(dev_handle, 0x20, 0x01));  // short duration → fast trigger
    ESP_ERROR_CHECK(i2c_write_reg(dev_handle, 0x37, 0x10));  // INT pin config
}

static void vTask_mpu6050 (void * pvParametars)
{
    for(;;)
    {
        printf("Enter vTask_mpu6050\n");

        // Wait for GPIO interrupt to notify the task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        printf("GPIO interrupt triggered!\n");

        uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
        uint8_t data[14]; // accel(6) + temp(2) + gyro(6)

        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg, 1, data, sizeof(data), -1));

        // Parse values (16-bit signed big-endian)
        mpu6050_data.accel_x = (data[0] << 8) | data[1];
        mpu6050_data.accel_y = (data[2] << 8) | data[3];
        mpu6050_data.accel_z = (data[4] << 8) | data[5];
        mpu6050_data.temp_raw = (data[6] << 8) | data[7];
        mpu6050_data.gyro_x  = (data[8] << 8) | data[9];
        mpu6050_data.gyro_y  = (data[10] << 8) | data[11];
        mpu6050_data.gyro_z  = (data[12] << 8) | data[13];
        
        printf("Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d \n",
                 mpu6050_data.accel_x, mpu6050_data.accel_y, mpu6050_data.accel_z, 
                 mpu6050_data.gyro_x, mpu6050_data.gyro_y, mpu6050_data.gyro_z);

        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    BaseType_t xReturned;
    
    printf("Hello world from Milos!\n");

    xReturned = xTaskCreate(vTask_mpu6050, "vTask_mpu6050", 4096, NULL, 1, &mpu6050_task_handle);
    if(xReturned != pdPASS)
    {
        printf("Error: Failed to create task!\n");
    }

    gpio_init();

    i2c_master_init();
    
    configure_mpu6050();  

    printf("End app_main.\n");

}
