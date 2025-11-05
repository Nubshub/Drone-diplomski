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
#include "i2c_config.h"
#include "wifi_config.h"


#define MPU6050_REG_ACCEL_XOUT_H    0x3B        /*!< MPU6050 register address of accelerometer X high byte */
#define GPIO_INPUT_PIN              GPIO_NUM_4  /*!< GPIO pin for input from MPU6050 sensor*/
#define MPU6050_SENSITIVITY_2G      16384    /*!< MPU6050 sensitivity at ±2g full scale (LSB/g) */
#define MPU6050_SENSITIVITY_250DPS   131     /*!< MPU6050 sensitivity at ±250°/s full scale (LSB/°/s) */

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


static void configure_mpu6050(void)
{
    // Wake up MPU6050
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x6B, 0x00));  // PWR_MGMT_1 = 0
    printf("MPU6050 is started!\n");

    // Set accelerometer to ±2g for max sensitivity
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x1C, 0x00));

    // Motion interrupt
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x38, 0x40));  // enable motion interrupt
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x1F, 0x02));  // lower threshold → small motion
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x20, 0x01));  // short duration → fast trigger
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x37, 0x10));  // INT pin config
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

        ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu6050_handle, &reg, 1, data, sizeof(data), -1));

        // Parse values (16-bit signed big-endian)
        mpu6050_data.accel_x = (data[0] << 8) | data[1];
        mpu6050_data.accel_y = (data[2] << 8) | data[3];
        mpu6050_data.accel_z = (data[4] << 8) | data[5];
        mpu6050_data.temp_raw = (data[6] << 8) | data[7];
        mpu6050_data.gyro_x  = (data[8] << 8) | data[9];
        mpu6050_data.gyro_y  = (data[10] << 8) | data[11];
        mpu6050_data.gyro_z  = (data[12] << 8) | data[13];
        
        printf("Accel: X=%.2f g Y=%.2f g Z=%.2f g | Gyro: X=%.2f o/s Y=%.2f o/s Z=%.2f o/s \n",
                 (float)mpu6050_data.accel_x/MPU6050_SENSITIVITY_2G, (float)mpu6050_data.accel_y/MPU6050_SENSITIVITY_2G, (float)mpu6050_data.accel_z/MPU6050_SENSITIVITY_2G, 
                 (float)mpu6050_data.gyro_x/MPU6050_SENSITIVITY_250DPS, (float)mpu6050_data.gyro_y/MPU6050_SENSITIVITY_250DPS, (float)mpu6050_data.gyro_z/MPU6050_SENSITIVITY_250DPS);

        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void parse_crtp_packet(uint8_t *data, int len)
{
    if (len < 2) {
        return; // Packet too short
    }

    uint8_t channel = data[0] & 0x0F; // Lower 4 bits for channel
    uint8_t command = data[1];

    printf("Received CRTP packet: Channel=%d, Command=0x%02X, Length=%d\n", channel, command, len);

    // Further processing based on channel and command can be added here
}

static void vUdp_server(void * pvParametars)
{
    uint8_t rx_buffer[128];
    struct sockaddr_in server_addr, client_addr;
    socklen_t socklen = sizeof(client_addr);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE("ESP32_UDP_AP", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE("ESP32_UDP_AP", "Socket bind failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI("ESP32_UDP_AP", "UDP server listening on port %d", UDP_PORT);

    while (1) {
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&client_addr, &socklen);
        if (len < 0) {
            ESP_LOGE("ESP32_UDP_AP", "recvfrom failed: errno %d", errno);
            break;
        } else if (len > 0) {
            parse_crtp_packet(rx_buffer, len);
        }
    }

    close(sock);
    vTaskDelete(NULL);
}


void app_main(void)
{
    BaseType_t xReturned;
    
    i2c_master_init();
    
    configure_mpu6050();  

    gpio_init();

    wifi_init();

    xReturned = xTaskCreate(vTask_mpu6050, "vTask_mpu6050", 4096, NULL, 1, &mpu6050_task_handle);
    if(xReturned != pdPASS)
    {
        printf("Error: Failed to create vTask_mpu6050 task!\n");
    }

    xReturned = xTaskCreate(vUdp_server, "vUdp_server", 4096, NULL, 5, NULL);
    if(xReturned != pdPASS)
    {
        printf("Error: Failed to create  vUdp_server task!\n");
    }


   
    printf("End app_main.\n");

}
