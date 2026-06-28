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
#include "comm.h"
#include "control.h"
#include "soc.h"


static void vTask_mpu6050 (void * pvParametars)
{
    while(1)
    {
        uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
        uint8_t data[14]; // accel(6) + temp(2) + gyro(6)

        ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu6050_handle, &reg, 1, data, sizeof(data), -1));

        // Parse values (16-bit signed big-endian)
        mpu6050_data.accel_x  = (int16_t)((data[0] << 8) | data[1]);
        mpu6050_data.accel_y  = (int16_t)((data[2] << 8) | data[3]);
        mpu6050_data.accel_z  = (int16_t)((data[4] << 8) | data[5]);
        mpu6050_data.temp_raw = (int16_t)((data[6] << 8) | data[7]);
        mpu6050_data.gyro_x  = (int16_t)((data[8] << 8)  | data[9])  - gyro_bias_x;
        mpu6050_data.gyro_y  = (int16_t)((data[10] << 8) | data[11]) - gyro_bias_y;
        mpu6050_data.gyro_z  = (int16_t)((data[12] << 8) | data[13]) - gyro_bias_z;
        

        // printf("Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d\n",
        //        mpu6050_data.accel_x, mpu6050_data.accel_y, mpu6050_data.accel_z,
        //        mpu6050_data.gyro_x, mpu6050_data.gyro_y, mpu6050_data.gyro_z);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

static void vMotor_control(void *pvParametars)
{
    while(1)
    {
        uint16_t m0 = pkt.thrust;
        uint16_t m1 = pkt.thrust;
        uint16_t m2 = pkt.thrust;
        uint16_t m3 = pkt.thrust;

        if(pkt.thrust > 0){
            // If the gyro readings exceed the deadband threshold, calculate errors and adjust motor thrust accordingly
            if(mpu6050_data.gyro_x > MPU6050_GYRO_DEADBAND || mpu6050_data.gyro_x < -MPU6050_GYRO_DEADBAND ||
            mpu6050_data.gyro_y > MPU6050_GYRO_DEADBAND || mpu6050_data.gyro_y < -MPU6050_GYRO_DEADBAND ||
            mpu6050_data.gyro_z > MPU6050_GYRO_DEADBAND || mpu6050_data.gyro_z < -MPU6050_GYRO_DEADBAND) {

                // Calculate errors between desired angles and current gyro readings
                float pitch_error = pkt.pitch - (float)mpu6050_data.gyro_x / MPU6050_SENSITIVITY_250DPS; 
                float roll_error = pkt.roll - (float)mpu6050_data.gyro_y / MPU6050_SENSITIVITY_250DPS;
                float yaw_error = pkt.yaw - (float)mpu6050_data.gyro_z / MPU6050_SENSITIVITY_250DPS; 

                // Calculate adjustments based on proportional gain
                float roll_adjustment = KP_LEVELING * roll_error;
                float pitch_adjustment = KP_LEVELING * pitch_error;
                float yaw_adjustment = KP_LEVELING * yaw_error;

                // Adjust motor thrust based on errors and adjustments
                m0 += (uint16_t)(roll_adjustment + pitch_adjustment + yaw_adjustment);
                m1 += (uint16_t)(pkt.thrust + -roll_adjustment + pitch_adjustment - yaw_adjustment);
                m2 += (uint16_t)(pkt.thrust + roll_adjustment - pitch_adjustment - yaw_adjustment);
                m3 += (uint16_t)(pkt.thrust + -roll_adjustment - pitch_adjustment + yaw_adjustment);

                // Clamp thrust values to ensure they are within valid range
                m0 = CLAMP(m0, 0, THURST_VALUE_90);
                m1 = CLAMP(m1, 0, THURST_VALUE_90);
                m2 = CLAMP(m2, 0, THURST_VALUE_90);
                m3 = CLAMP(m3, 0, THURST_VALUE_90);
            }
                motor_thrust(m0, MOTOR_FRONT_LEFT);
                motor_thrust(m1, MOTOR_FRONT_RIGHT);
                motor_thrust(m2, MOTOR_BACK_LEFT);
                motor_thrust(m3, MOTOR_BACK_RIGHT);

        } else {
            motor_thrust(0, MOTOR_FRONT_LEFT);
            motor_thrust(0, MOTOR_FRONT_RIGHT);
            motor_thrust(0, MOTOR_BACK_LEFT);
            motor_thrust(0, MOTOR_BACK_RIGHT);
        }

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

static void vUdp_server(void * pvParametars)
{
    
    struct sockaddr_in server_addr, client_addr;
    socklen_t socklen = sizeof(client_addr);
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    UDP_server_init(&server_addr, &client_addr, &socklen, &sock);

    while (1) {
        len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&client_addr, &socklen);
        if (len < 0) {
            ESP_LOGE("ESP32_UDP_AP", "recvfrom failed: errno %d", errno);
            break;
        } else if (len > 0) {
            rx_buffer[len] = '\0'; // Null-terminate the received data
            memcpy(&pkt, rx_buffer, sizeof(crtp_packet_t));
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
    calibrate_mpu6050();

    timer_config();

    wifi_init();

    xReturned = xTaskCreate(vTask_mpu6050, "vTask_mpu6050", 4096, NULL, 1, NULL);
    if(xReturned != pdPASS)
    {
        printf("Error: Failed to create vTask_mpu6050 task!\n");
    }

    xReturned = xTaskCreate(vUdp_server, "vUdp_server", 4096, NULL, 5, NULL);
    if(xReturned != pdPASS)
    {
        printf("Error: Failed to create  vUdp_server task!\n");
    }

    xReturned = xTaskCreate(vMotor_control, "vMotor_control", 4096, NULL, 2, NULL);
    if(xReturned != pdPASS)
    {
        printf("Error: Failed to create  vMotor_control task!\n");
    }

    printf("End app_main.\n");

}
