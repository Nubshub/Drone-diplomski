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

PID_t roll_pid = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 2.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .integral_limit = 10.0f,
    .output_limit = 5000.0f
};

PID_t pitch_pid = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 2.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .integral_limit = 10.0f,
    .output_limit = 5000.0f
};

PID_t yaw_pid = {
    .kp = 100.0f,
    .ki = 0.0f,
    .kd = 2.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .integral_limit = 10.0f,
    .output_limit = 5000.0f
};

static void vMPU6050_data (void * pvParametars)
{
    float dt = 0.005;
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t buffer[14]; // accel(6) + temp(2) + gyro(6)
    
    while(1)
    {
        ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu6050_handle, &reg, 1, buffer, sizeof(buffer), -1));

        // Parse values (16-bit signed big-endian)
        parse_mpu6050_data(&mpu6050_raw, buffer);
        
        // Complementary filter
        compl_filter(mpu6050_raw, &angles, dt, ALPHA);

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

static void vMotor_control(void *pvParametars)
{
    Motor_thrust_t motors;

    while(1)
    {
        printf("Raw Gyro in degrees/s: X=%.2f, Y=%.2f, Z=%.2f\n", (float)mpu6050_raw.gyro_x / MPU6050_SENSITIVITY_250DPS, (float)mpu6050_raw.gyro_y / MPU6050_SENSITIVITY_250DPS, (float)mpu6050_raw.gyro_z / MPU6050_SENSITIVITY_250DPS);
        printf("Raw Accel in g: X=%.2f, Y=%.2f, Z=%.2f\n", (float)mpu6050_raw.accel_x / MPU6050_SENSITIVITY_2G, (float)mpu6050_raw.accel_y / MPU6050_SENSITIVITY_2G, (float)mpu6050_raw.accel_z / MPU6050_SENSITIVITY_2G);
        printf("Current Roll: %.2f, Current Pitch: %.2f, Current Yaw: %.2f\n", angles.roll, angles.pitch, angles.yaw);
        if(pkt.thrust > 1)
        {
            motors = pid_control(&roll_pid, &pitch_pid, &yaw_pid,
                                pkt.roll, pkt.pitch, pkt.yaw,
                                (float)mpu6050_raw.gyro_y / MPU6050_SENSITIVITY_250DPS, 
                                (float)mpu6050_raw.gyro_x / MPU6050_SENSITIVITY_250DPS, 
                                (float)mpu6050_raw.gyro_z / MPU6050_SENSITIVITY_250DPS,
                                pkt.thrust, 
                                0.005f); // 5ms loop time
            
            motor_thrust(motors.m1, MOTOR_FRONT_LEFT);
            motor_thrust(motors.m2, MOTOR_FRONT_RIGHT);
            motor_thrust(motors.m3, MOTOR_BACK_LEFT);
            motor_thrust(motors.m4, MOTOR_BACK_RIGHT);
        }
        else
        {
            motor_thrust(0, MOTOR_FRONT_LEFT);
            motor_thrust(0, MOTOR_FRONT_RIGHT);
            motor_thrust(0, MOTOR_BACK_LEFT);
            motor_thrust(0, MOTOR_BACK_RIGHT);
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
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

    xReturned = xTaskCreate(vMPU6050_data, "vMPU6050_data", 4096, NULL, 4, NULL);
    if(xReturned != pdPASS)
    {
        printf("Error: Failed to create vMPU6050_data task!\n");
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
