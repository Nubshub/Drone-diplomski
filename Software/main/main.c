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

static portMUX_TYPE pkt_spinlock = portMUX_INITIALIZER_UNLOCKED;

static void vMotor_control(void *pvParametars)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Motor_thrust_t motors;
    float dt = 0.005;
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t buffer[14]; // accel(6) + temp(2) + gyro(6)

    while(1)
    {
        // printf("Raw Gyro in degrees/s: X=%.2f, Y=%.2f, Z=%.2f\n", (float)mpu6050_raw.gyro_x / MPU6050_SENSITIVITY_250DPS, 
        //     (float)mpu6050_raw.gyro_y / MPU6050_SENSITIVITY_250DPS, (float)mpu6050_raw.gyro_z / MPU6050_SENSITIVITY_250DPS);

        // printf("Raw Accel in g: X=%.2f, Y=%.2f, Z=%.2f\n", (float)mpu6050_raw.accel_x / MPU6050_SENSITIVITY_2G, 
        //     (float)mpu6050_raw.accel_y / MPU6050_SENSITIVITY_2G, (float)mpu6050_raw.accel_z / MPU6050_SENSITIVITY_2G);

        // printf("Current Roll: %.2f, Current Pitch: %.2f, Current Yaw: %.2f\n", angles.roll, angles.pitch, angles.yaw);
        
        // Create a local copy for received packets
        crtp_packet_t local_pkt;

        taskENTER_CRITICAL(&pkt_spinlock);
        local_pkt = pkt;
        taskEXIT_CRITICAL(&pkt_spinlock);

        esp_err_t err = i2c_master_transmit_receive(mpu6050_handle, &reg, 1, buffer, sizeof(buffer), -1);
        if (err != ESP_OK)
        {
            printf("Failed to read MPU6050\n");
            continue; // Skip one iteration
        }
        // Parse values (16-bit signed big-endian)
        parse_mpu6050_data(&mpu6050_raw, buffer);
        
        // Complementary filter
        compl_filter(mpu6050_raw, &angles, dt, ALPHA);

        if(pkt.thrust > 1)
        {
            motors = pid_control(local_pkt.roll, local_pkt.pitch, local_pkt.yaw,
                                angles.roll, angles.pitch, angles.yaw,
                                local_pkt.thrust, 
                                dt); // 5ms loop time
            
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

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
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
            continue;
        } else if (len > 0) {
            rx_buffer[len] = '\0'; // Null-terminate the received data
            taskENTER_CRITICAL(&pkt_spinlock);
            memcpy(&pkt, rx_buffer, sizeof(crtp_packet_t));
            taskEXIT_CRITICAL(&pkt_spinlock);
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


    xReturned = xTaskCreate(vUdp_server, 
                            "vUdp_server", 
                            4096, 
                            NULL, 
                            2, 
                            NULL);
    if(xReturned != pdPASS)
    {
        printf("Error: Failed to create  vUdp_server task!\n");
    }

    xReturned = xTaskCreatePinnedToCore(vMotor_control, 
                                        "vMotor_control", 
                                        4096, 
                                        NULL, 
                                        5, 
                                        NULL, 
                                        1); // Pinned to core 1
    if(xReturned != pdPASS)
    {
        printf("Error: Failed to create  vMotor_control task!\n");
    }

    printf("End app_main.\n");

}
