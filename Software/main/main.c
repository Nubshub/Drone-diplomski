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
#include "driver/ledc.h"


#define MPU6050_REG_ACCEL_XOUT_H    0x3B        /*!< MPU6050 register address of accelerometer X high byte */
#define GPIO_INPUT_PIN              GPIO_NUM_17  /*!< GPIO pin for input from MPU6050 sensor*/
#define MOTOR1_CONTROL_PIN          GPIO_NUM_12 /*!< GPIO pin for motor 1 control */
#define MOTOR2_CONTROL_PIN          GPIO_NUM_13 /*!< GPIO pin for motor 2 control */
#define MOTOR3_CONTROL_PIN          GPIO_NUM_14 /*!< GPIO pin for motor 3 control */
#define MOTOR4_CONTROL_PIN          GPIO_NUM_15 /*!< GPIO pin for motor 4 controlS */
#define MPU6050_SENSITIVITY_2G      16384       /*!< MPU6050 sensitivity at ±2g full scale (LSB/g) */
#define MPU6050_SENSITIVITY_500DPS  65.5        /*!< MPU6050 sensitivity at ±500°/s full scale (LSB/°/s) */
#define MPU6050_GYRO_DEADBAND       20           /*!< MPU6050 gyro deadband threshold (LSB) */
#define MAX_THURST_VALUE            65535     /*!< Maximum thrust value for motor control is 65535 */
// #define THURST_VALUE_90             58500      /*!< Maximum thrust value to avoid overloading motors */
#define MAX_PWM_DUTY_CYCLE          8191        /*!< Maximum PWM duty cycle for 10-bit resolution */
#define KP_LEVELING                 2       /*!< Proportional gain for roll/pitch leveling — tune this */
#define KD_LEVELING                 0.02      /*!< Derivative gain (gyro damping) — tune this */
#define CLAMP(x, lo, hi)            ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp_raw;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

}mpu6050_regs;

typedef struct __attribute__((packed)) {
    uint8_t channel : 4;
    uint8_t port : 4;
    float roll;
    float pitch;
    float yaw;
    uint16_t thrust;
    uint8_t reserved;
} crtp_packet_t;

crtp_packet_t pkt;

uint8_t rx_buffer[1024];
int len;
static mpu6050_regs mpu6050_data;
static int16_t gyro_bias_x, gyro_bias_y, gyro_bias_z;

const struct {ledc_channel_t ch; int gpio;} motors[] = {
    {LEDC_CHANNEL_0, MOTOR1_CONTROL_PIN},
    {LEDC_CHANNEL_1, MOTOR2_CONTROL_PIN},
    {LEDC_CHANNEL_2, MOTOR3_CONTROL_PIN},
    {LEDC_CHANNEL_3, MOTOR4_CONTROL_PIN},
};
static void timer_config(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 50,  // Set frequency to 50 kHz for motor control
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    printf("LEDC timer configured for motor control.\n");

    for(int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_HIGH_SPEED_MODE,
            .channel        = motors[i].ch,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = motors[i].gpio,
            .duty           = 0, // Start with 0% duty cycle
            .hpoint         = 0
        };

        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
   
    printf("LEDC channel configured for motor control.\n");


}

static void calibrate_mpu6050(void)
{
    const int N = 500;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    // Wait for PLL to lock and DLPF pipeline to fill
    vTaskDelay(100 / portTICK_PERIOD_MS);

    printf("MPU6050 calibration started, keep drone still...\n");

    for (int i = 0; i < N; i++) {
        uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
        uint8_t data[14];
        ESP_ERROR_CHECK(i2c_master_transmit_receive(mpu6050_handle, &reg, 1, data, sizeof(data), -1));

        sum_gx += (int16_t)((data[8] << 8)  | data[9]);
        sum_gy += (int16_t)((data[10] << 8) | data[11]);
        sum_gz += (int16_t)((data[12] << 8) | data[13]);

        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    // Gyro bias
    gyro_bias_x = (int16_t)(sum_gx / N);
    gyro_bias_y = (int16_t)(sum_gy / N);
    gyro_bias_z = (int16_t)(sum_gz / N);

    printf("Calibration done.\n");
    printf("  Gyro bias  (LSB): X=%d Y=%d Z=%d\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

static void configure_mpu6050(void)
{
    // Full device reset — clears all registers including hardware offset registers
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x6B, 0x80));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x6B, 0x01));  // PWR_MGMT_1: wake, PLL gyro X
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x1A, 0x03));  // CONFIG: DLPF 42 Hz (reduces vibration noise)
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x1B, 0x08));  // GYRO_CONFIG: ±500°/s
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x1C, 0x00));  // ACCEL_CONFIG: ±2g
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x38, 0x00));  // INT_ENABLE: all off (polling mode)
    printf("MPU6050 configured.\n");
}

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

static void motor_thrust(uint16_t thrust, uint8_t motor_index)
{
    if (thrust > MAX_THURST_VALUE) {
        thrust = MAX_THURST_VALUE;
    }

    uint32_t duty = ((uint32_t)thrust * MAX_PWM_DUTY_CYCLE) / MAX_THURST_VALUE;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, motors[motor_index].ch, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, motors[motor_index].ch));

}

static void vMotor_control(void *pvParametars)
{
    while(1)
    {
        memcpy(&pkt, rx_buffer, sizeof(pkt));

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
                float roll_error = pkt.roll - (float)mpu6050_data.gyro_x / MPU6050_SENSITIVITY_500DPS;
                float pitch_error = pkt.pitch - (float)mpu6050_data.gyro_y / MPU6050_SENSITIVITY_500DPS; 
                float yaw_error = pkt.yaw - (float)mpu6050_data.gyro_z / MPU6050_SENSITIVITY_500DPS; 

                // Calculate adjustments based on proportional gain
                float roll_adjustment = KP_LEVELING * roll_error;
                float pitch_adjustment = KP_LEVELING * pitch_error;
                float yaw_adjustment = KP_LEVELING * yaw_error;

                // Adjust motor thrust based on errors and adjustments
                m0 = pkt.thrust + (uint16_t)(roll_adjustment + pitch_adjustment + yaw_adjustment);
                m1 = pkt.thrust + (uint16_t)(roll_adjustment - pitch_adjustment - yaw_adjustment);
                m2 = pkt.thrust + (uint16_t)(-roll_adjustment + pitch_adjustment - yaw_adjustment);
                m3 = pkt.thrust + (uint16_t)(-roll_adjustment - pitch_adjustment + yaw_adjustment);

                // Clamp thrust values to ensure they are within valid range
                m0 = CLAMP(m0, 0, MAX_THURST_VALUE);
                m1 = CLAMP(m1, 0, MAX_THURST_VALUE);
                m2 = CLAMP(m2, 0, MAX_THURST_VALUE);
                m3 = CLAMP(m3, 0, MAX_THURST_VALUE);
            }
                motor_thrust(m0, 0);
                motor_thrust(m1, 1);
                motor_thrust(m2, 2);
                motor_thrust(m3, 3);

        } else {
            motor_thrust(0, 0);
            motor_thrust(0, 1);
            motor_thrust(0, 2);
            motor_thrust(0, 3);
        }

        
        // uint16_t m0 = pkt.thrust;
        // uint16_t m1 = pkt.thrust;
        // uint16_t m2 = pkt.thrust;
        // uint16_t m3 = pkt.thrust;

        // if (pkt.thrust > 0) {

        //     if (mpu6050_data.gyro_x > MPU6050_GYRO_DEADBAND) {
        //         m0 = (uint16_t)(m0 * 0.8f);
        //         m1 = (uint16_t)(m1 * 0.8f);
        //     } else if (mpu6050_data.gyro_x < -MPU6050_GYRO_DEADBAND) {
        //         m2 = (uint16_t)(m2 * 0.8f);
        //         m3 = (uint16_t)(m3 * 0.8f);
        //     }

        //     if (mpu6050_data.gyro_y > MPU6050_GYRO_DEADBAND) {
        //         m0 = (uint16_t)(m0 * 0.8f);
        //         m2 = (uint16_t)(m2 * 0.8f);
        //     } else if (mpu6050_data.gyro_y < -MPU6050_GYRO_DEADBAND) {
        //         m1 = (uint16_t)(m1 * 0.8f);
        //         m3 = (uint16_t)(m3 * 0.8f);
        //     }

        //     motor_thrust(m0, 0);
        //     motor_thrust(m1, 1);
        //     motor_thrust(m2, 2);
        //     motor_thrust(m3, 3);

        // } else {
        //     motor_thrust(0, 0);
        //     motor_thrust(0, 1);
        //     motor_thrust(0, 2);
        //     motor_thrust(0, 3);
        // }

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

static void vUdp_server(void * pvParametars)
{
    
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
        len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&client_addr, &socklen);
        if (len < 0) {
            ESP_LOGE("ESP32_UDP_AP", "recvfrom failed: errno %d", errno);
            break;
        } else if (len > 0) {
            rx_buffer[len] = '\0'; // Null-terminate the received data
            // parse_crtp_packet(rx_buffer, len);
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

    xReturned = xTaskCreate(vMotor_control, "vMotor_control", 4096, NULL, 6, NULL);
    if(xReturned != pdPASS)
    {
        printf("Error: Failed to create  vMotor_control task!\n");
    }


   
    printf("End app_main.\n");

}
