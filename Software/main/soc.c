#include "soc.h"

mpu6050_regs mpu6050_raw;
angles_data angles;
int16_t gyro_bias_x, gyro_bias_y, gyro_bias_z;

const motor_config_t motors[4] = {
    {LEDC_CHANNEL_0, MOTOR1_CONTROL_PIN},
    {LEDC_CHANNEL_1, MOTOR2_CONTROL_PIN},
    {LEDC_CHANNEL_2, MOTOR3_CONTROL_PIN},
    {LEDC_CHANNEL_3, MOTOR4_CONTROL_PIN},
};

void timer_config(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_HIGH_SPEED_MODE,
            .channel        = motors[i].ch,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = motors[i].gpio,
            .duty           = 0,
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    printf("LEDC configured for motor control.\n");
}

void calibrate_mpu6050(void)
{
    const int N = 500;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

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

    gyro_bias_x = (int16_t)(sum_gx / N);
    gyro_bias_y = (int16_t)(sum_gy / N);
    gyro_bias_z = (int16_t)(sum_gz / N);

    printf("Calibration done. Gyro bias (LSB): X=%d Y=%d Z=%d\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

void configure_mpu6050(void)
{
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x6B, 0x80));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x6B, 0x01));  // PWR_MGMT_1: PLL gyro X
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x1A, 0x03));  // DLPF 42 Hz
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x1B, 0x08));  // GYRO ±500 °/s
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x1C, 0x08));  // ACCEL ±4g
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x38, 0x00));  // INT off
    printf("MPU6050 configured.\n");
}

void parse_mpu6050_data(mpu6050_regs *data, uint8_t *buffer)
{
    data->accel_x  = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y  = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z  = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
    data->gyro_x   = (int16_t)((buffer[8] << 8)  | buffer[9])  - gyro_bias_x;
    data->gyro_y   = (int16_t)((buffer[10] << 8) | buffer[11]) - gyro_bias_y;
    data->gyro_z   = (int16_t)((buffer[12] << 8) | buffer[13]) - gyro_bias_z;
}

float roll_rate_dps(const mpu6050_regs *r)
{
    return GYRO_SIGN_ROLL * (float)r->gyro_y / MPU6050_SENSITIVITY_500DPS;
}

float pitch_rate_dps(const mpu6050_regs *r)
{
    return GYRO_SIGN_PITCH * (float)r->gyro_x / MPU6050_SENSITIVITY_500DPS;
}

float yaw_rate_dps(const mpu6050_regs *r)
{
    return (float)r->gyro_z / MPU6050_SENSITIVITY_500DPS;
}

void compl_filter(const mpu6050_regs *raw, angles_data *angles, float dt, float alpha)
{
    angles->roll  += roll_rate_dps(raw)  * dt;
    angles->pitch += pitch_rate_dps(raw) * dt;
    angles->yaw    = yaw_rate_dps(raw);

    float accel_roll = atan2f((float)raw->accel_x,
                              sqrtf((float)raw->accel_y * raw->accel_y +
                                    (float)raw->accel_z * raw->accel_z)) * (180.0f / M_PI);

    float accel_pitch = atan2f(-(float)raw->accel_y,
                               sqrtf((float)raw->accel_x * raw->accel_x +
                                     (float)raw->accel_z * raw->accel_z)) * (180.0f / M_PI);

    angles->roll  = alpha * angles->roll  + (1.0f - alpha) * accel_roll;
    angles->pitch = alpha * angles->pitch + (1.0f - alpha) * accel_pitch;
}