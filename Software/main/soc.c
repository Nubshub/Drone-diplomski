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

void calibrate_mpu6050(void)
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

void configure_mpu6050(void)
{
    // Full device reset — clears all registers including hardware offset registers
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x6B, 0x80));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x6B, 0x01));  // PWR_MGMT_1: wake, PLL gyro X
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x1A, 0x03));  // CONFIG: DLPF 42 Hz (reduces vibration noise)
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x1B, 0x00));  // GYRO_CONFIG: ±250°/s
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x1C, 0x00));  // ACCEL_CONFIG: ±2g
    ESP_ERROR_CHECK(i2c_write_reg(mpu6050_handle, 0x38, 0x00));  // INT_ENABLE: all off (polling mode)
    printf("MPU6050 configured.\n");
}

void parse_mpu6050_data(mpu6050_regs *data, uint8_t *buffer)
{
        uint16_t accel_x  = (int16_t)((buffer[0] << 8) | buffer[1]);
        uint16_t accel_y  = (int16_t)((buffer[2] << 8) | buffer[3]);
        uint16_t accel_z  = (int16_t)((buffer[4] << 8) | buffer[5]);
        uint16_t gyro_x  = (int16_t)((buffer[8] << 8)  | buffer[9])  - gyro_bias_x;
        uint16_t gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]) - gyro_bias_y;
        uint16_t gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]) - gyro_bias_z;

        data->accel_x  = (int16_t)(accel_x);
        data->accel_y  = (int16_t)(accel_y);
        data->accel_z  = (int16_t)(accel_z);
        data->temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
        data->gyro_x  = (int16_t)(gyro_x);
        data->gyro_y  = (int16_t)(gyro_y);
        data->gyro_z  = (int16_t)(gyro_z);
}

void compl_filter(mpu6050_regs mpu6050_raw, angles_data *angles, float dt, int alpha)
{
    // Convert gyro readings to degrees/s
    float gyro_x = (float)mpu6050_raw.gyro_x / MPU6050_SENSITIVITY_250DPS;
    float gyro_y = (float)mpu6050_raw.gyro_y / MPU6050_SENSITIVITY_250DPS;
    float gyro_z = (float)mpu6050_raw.gyro_z / MPU6050_SENSITIVITY_250DPS;

    // Integrate gyro data to get angles
    angles->roll  += gyro_y * dt;
    angles->pitch += gyro_x * dt;
    angles->yaw   += gyro_z * dt;

    // Calculate accelerometer angles
    float accel_roll  = atan2f((float)mpu6050_raw.accel_y, (float)mpu6050_raw.accel_z) * (180.0f / M_PI);
    float accel_pitch = atan2f(-(float)mpu6050_raw.accel_x, sqrtf((float)mpu6050_raw.accel_y * (float)mpu6050_raw.accel_y + (float)mpu6050_raw.accel_z * (float)mpu6050_raw.accel_z)) * (180.0f / M_PI);
    
    // Complementary filter
    angles->roll  = ((alpha * angles->roll) + ((100 - alpha) * accel_roll)) / 100.0f;
    angles->pitch = ((alpha * angles->pitch) + ((100 - alpha) * accel_pitch)) / 100.0f;
    angles->yaw   = angles->yaw; // Yaw is not corrected by accelerometer

}