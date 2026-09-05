#ifndef _SOC_H__
#define _SOC_H__

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "i2c_config.h"

#define MOTOR1_CONTROL_PIN          GPIO_NUM_12
#define MOTOR2_CONTROL_PIN          GPIO_NUM_13
#define MOTOR3_CONTROL_PIN          GPIO_NUM_14
#define MOTOR4_CONTROL_PIN          GPIO_NUM_15
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_SENSITIVITY_4G      8192
#define MPU6050_SENSITIVITY_500DPS  65.5f
#define MAX_PWM_DUTY_CYCLE          8191
#define ALPHA                       0.98f

#define GYRO_SIGN_ROLL              (-1.0f)
#define GYRO_SIGN_PITCH             (-1.0f)

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp_raw;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_regs;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} angles_data;

typedef struct {
    ledc_channel_t ch;
    int gpio;
} motor_config_t;

extern mpu6050_regs mpu6050_raw;
extern angles_data angles;
extern int16_t gyro_bias_x, gyro_bias_y, gyro_bias_z;
extern const motor_config_t motors[4];

void timer_config(void);
void configure_mpu6050(void);
void calibrate_mpu6050(void);
void parse_mpu6050_data(mpu6050_regs *data, uint8_t raw_data[]);
void compl_filter(const mpu6050_regs *raw, angles_data *angles, float dt, float alpha);

float roll_rate_dps(const mpu6050_regs *r);
float pitch_rate_dps(const mpu6050_regs *r);
float yaw_rate_dps(const mpu6050_regs *r);

#endif