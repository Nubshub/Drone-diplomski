#ifndef _SOC_H__
#define _SOC_H__

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "i2c_config.h"

#define MOTOR1_CONTROL_PIN          GPIO_NUM_12 /* GPIO pin for motor 1 control */
#define MOTOR2_CONTROL_PIN          GPIO_NUM_13 /* GPIO pin for motor 2 control */
#define MOTOR3_CONTROL_PIN          GPIO_NUM_14 /* GPIO pin for motor 3 control */
#define MOTOR4_CONTROL_PIN          GPIO_NUM_15 /* GPIO pin for motor 4 controlS */
#define MPU6050_REG_ACCEL_XOUT_H    0x3B        /* MPU6050 register address of accelerometer X high byte */
#define GPIO_INPUT_PIN              GPIO_NUM_17 /* GPIO pin for input from MPU6050 sensor*/
#define MPU6050_SENSITIVITY_2G      16384       /* MPU6050 sensitivity at ±2g full scale (LSB/g) */
#define MPU6050_SENSITIVITY_250DPS  131.0f      /* MPU6050 sensitivity at ±250°/s full scale (LSB/°/s) */
#define MPU6050_GYRO_DEADBAND       20          /* MPU6050 gyro deadband threshold (LSB) */
#define MAX_PWM_DUTY_CYCLE          8191        /* Maximum PWM duty cycle for 13-bit resolution */
#define ALPHA                       0.98f       /* Determines the contribution of gyroscope data in the filter */

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
void compl_filter(mpu6050_regs mpu6050_raw, angles_data *angles, float dt, float alpha);

#endif