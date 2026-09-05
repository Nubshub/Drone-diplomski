#ifndef _CONTROL_H__
#define _CONTROL_H__

#include <stdio.h>
#include <inttypes.h>
#include "soc.h"

#define MAX_THURST_VALUE            65535
#define MIN_THURST_VALUE            10000
#define THURST_VALUE_90             58500
#define CLAMP(x, lo, hi)            ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

#define MOTOR_FRONT_LEFT            0
#define MOTOR_FRONT_RIGHT           1
#define MOTOR_BACK_LEFT             2
#define MOTOR_BACK_RIGHT            3

#define ROLL_PID_KP                 400.0f
#define ROLL_PID_KI                 0.0f
#define ROLL_PID_KD                 40.0f

#define PITCH_PID_KP                400.0f
#define PITCH_PID_KI                0.0f
#define PITCH_PID_KD                40.0f

#define YAW_PID_KP                  0.0f
#define YAW_PID_KI                  0.0f
#define YAW_PID_KD                  0.0f

#define PID_OUTPUT_LIMIT_RP         15000.0f
#define PID_OUTPUT_LIMIT_YAW        8000.0f
#define PID_INTEGRAL_LIMIT_RP       6000.0f
#define PID_INTEGRAL_LIMIT_YAW      3000.0f

#define YAW_MIX_ENABLED             0

typedef struct {
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
} Motor_thrust_t;

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float integral_limit;
    float output_limit;

    float p_out;
    float i_out;
    float d_out;
    float out;
} PID_t;

extern PID_t roll_pid;
extern PID_t pitch_pid;
extern PID_t yaw_pid;

void  motor_thrust(uint16_t thrust, uint8_t motor_index);
void  pid_reset(PID_t *pid);
float pid_update(PID_t *pid, float target, float current, float rate, float dt);

Motor_thrust_t pid_control(float target_roll, float target_pitch, float target_yaw_rate,
                           float current_roll, float current_pitch,
                           float roll_rate, float pitch_rate, float yaw_rate,
                           uint16_t base_thrust, float dt);

#endif