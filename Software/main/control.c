#include "control.h"
#include "soc.h"

PID_t roll_pid = {
    .kp = ROLL_PID_KP, .ki = ROLL_PID_KI, .kd = ROLL_PID_KD,
    .integral_limit = PID_INTEGRAL_LIMIT_RP,
    .output_limit   = PID_OUTPUT_LIMIT_RP
};

PID_t pitch_pid = {
    .kp = PITCH_PID_KP, .ki = PITCH_PID_KI, .kd = PITCH_PID_KD,
    .integral_limit = PID_INTEGRAL_LIMIT_RP,
    .output_limit   = PID_OUTPUT_LIMIT_RP
};

PID_t yaw_pid = {
    .kp = YAW_PID_KP, .ki = YAW_PID_KI, .kd = YAW_PID_KD,
    .integral_limit = PID_INTEGRAL_LIMIT_YAW,
    .output_limit   = PID_OUTPUT_LIMIT_YAW
};

void motor_thrust(uint16_t thrust, uint8_t motor_index)
{
    if (thrust > 0) {
        thrust = CLAMP(thrust, MIN_THURST_VALUE, THURST_VALUE_90);
    }

    uint32_t duty = ((uint32_t)thrust * MAX_PWM_DUTY_CYCLE) / THURST_VALUE_90;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, motors[motor_index].ch, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, motors[motor_index].ch));
}

void pid_reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->p_out = pid->i_out = pid->d_out = pid->out = 0.0f;
}

float pid_update(PID_t *pid, float target, float current, float rate, float dt)
{
    float error = target - current;

    pid->p_out = pid->kp * error;

    pid->integral += pid->ki * error * dt;
    pid->integral  = CLAMP(pid->integral, -pid->integral_limit, pid->integral_limit);
    pid->i_out     = pid->integral;

    pid->d_out = -pid->kd * rate;

    pid->out = CLAMP(pid->p_out + pid->i_out + pid->d_out, -pid->output_limit, pid->output_limit);
    return pid->out;
}

Motor_thrust_t pid_control(float target_roll, float target_pitch, float target_yaw_rate,
                           float current_roll, float current_pitch,
                           float roll_rate, float pitch_rate, float yaw_rate,
                           uint16_t base_thrust, float dt)
{
    Motor_thrust_t out;

    if (base_thrust < MIN_THURST_VALUE) {
        pid_reset(&roll_pid);
        pid_reset(&pitch_pid);
        pid_reset(&yaw_pid);
        out.m1 = out.m2 = out.m3 = out.m4 = 0u;
        return out;
    }

    float roll_corr  = pid_update(&roll_pid,  target_roll,     current_roll,  roll_rate,  dt);
    float pitch_corr = pid_update(&pitch_pid, target_pitch,    current_pitch, pitch_rate, dt);
    float yaw_corr   = pid_update(&yaw_pid,   target_yaw_rate, yaw_rate,      0.0f,       dt);

#if !YAW_MIX_ENABLED
    yaw_corr = 0.0f;
#endif

    float m[4];
    m[0] = (float)base_thrust - roll_corr - pitch_corr + yaw_corr;
    m[1] = (float)base_thrust + roll_corr - pitch_corr - yaw_corr;
    m[2] = (float)base_thrust - roll_corr + pitch_corr - yaw_corr;
    m[3] = (float)base_thrust + roll_corr + pitch_corr + yaw_corr;

    // In saturation, move all motors to preserve the difference between them
    float mx = m[0], mn = m[0];
    for (int i = 1; i < 4; i++) {
        if (m[i] > mx) mx = m[i];
        if (m[i] < mn) mn = m[i];
    }
    if (mx > THURST_VALUE_90) {
        float d = mx - THURST_VALUE_90;
        for (int i = 0; i < 4; i++) m[i] -= d;
    }
    if (mn < MIN_THURST_VALUE) {
        float d = MIN_THURST_VALUE - mn;
        for (int i = 0; i < 4; i++) m[i] += d;
    }

    out.m1 = (uint16_t)CLAMP(m[0], MIN_THURST_VALUE, THURST_VALUE_90);
    out.m2 = (uint16_t)CLAMP(m[1], MIN_THURST_VALUE, THURST_VALUE_90);
    out.m3 = (uint16_t)CLAMP(m[2], MIN_THURST_VALUE, THURST_VALUE_90);
    out.m4 = (uint16_t)CLAMP(m[3], MIN_THURST_VALUE, THURST_VALUE_90);

    return out;
}