#include "control.h"
#include "soc.h"

PID_t roll_pid = {
    .kp = 200.0f,
    .ki = 60.0f,
    .kd = 25.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .integral_limit = 50.0f,
    .output_limit = 8000.0f
};

PID_t pitch_pid = {
    .kp = 200.0f,
    .ki = 60.0f,
    .kd = 25.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .integral_limit = 50.0f,
    .output_limit = 8000.0f
};

PID_t yaw_pid = {
    .kp = 300.0f,
    .ki = 30.0f,
    .kd = 0.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .integral_limit = 80.0f,
    .output_limit = 6000.0f
};

void motor_thrust(uint16_t thrust, uint8_t motor_index)
{
    if (thrust > THURST_VALUE_90) {
        thrust = THURST_VALUE_90;
    }

    // Convert thrust to PWM duty cycle (10-bit resolution)
    uint32_t duty = ((uint32_t)thrust * MAX_PWM_DUTY_CYCLE) / THURST_VALUE_90;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, motors[motor_index].ch, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, motors[motor_index].ch));
    
    // printf("Duty m%d: %d\n", motor_index, duty);

}

void pid_reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update (PID_t *pid, float target, float current, float dt)
{
    float error = target - current; 
    pid->integral += error * dt;
    pid->integral = CLAMP(pid->integral, -pid->integral_limit, pid->integral_limit);

    float derivative = (error - pid->prev_error) / dt;

    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    output = CLAMP(output, -pid->output_limit, pid->output_limit);

    pid->prev_error = error;

    return output;
}

Motor_thrust_t pid_control( float target_roll, float target_pitch, float target_yaw,
                            float current_roll, float current_pitch, float current_yaw,
                            uint16_t base_thrust, float dt)
{
    Motor_thrust_t motors;


    if (base_thrust < MIN_THURST_VALUE) {
        pid_reset(roll_pid);
        pid_reset(pitch_pid);
        pid_reset(yaw_pid);
        motors.m1 = motors.m2 = motors.m3 = motors.m4 = 0u;
        return motors;
    }

    float roll_corr = pid_update(&roll_pid, target_roll, current_roll, dt);
    float pitch_corr = pid_update(&pitch_pid, target_pitch, current_pitch, dt);
    float yaw_corr = pid_update(&yaw_pid, target_yaw, current_yaw, dt);

    motors.m1 = CLAMP(base_thrust + roll_corr + pitch_corr - yaw_corr, 0, MAX_THURST_VALUE);
    motors.m2 = CLAMP(base_thrust - roll_corr + pitch_corr + yaw_corr, 0, MAX_THURST_VALUE);
    motors.m3 = CLAMP(base_thrust + roll_corr - pitch_corr + yaw_corr, 0, MAX_THURST_VALUE);
    motors.m4 = CLAMP(base_thrust - roll_corr - pitch_corr - yaw_corr, 0, MAX_THURST_VALUE);

    return motors;
}