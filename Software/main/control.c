#include "control.h"

void motor_thrust(uint16_t thrust, uint8_t motor_index)
{
    if (thrust > THURST_VALUE_90) {
        thrust = THURST_VALUE_90;
    }

    // Convert thrust to PWM duty cycle (10-bit resolution)
    uint32_t duty = ((uint32_t)thrust * MAX_PWM_DUTY_CYCLE) / THURST_VALUE_90;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, motors[motor_index].ch, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, motors[motor_index].ch));

}
