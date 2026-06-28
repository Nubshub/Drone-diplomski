#ifndef _CONTROL_H__
#define _CONTROL_H__

#include <stdio.h>
#include <inttypes.h>
#include "soc.h"

#define MAX_THURST_VALUE            65535     /*!< Maximum thrust value for motor control is 65535 */
#define THURST_VALUE_90             58500      /*!< Maximum thrust value to avoid overloading motors */
#define KP_LEVELING                 20       /*!< Proportional gain for roll/pitch leveling — tune this */
#define KD_LEVELING                 0.02      /*!< Derivative gain (gyro damping) — tune this */
#define CLAMP(x, lo, hi)            ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

#define MOTOR_FRONT_LEFT            0
#define MOTOR_FRONT_RIGHT           1
#define MOTOR_BACK_LEFT             2
#define MOTOR_BACK_RIGHT            3

void motor_thrust(uint16_t thrust, uint8_t motor_index);

#endif