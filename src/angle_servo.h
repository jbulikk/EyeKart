#ifndef ANGLE_SERVO_H
#define ANGLE_SERVO_H

#include "servo.h"

// High-level abstraction for angle-calibrated servo

typedef struct angle_servo_t
{
    servo_dt_spec servo;
    float min_angle; // degrees
    float max_angle; // degrees
    float ratio;     // (max_pulse - min_pulse) / (max_angle - min_angle)
} angle_servo_t;

// Initialize an angle_servo_t from a servo_t and angle range
int angle_servo_init(angle_servo_t *as, const servo_dt_spec *servo, float min_angle, float max_angle);

// Set the angle (degrees) for the angle_servo
int angle_servo_set_angle(const angle_servo_t *as, float angle_deg);

#endif // ANGLE_SERVO_H
