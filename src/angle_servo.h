#ifndef ANGLE_SERVO_H
#define ANGLE_SERVO_H

#include "servo.h"

typedef struct angle_servo_t
{
    servo_dt_spec servo;
    float min_angle;
    float max_angle;
    float ratio; // (max_pulse - min_pulse) / (max_angle - min_angle)
} angle_servo_t;

int angle_servo_init(angle_servo_t *as, const servo_dt_spec *servo, float min_angle, float max_angle);
int angle_servo_set_angle(const angle_servo_t *as, float angle_deg);

#endif // ANGLE_SERVO_H
