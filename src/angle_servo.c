#include "angle_servo.h"

int angle_servo_init(angle_servo_t *as, const servo_dt_spec *servo, float min_angle, float max_angle)
{
    if (!servo_is_ready_dt(servo))
        return -1;

    as->servo = *servo;
    as->min_angle = min_angle;
    as->max_angle = max_angle;
    float range_angle = max_angle - min_angle;
    float range_pulse = (float)(servo->max_pulse - servo->min_pulse);
    if (0.0f == range_angle)
        as->ratio = 0.0f;
    else
        as->ratio = (range_pulse / range_angle);
    return 0;
}

int angle_servo_set_angle(const angle_servo_t *as, float angle_deg)
{
    if (angle_deg < as->min_angle)
        angle_deg = as->min_angle;
    if (angle_deg > as->max_angle)
        angle_deg = as->max_angle;
    uint32_t pulse = as->servo.min_pulse + (uint32_t)((angle_deg - as->min_angle) * as->ratio);
    return servo_set_pulse(&as->servo, pulse);
}
