#include "servo.h"

int servo_set_pulse(const struct servo_dt_spec *servo, uint32_t pulse_width)
{
    return pwm_set_pulse_dt(&servo->pwm, pulse_width);
}