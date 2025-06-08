#ifndef SERVO_H
#define SERVO_H

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <stdint.h>

typedef struct servo_dt_spec
{
    struct pwm_dt_spec pwm;
    uint32_t min_pulse;
    uint32_t max_pulse;
} servo_dt_spec;

#define SERVO_DT_SPEC_GET(node_id)                \
    {                                             \
        .pwm = PWM_DT_SPEC_GET(node_id),          \
        .min_pulse = DT_PROP(node_id, min_pulse), \
        .max_pulse = DT_PROP(node_id, max_pulse), \
    }

static inline bool servo_is_ready_dt(const struct servo_dt_spec *spec)
{
    return pwm_is_ready_dt(&spec->pwm);
}

int servo_set_pulse(const struct servo_dt_spec *servo, uint32_t pulse_width);

#endif // SERVO_H
