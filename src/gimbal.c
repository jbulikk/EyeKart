#include "gimbal.h"
#include "angle_servo.h"
#include "servo.h"

#define SERVO_YAW_NODE DT_ALIAS(servo_yaw)
#define SERVO_PITCH_NODE DT_ALIAS(servo_pitch)

static const servo_dt_spec servo_yaw = SERVO_DT_SPEC_GET(SERVO_YAW_NODE);
static const servo_dt_spec servo_pitch = SERVO_DT_SPEC_GET(SERVO_PITCH_NODE);

angle_servo_t gimbal_yaw;
angle_servo_t gimbal_pitch;

typedef const struct limits_t {
    float min_angle; // degrees
    float max_angle; // degrees
} limits_t;

static const limits_t yaw_limits = {.min_angle = -45.0f, .max_angle = 45.0f};

static const limits_t pitch_limits = {.min_angle = -15.0f, .max_angle = 15.0f};

int gimbal_init(void) {
    if (angle_servo_init(&gimbal_yaw, &servo_yaw, -95.0f, 95.0f) != 0) {
        return -1;
    }
    if (angle_servo_init(&gimbal_pitch, &servo_pitch, -95.0f, 95.0f) != 0) {
        return -1;
    }
    return 0;
}

int gimbal_set_yaw(float angle_deg) {
    if (angle_deg < yaw_limits.min_angle)
        angle_deg = yaw_limits.min_angle;
    if (angle_deg > yaw_limits.max_angle)
        angle_deg = yaw_limits.max_angle;
    return angle_servo_set_angle(&gimbal_yaw, angle_deg);
}

int gimbal_set_pitch(float angle_deg) {
    if (angle_deg < pitch_limits.min_angle)
        angle_deg = pitch_limits.min_angle;
    if (angle_deg > pitch_limits.max_angle)
        angle_deg = pitch_limits.max_angle;
    return angle_servo_set_angle(&gimbal_pitch, angle_deg);
}
