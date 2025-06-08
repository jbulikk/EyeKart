#include "gimbal.h"
#include "angle_servo.h"
#include "servo.h"

#define SERVO_YAW_NODE DT_ALIAS(servo_yaw)
#define SERVO_PITCH_NODE DT_ALIAS(servo_pitch)

static const servo_dt_spec servo_yaw = SERVO_DT_SPEC_GET(SERVO_YAW_NODE);
static const servo_dt_spec servo_pitch = SERVO_DT_SPEC_GET(SERVO_PITCH_NODE);

angle_servo_t gimbal_yaw;
angle_servo_t gimbal_pitch;

int gimbal_init(void) {
    if (!angle_servo_init(&gimbal_yaw, &servo_yaw, -95.0f, 95.0f)) {
        return 0;
    }
    if (!angle_servo_init(&gimbal_pitch, &servo_pitch, -95.0f, 95.0f)) {
        return 0;
    }
    return 1;
}

int gimbal_set_yaw(float angle_deg) { return angle_servo_set_angle(&gimbal_yaw, angle_deg); }

int gimbal_set_pitch(float angle_deg) { return angle_servo_set_angle(&gimbal_pitch, angle_deg); }
