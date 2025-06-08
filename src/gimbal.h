#ifndef GIMBAL_H
#define GIMBAL_H

int gimbal_init(void);

int gimbal_set_yaw(float angle_deg);

int gimbal_set_pitch(float angle_deg);

#endif // GIMBAL_H