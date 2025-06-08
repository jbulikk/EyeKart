#ifndef CAMERA_H
#define CAMERA_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

int camera_init(void);
void camera_on(void);
void camera_off(void);

#endif // CAMERA_H
