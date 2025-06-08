#ifndef BUTTON_CONTROL_H
#define BUTTON_CONTROL_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>

int button_init(void);
bool button_is_pressed(void);

#endif // BUTTON_CONTROL_H
