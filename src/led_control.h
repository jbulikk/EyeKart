#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>

#define LED0_NODE DT_ALIAS(led0)

int led_init(void);
int led_toggle(void);

#endif // LED_CONTROL_H