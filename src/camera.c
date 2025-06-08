#include "camera.h"
#include <zephyr/drivers/gpio.h>

#define CAMERA_PORT "GPIO_0"
#define CAMERA_PIN 28
static const struct device *camera_dev = NULL;

int camera_init(void) {
    camera_dev = device_get_binding(CAMERA_PORT);
    if (!camera_dev) {
        printk("Camera GPIO device not found!\n");
        return -1;
    }
    int ret = gpio_pin_configure(camera_dev, CAMERA_PIN, GPIO_OUTPUT_HIGH); // P-MOSFET: HIGH = OFF
    if (ret != 0) {
        printk("Failed to configure camera pin!\n");
        return -1;
    }
    return 0;
}

void camera_on(void) {
    // P-MOSFET: LOW = ON
    gpio_pin_set(camera_dev, CAMERA_PIN, 0);
}

void camera_off(void) {
    // P-MOSFET: HIGH = OFF
    gpio_pin_set(camera_dev, CAMERA_PIN, 1);
}
