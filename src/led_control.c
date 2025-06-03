#include "led_control.h"
#include <zephyr/sys/printk.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static bool led_initialized = false;

int led_init(void)
{
    if (!gpio_is_ready_dt(&led)) {
        printk("LED device not ready\n");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Failed to configure LED pin\n");
        return ret;
    }

    led_initialized = true;
    return 0;
}

int led_toggle(void)
{
    if (!led_initialized) {
        printk("LED not initialized\n");
        return -EIO;
    }

    return gpio_pin_toggle_dt(&led);
}