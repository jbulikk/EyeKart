#include "button_control.h"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define BUTTON_NODE DT_ALIAS(sw0)

static struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(BUTTON_NODE, gpios, {0});
static struct gpio_callback button_cb_data;
static void (*button_user_callback)(void) = NULL;

static void button_irq_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (button_user_callback) {
        button_user_callback();
    }
}

void button_set_callback(void (*cb)(void)) { button_user_callback = cb; }

int button_init(void) {
    if (!device_is_ready(button.port)) {
        printk("Button GPIO device not ready!\n");
        return -1;
    }
    int ret = gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
    if (ret != 0) {
        printk("Failed to configure button pin!\n");
        return ret;
    }
    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("Failed to configure button interrupt!\n");
        return ret;
    }
    gpio_init_callback(&button_cb_data, button_irq_handler, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    return 0;
}

bool button_is_pressed(void) {
    if (!device_is_ready(button.port))
        return false;
    return gpio_pin_get_dt(&button) == 0; // Active low with pull-up
}
