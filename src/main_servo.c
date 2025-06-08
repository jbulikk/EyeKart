#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>

#include "ICM20948.h"
#include "bt_peripheral.h"
#include "gimbal.h"
#include "led_control.h"

// BT PERIPHERAL
static void on_receive(const char *data) {
    float pitch = 0.0f, yaw = 0.0f;
    if (sscanf(data, "%f, %f", &pitch, &yaw) == 2) {
        gimbal_set_pitch(pitch);
        gimbal_set_yaw(yaw);
        printk("Received pitch: %.2f, yaw: %.2f\n", pitch, yaw);
    } else {
        printk("Peripheral received (unparsed): %s\n", data);
    }
}

static void led_timer_callback(struct k_timer *timer_id) {
    led_toggle();
    // bt_central_send("Hello from timer");
}
K_TIMER_DEFINE(led_timer, led_timer_callback, NULL);

int main(void) {
    int result = 0;
    result = led_init();

    if (result != 0) {
        printk("LED initialization failed with error code: %d\n", result);
        return result;
    }

    k_timer_start(&led_timer, K_SECONDS(1), K_SECONDS(1));
    result = gimbal_init();

    if (result != 0) {
        printk("Gimbal initialization failed with error code: %d\n", result);
        return result;
    }

    bt_peripheral_init(on_receive);
    if (result != 0) {
        printk("Bluetooth peripheral initialization failed with error code: %d\n", result);
        return result;
    }

    printk("Initialization completed!\n");
}
