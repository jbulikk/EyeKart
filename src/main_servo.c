#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "ICM20948.h"
#include "bt_peripheral.h"
#include "gimbal.h"
#include "led_control.h"

// BT PERIPHERAL
static void on_receive(const char *data) {
    float pitch = 0.0f, yaw = 0.0f;
    printk("Peripheral received: %s\n", data);
    const char *sep = strchr(data, ';');
    if (sep) {
        char pitch_str[16] = {0};
        char yaw_str[16] = {0};
        size_t len = sep - data;
        if (len > 0 && len < sizeof(pitch_str)) {
            memcpy(pitch_str, data, len);
            strncpy(yaw_str, sep + 1, sizeof(yaw_str) - 1);
            pitch = strtof(pitch_str, NULL);
            yaw = strtof(yaw_str, NULL);
            gimbal_set_pitch(pitch);
            gimbal_set_yaw(yaw);
            return;
        }
    }
    printk("Peripheral received (unparsed): %s\n", data);
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
    gimbal_set_yaw(0.0f);
    gimbal_set_pitch(0.0f);

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
