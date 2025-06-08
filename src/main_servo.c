#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "ICM20948.h"
#include "bt_peripheral.h"
#include "gimbal.h"
#include "led_control.h"

static void on_receive(const char *data) {
    int pitch_i = 0, roll_i = 0, yaw_i = 0;
    printk("Peripheral received: %s\n", data);
    if (sscanf(data, "%d;%d;%d", &pitch_i, &roll_i, &yaw_i) == 3) {
        float pitch = pitch_i / 100.0f;
        float roll = roll_i / 100.0f;
        float yaw = yaw_i / 100.0f;
        gimbal_set_pitch(-pitch);
        // roll is received but not used for gimbal
        gimbal_set_yaw(roll);
        return;
    }
    printk("Peripheral received (unparsed): %s\n", data);
}

static void led_timer_callback(struct k_timer *timer_id) {
    led_toggle();
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
