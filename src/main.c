/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM-based servomotor control
 */

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "gimbal.h"

enum direction {
    DOWN,
    UP,
};

// Timer example
static void led_timer_callback(struct k_timer *timer_id) {
    led_toggle();
    // bt_central_send("Hello from timer");
}
K_TIMER_DEFINE(led_timer, led_timer_callback, NULL);

// BT PERIPHERAL
static void on_receive(const char *data) { printk("Peripheral received: %s\n", data); }

// BT CENTRAL
static void on_connected(void) {
    printk("Connected to peripheral, sending data...\n");
    // bt_central_send("Hello Peripheral!");
    // printk("Data 1 sent\n");
    // bt_central_send("DZIALA");
    // printk("Data 2 sent\n");
}

// Simple and safe gimbal RTOS task
void gimbal_task_thread(void *arg1, void *arg2, void *arg3) {
    bool up = true;
    while (1) {
        if (up) {
            gimbal_set_pitch(-45);
            gimbal_set_yaw(60);
            printk("[GIMBAL TASK] state 0\n");
        } else {
            gimbal_set_pitch(-45);
            gimbal_set_yaw(-60);
            printk("[GIMBAL TASK] state 1\n");
        }
        up = !up;
        k_sleep(K_SECONDS(1));
    }
}

#define GIMBAL_STACK_SIZE 512
#define GIMBAL_PRIORITY 5
K_THREAD_STACK_DEFINE(gimbal_stack, GIMBAL_STACK_SIZE);
static struct k_thread gimbal_thread_data;

int main(void) {
    printk("Start of main\n");
    led_init();
    // bt_peripheral_init(on_receive);
    bt_central_init(on_connected);
    gimbal_init();

    // Start the LED toggle timer to fire every 1 second
    k_timer_start(&led_timer, K_SECONDS(1), K_SECONDS(1));

    // Start the gimbal RTOS task/thread
    k_thread_create(&gimbal_thread_data,
                    gimbal_stack,
                    GIMBAL_STACK_SIZE,
                    gimbal_task_thread,
                    NULL,
                    NULL,
                    NULL,
                    GIMBAL_PRIORITY,
                    0,
                    K_NO_WAIT);

    enum direction dir = UP;
    int ret;

    printk("Servomotor control\n");

    int i = 0;
    char buf[32];
    while (1) {
        snprintf(buf, sizeof(buf), "ABC %d", i);
        printk("Sending: %s\n", buf);
        bt_central_send(buf);
        i++;
        k_sleep(K_MSEC(5000));
    }
    return 0;
}
