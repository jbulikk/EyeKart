#include "ICM20948.h"
#include "bt_central.h"
#include "led_control.h"
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define I2C_DEV_LABEL "I2C_0"

static ImuData imu;

static void led_timer_callback(struct k_timer *timer_id) {
    led_toggle();
    // bt_central_send("Hello from timer");
}
K_TIMER_DEFINE(led_timer, led_timer_callback, NULL);

// BT CENTRAL
static void on_connected(void) { printk("Connected to peripheral.\n"); }

// Filtering function for pitch and roll
static void filter_angles_inplace(float *pitch, float *roll) {
    const float alpha = 0.2f; // Smoothing factor (0 < alpha <= 1)
    static float prev_pitch = 0.0f;
    static float prev_roll = 0.0f;
    prev_pitch = alpha * (*pitch) + (1.0f - alpha) * prev_pitch;
    prev_roll = alpha * (*roll) + (1.0f - alpha) * prev_roll;
    *pitch = prev_pitch;
    *roll = prev_roll;
}

static void send_angle_callback(struct k_timer *timer_id) {
    char message[24];
    float pitch, roll, yaw;
    unsigned int key = irq_lock();
    pitch = imu.pitch_complementary;
    roll = imu.roll_complementary;
    yaw = imu.yaw_complementary;
    irq_unlock(key);
    // filter_angles_inplace(&pitch, &roll);
    snprintf(message, sizeof(message), "%d; %d; %d", (int)(pitch * 100), (int)(roll * 100), (int)(yaw * 100));

    printk("Sending %s\n", message);
    bt_central_send(message); // Send the whole string, not a single char
}
K_TIMER_DEFINE(send_angle_timer, send_angle_callback, NULL);

int main(void) {
    led_init();
    k_timer_start(&led_timer, K_SECONDS(1), K_SECONDS(1));

    printk("Start of main\n");

    const struct device *i2c_dev = device_get_binding(I2C_DEV_LABEL);
    if (!i2c_dev) {
        printk("I2C device not found: %s\n", I2C_DEV_LABEL);
        return 0;
    }

    icm_read_whoami(i2c_dev);

    printk("Initializing ICM-20948...\n");
    icm_init(i2c_dev);

    printk("Calibrating gyro...\n");
    icm_calibrate_gyro(i2c_dev, 500);
    printk("Gyro calibrated\n");

    bt_central_init(on_connected);
    k_timer_start(&send_angle_timer, K_MSEC(50), K_MSEC(50));

    printk("Start loop...\n");

    while (1) {

        unsigned int key = irq_lock();
        icm_read_acc_mag_temp(i2c_dev, &imu);
        irq_unlock(key);
        k_sleep(K_SECONDS(0.01));
    }
}
