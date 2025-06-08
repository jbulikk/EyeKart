#include "ICM20948.h"
#include "bt_central.h"
#include "button_control.h"
#include "led_control.h"
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define I2C_DEV_LABEL "I2C_0"

static ImuData imu;

static float roll_offset = 0.0f;
static float pitch_offset = 0.0f;
static float yaw_offset = 0.0f;

static void led_timer_callback(struct k_timer *timer_id) {
    led_toggle();
}
K_TIMER_DEFINE(led_timer, led_timer_callback, NULL);

static void on_connected(void) { printk("Connected to peripheral.\n"); }

static void filter_angles_inplace(float *pitch, float *roll) {
    const float alpha = 0.2f;
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
    pitch = imu.pitch_complementary - pitch_offset;
    roll = imu.roll_complementary - roll_offset;
    yaw = imu.yaw_complementary - yaw_offset;
    irq_unlock(key);
    snprintf(message, sizeof(message), "%d; %d; %d", (int)(pitch * 100), (int)(roll * 100), (int)(yaw * 100));

    printk("Sending %s\n", message);
    bt_central_send(message);
}
K_TIMER_DEFINE(send_angle_timer, send_angle_callback, NULL);

static void button_angle_offset_callback(void) {
    roll_offset = imu.roll_complementary;
    pitch_offset = imu.pitch_complementary;
    yaw_offset = imu.yaw_complementary;
    printk("Offsets set: roll=%.2f, pitch=%.2f, yaw=%.2f\n", roll_offset, pitch_offset, yaw_offset);
}

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

    button_init();
    button_set_callback(button_angle_offset_callback);

    bt_central_init(on_connected);
    k_timer_start(&send_angle_timer, K_MSEC(50), K_MSEC(50));

    printk("Start loop...\n");

    while (1) {
        unsigned int key = irq_lock();
        icm_read_data_and_calculate_angle(i2c_dev, &imu);
        irq_unlock(key);
        k_sleep(K_SECONDS(0.01));
    }
}
